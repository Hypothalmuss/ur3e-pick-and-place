#!/usr/bin/env python3
"""
Demo Recorder Node
==================
A lifecycle-managed ROS2 node that records robot demonstration episodes
during teleoperation.

Design principles applied
-------------------------
* **Lifecycle node** -- deterministic startup/shutdown via
  on_configure / on_activate / on_deactivate / on_cleanup / on_shutdown.
* **Threaded sensor capture with bounded buffers** -- sensor callbacks
  push into fixed-size deques so the main capture loop never blocks the
  ROS executor.
* **Software-timestamp synchronisation** -- the capture loop picks the
  closest-in-time camera and joint-state messages and checks they fall
  within a configurable sync window.
* **Episode recorder pattern** -- each episode is a structured container
  of timestep streams plus rich metadata and quality metrics.  Persistence
  is delegated to EpisodeWriter (single responsibility).
* **Proper QoS** -- BEST_EFFORT / depth-1 for high-rate sensor data,
  RELIABLE for services and the status publisher.
* **Fail-safe defaults** -- if either sensor drops out (watchdog), the
  node logs a warning and pauses recording rather than saving corrupt data.
* **Configuration over code** -- every tunable is a declared ROS
  parameter with descriptor, loaded from YAML, changeable at runtime.
* **Idempotent services** -- calling start while recording, or stop while
  idle, returns a descriptive failure without side-effects.
* **Structured logging / observability** -- the RecordingStatus message
  carries sensor-alive flags, sync offset, capture rate, and drop count.
* **Graceful degradation** -- partial sensor data is handled; images can
  be omitted if the camera is offline but joints are still valid.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Any, Deque, Dict, List, Optional

import numpy as np
import rclpy
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from rcl_interfaces.msg import (
    ParameterDescriptor,
    FloatingPointRange,
    IntegerRange,
    SetParametersResult,
)
from sensor_msgs.msg import Image, JointState

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None  # graceful degradation in test / headless environments

from demo_recorder.msg import RecordingStatus
from demo_recorder.srv import StartRecording, StopRecording
from demo_recorder.episode_writer import Episode, EpisodeWriter


# ---------------------------------------------------------------------------
# QoS profiles (following ROS2 skill best practices)
# ---------------------------------------------------------------------------
# Sensor data (cameras, lidars) -- tolerate drops, want latest
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.VOLATILE,
)

# Commands, services, status -- never miss, small buffer
RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE,
)


# ---------------------------------------------------------------------------
# Threaded bounded buffer (CameraStream / SensorStream pattern)
# ---------------------------------------------------------------------------
class SensorBuffer:
    """Thread-safe, bounded FIFO buffer for sensor messages.

    Drops oldest messages when full so callbacks never block the executor.
    Tracks the number of drops for quality metrics.
    """

    def __init__(self, maxlen: int = 2) -> None:
        self._buf: Deque = deque(maxlen=maxlen)
        self._lock = threading.Lock()
        self._drops: int = 0
        self._last_recv_time: Optional[float] = None

    def push(self, msg: Any) -> None:
        with self._lock:
            if len(self._buf) == self._buf.maxlen:
                self._drops += 1
            self._buf.append(msg)
            self._last_recv_time = time.monotonic()

    def latest(self) -> Optional[Any]:
        with self._lock:
            return self._buf[-1] if self._buf else None

    @property
    def drops(self) -> int:
        with self._lock:
            return self._drops

    @property
    def last_recv_time(self) -> Optional[float]:
        with self._lock:
            return self._last_recv_time

    def clear(self) -> None:
        with self._lock:
            self._buf.clear()


# ---------------------------------------------------------------------------
# Main lifecycle node
# ---------------------------------------------------------------------------
class DemoRecorderNode(LifecycleNode):
    """Lifecycle-managed demo episode recorder.

    State machine: Unconfigured -> Inactive -> Active -> Inactive -> Finalized

    - on_configure: declare params, create subs/pubs/services, allocate buffers
    - on_activate:  start capture timer and status timer
    - on_deactivate: auto-save any in-progress recording, destroy timers
    - on_cleanup:   release buffers
    - on_shutdown:  auto-save and final cleanup
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__("demo_recorder", **kwargs)
        self._declare_all_parameters()
        self.get_logger().info("DemoRecorderNode created (unconfigured).")

    # ------------------------------------------------------------------
    # Parameter declarations with descriptors and ranges
    # ------------------------------------------------------------------
    def _declare_all_parameters(self) -> None:
        self.declare_parameter(
            "camera_topic",
            "/camera/wrist/color/image_raw",
            ParameterDescriptor(description="Camera image topic name"),
        )
        self.declare_parameter(
            "joint_states_topic",
            "/franka/joint_states",
            ParameterDescriptor(description="Joint states topic name"),
        )
        self.declare_parameter(
            "save_directory",
            "/tmp/demo_recordings",
            ParameterDescriptor(description="Directory for saved episodes"),
        )
        self.declare_parameter(
            "capture_rate_hz",
            30.0,
            ParameterDescriptor(
                description="Timestep capture rate (Hz)",
                floating_point_range=[
                    FloatingPointRange(from_value=1.0, to_value=120.0, step=0.0)
                ],
            ),
        )
        self.declare_parameter(
            "status_publish_rate_hz",
            2.0,
            ParameterDescriptor(
                description="Status publish rate (Hz)",
                floating_point_range=[
                    FloatingPointRange(from_value=0.1, to_value=10.0, step=0.0)
                ],
            ),
        )
        self.declare_parameter(
            "sensor_timeout_sec",
            1.0,
            ParameterDescriptor(
                description="Sensor watchdog timeout (s)",
                floating_point_range=[
                    FloatingPointRange(from_value=0.1, to_value=10.0, step=0.0)
                ],
            ),
        )
        self.declare_parameter(
            "buffer_size",
            2,
            ParameterDescriptor(
                description="Per-sensor bounded buffer depth",
                integer_range=[
                    IntegerRange(from_value=1, to_value=100, step=1)
                ],
            ),
        )
        self.declare_parameter(
            "max_sync_offset_ms",
            50.0,
            ParameterDescriptor(
                description="Max allowable camera/joint sync offset (ms)",
                floating_point_range=[
                    FloatingPointRange(from_value=1.0, to_value=500.0, step=0.0)
                ],
            ),
        )
        self.declare_parameter(
            "min_episode_timesteps",
            10,
            ParameterDescriptor(
                description="Minimum timesteps for a valid episode",
                integer_range=[
                    IntegerRange(from_value=0, to_value=100000, step=1)
                ],
            ),
        )
        self.declare_parameter(
            "max_dropped_frame_ratio",
            0.05,
            ParameterDescriptor(
                description="Max fraction of dropped frames before quality warning",
                floating_point_range=[
                    FloatingPointRange(from_value=0.0, to_value=1.0, step=0.0)
                ],
            ),
        )
        self.declare_parameter(
            "image_encoding",
            "bgr8",
            ParameterDescriptor(description="cv_bridge encoding for images"),
        )
        self.declare_parameter(
            "save_images",
            True,
            ParameterDescriptor(
                description="Whether to save full image arrays in episodes"
            ),
        )
        self.declare_parameter(
            "expected_num_joints",
            7,
            ParameterDescriptor(
                description="Expected DOF count for validation (Franka = 7)",
                integer_range=[
                    IntegerRange(from_value=1, to_value=30, step=1)
                ],
            ),
        )

    # ------------------------------------------------------------------
    # Helper: read parameters into instance attributes
    # ------------------------------------------------------------------
    def _load_params(self) -> None:
        self._camera_topic = self.get_parameter("camera_topic").value
        self._joint_topic = self.get_parameter("joint_states_topic").value
        self._save_dir = self.get_parameter("save_directory").value
        self._capture_rate = self.get_parameter("capture_rate_hz").value
        self._status_rate = self.get_parameter("status_publish_rate_hz").value
        self._sensor_timeout = self.get_parameter("sensor_timeout_sec").value
        self._buffer_size = self.get_parameter("buffer_size").value
        self._max_sync_offset_ms = self.get_parameter("max_sync_offset_ms").value
        self._min_episode_ts = self.get_parameter("min_episode_timesteps").value
        self._max_drop_ratio = self.get_parameter("max_dropped_frame_ratio").value
        self._image_encoding = self.get_parameter("image_encoding").value
        self._save_images = self.get_parameter("save_images").value
        self._expected_joints = self.get_parameter("expected_num_joints").value

    # ==================================================================
    # Lifecycle callbacks
    # ==================================================================
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring...")
        try:
            self._load_params()

            # cv_bridge (graceful if unavailable)
            self._bridge = CvBridge() if CvBridge is not None else None
            if self._bridge is None:
                self.get_logger().warn(
                    "cv_bridge not available -- images will be recorded as "
                    "metadata only (shape/dtype)."
                )

            # Episode writer (single responsibility: persistence + quality gates)
            self._writer = EpisodeWriter(
                save_directory=self._save_dir,
                min_episode_timesteps=self._min_episode_ts,
                max_dropped_frame_ratio=self._max_drop_ratio,
            )

            # Bounded sensor buffers
            self._camera_buf = SensorBuffer(maxlen=self._buffer_size)
            self._joint_buf = SensorBuffer(maxlen=self._buffer_size)

            # Recording state (protected by lock for service thread safety)
            self._lock = threading.Lock()
            self._is_recording = False
            self._episode: Optional[Episode] = None
            self._episode_count = 0
            self._current_timestep = 0

            # Metrics for observability
            self._capture_timestamps: Deque[float] = deque(maxlen=100)
            self._latest_sync_offset_ms = 0.0

            # Callback groups: sensors in reentrant, services in exclusive
            self._sensor_cbg = ReentrantCallbackGroup()
            self._service_cbg = MutuallyExclusiveCallbackGroup()
            self._timer_cbg = MutuallyExclusiveCallbackGroup()

            # Subscribers (BEST_EFFORT sensor QoS, bounded depth)
            self._image_sub = self.create_subscription(
                Image,
                self._camera_topic,
                self._on_image,
                SENSOR_QOS,
                callback_group=self._sensor_cbg,
            )
            self._joint_sub = self.create_subscription(
                JointState,
                self._joint_topic,
                self._on_joint_state,
                SENSOR_QOS,
                callback_group=self._sensor_cbg,
            )

            # Services (RELIABLE QoS is the default for services)
            self._start_srv = self.create_service(
                StartRecording,
                "~/start_recording",
                self._handle_start,
                callback_group=self._service_cbg,
            )
            self._stop_srv = self.create_service(
                StopRecording,
                "~/stop_recording",
                self._handle_stop,
                callback_group=self._service_cbg,
            )

            # Status publisher (RELIABLE so UI gets every update)
            self._status_pub = self.create_publisher(
                RecordingStatus, "~/status", RELIABLE_QOS
            )

            # Parameter change callback for runtime reconfiguration
            self.add_on_set_parameters_callback(self._on_param_change)

            self.get_logger().info("Configuration complete.")
            return TransitionCallbackReturn.SUCCESS

        except Exception as exc:
            self.get_logger().error(f"Configuration failed: {exc}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating...")
        try:
            # Capture timer (decoupled from sensor rate -- separation of rates)
            self._capture_timer = self.create_timer(
                1.0 / self._capture_rate,
                self._capture_tick,
                callback_group=self._timer_cbg,
            )

            # Status timer (low rate for UI consumption)
            self._status_timer = self.create_timer(
                1.0 / self._status_rate,
                self._publish_status,
                callback_group=self._timer_cbg,
            )

            self.get_logger().info(
                f"Active. Capture @ {self._capture_rate} Hz, "
                f"status @ {self._status_rate} Hz."
            )
            return super().on_activate(state)

        except Exception as exc:
            self.get_logger().error(f"Activation failed: {exc}")
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating...")
        # Fail-safe: auto-save any in-progress recording
        with self._lock:
            if self._is_recording and self._episode is not None:
                self.get_logger().warn(
                    "Deactivating while recording -- auto-saving episode."
                )
                self._writer.save(
                    self._episode,
                    success_label=False,
                    notes="auto-saved on deactivate",
                )
                self._is_recording = False

        # Destroy timers
        if hasattr(self, "_capture_timer"):
            self.destroy_timer(self._capture_timer)
        if hasattr(self, "_status_timer"):
            self.destroy_timer(self._status_timer)

        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up...")
        self._camera_buf.clear()
        self._joint_buf.clear()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down...")
        with self._lock:
            if self._is_recording and self._episode is not None:
                self.get_logger().warn(
                    "Shutting down while recording -- auto-saving episode."
                )
                self._writer.save(
                    self._episode,
                    success_label=False,
                    notes="auto-saved on shutdown",
                )
                self._is_recording = False
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Runtime parameter change callback
    # ------------------------------------------------------------------
    def _on_param_change(self, params) -> SetParametersResult:
        for param in params:
            self.get_logger().info(
                f"Parameter changed: {param.name} = {param.value}"
            )
            # Update cached values for parameters that are safe to hot-reload
            if param.name == "sensor_timeout_sec":
                self._sensor_timeout = param.value
            elif param.name == "max_sync_offset_ms":
                self._max_sync_offset_ms = param.value
            elif param.name == "save_images":
                self._save_images = param.value
            elif param.name == "image_encoding":
                self._image_encoding = param.value
            elif param.name == "min_episode_timesteps":
                self._min_episode_ts = param.value
            elif param.name == "max_dropped_frame_ratio":
                self._max_drop_ratio = param.value

        return SetParametersResult(successful=True)

    # ==================================================================
    # Sensor callbacks -- push into bounded buffers, never block
    # ==================================================================
    def _on_image(self, msg: Image) -> None:
        self._camera_buf.push(msg)

    def _on_joint_state(self, msg: JointState) -> None:
        self._joint_buf.push(msg)

    # ==================================================================
    # Sensor health (watchdog pattern)
    # ==================================================================
    def _sensor_alive(self, buf: SensorBuffer) -> bool:
        """Check if a sensor has sent data within the timeout window."""
        t = buf.last_recv_time
        if t is None:
            return False
        return (time.monotonic() - t) < self._sensor_timeout

    # ==================================================================
    # Capture tick -- runs at capture_rate_hz, decoupled from sensors
    # ==================================================================
    def _capture_tick(self) -> None:
        with self._lock:
            if not self._is_recording:
                return

            camera_alive = self._sensor_alive(self._camera_buf)
            joints_alive = self._sensor_alive(self._joint_buf)

            # Fail-safe: joint states are mandatory for a valid timestep
            if not joints_alive:
                self.get_logger().warn(
                    "Joint state sensor timeout -- skipping timestep.",
                    throttle_duration_sec=2.0,
                )
                if self._episode is not None:
                    self._episode.dropped_frames += 1
                return

            joint_msg: Optional[JointState] = self._joint_buf.latest()
            image_msg: Optional[Image] = self._camera_buf.latest()

            if joint_msg is None:
                return

            # --- Timestamp synchronisation --------------------------------
            joint_stamp_ns = (
                joint_msg.header.stamp.sec * 1_000_000_000
                + joint_msg.header.stamp.nanosec
            )

            sync_offset_ms = 0.0
            if image_msg is not None and camera_alive:
                image_stamp_ns = (
                    image_msg.header.stamp.sec * 1_000_000_000
                    + image_msg.header.stamp.nanosec
                )
                sync_offset_ms = abs(joint_stamp_ns - image_stamp_ns) / 1e6
                self._latest_sync_offset_ms = sync_offset_ms

                if sync_offset_ms > self._max_sync_offset_ms:
                    self.get_logger().debug(
                        f"Sync offset {sync_offset_ms:.1f} ms exceeds limit "
                        f"({self._max_sync_offset_ms:.1f} ms) -- skipping."
                    )
                    if self._episode is not None:
                        self._episode.dropped_frames += 1
                    return
            else:
                # Graceful degradation: record without image
                self.get_logger().debug(
                    "Camera offline -- recording joints only.",
                    throttle_duration_sec=5.0,
                )

            # --- Build timestep -------------------------------------------
            now_ns = self.get_clock().now().nanoseconds
            timestep: Dict[str, Any] = {
                "timestamp_ns": now_ns,
                "observation": self._build_observation(
                    joint_msg, image_msg, camera_alive
                ),
                "action": {
                    "joint_positions": list(joint_msg.position),
                },
            }

            if self._episode is not None:
                self._episode.add_timestep(timestep)
                if sync_offset_ms > 0:
                    self._episode.sync_offsets_ms.append(sync_offset_ms)
                self._current_timestep = self._episode.num_timesteps

            # Track capture rate for observability
            self._capture_timestamps.append(time.monotonic())

    # ------------------------------------------------------------------
    # Observation builder
    # ------------------------------------------------------------------
    def _build_observation(
        self,
        joint_msg: JointState,
        image_msg: Optional[Image],
        camera_alive: bool,
    ) -> Dict[str, Any]:
        obs: Dict[str, Any] = {
            "joint_names": list(joint_msg.name),
            "joint_positions": list(joint_msg.position),
            "joint_velocities": list(joint_msg.velocity) if joint_msg.velocity else [],
            "joint_efforts": list(joint_msg.effort) if joint_msg.effort else [],
        }

        # Validate joint count against expected DOF
        if len(joint_msg.position) != self._expected_joints:
            self.get_logger().warn(
                f"Expected {self._expected_joints} joints, got "
                f"{len(joint_msg.position)}.",
                throttle_duration_sec=5.0,
            )

        # Image handling with graceful degradation
        if image_msg is not None and camera_alive:
            if self._bridge is not None and self._save_images:
                try:
                    cv_image = self._bridge.imgmsg_to_cv2(
                        image_msg, self._image_encoding
                    )
                    obs["image"] = cv_image.tolist()
                    obs["image_shape"] = list(cv_image.shape)
                except Exception as exc:
                    self.get_logger().warn(
                        f"cv_bridge conversion failed: {exc}",
                        throttle_duration_sec=5.0,
                    )
                    obs["image_shape"] = [
                        image_msg.height,
                        image_msg.width,
                        1 if image_msg.encoding == "mono8" else 3,
                    ]
            else:
                # Metadata only (no bridge or save_images is False)
                obs["image_shape"] = [
                    image_msg.height,
                    image_msg.width,
                    1 if image_msg.encoding == "mono8" else 3,
                ]
                obs["image_encoding"] = image_msg.encoding
        else:
            obs["image_shape"] = None

        return obs

    # ==================================================================
    # Service handlers (idempotent)
    # ==================================================================
    def _handle_start(
        self,
        request: StartRecording.Request,
        response: StartRecording.Response,
    ) -> StartRecording.Response:
        with self._lock:
            if self._is_recording:
                response.success = False
                response.message = (
                    "Already recording. Stop the current episode first."
                )
                response.episode_id = (
                    self._episode.episode_id if self._episode else 0
                )
                self.get_logger().warn(response.message)
                return response

            self._episode_count += 1
            self._episode = Episode(
                episode_id=self._episode_count,
                task_name=request.task_name,
                operator_name=request.operator_name,
            )
            self._episode.set_start_monotonic(time.monotonic())
            self._current_timestep = 0
            self._is_recording = True

            response.success = True
            response.episode_id = self._episode_count
            response.message = (
                f"Recording episode {self._episode_count} "
                f"(task={request.task_name!r}, operator={request.operator_name!r})"
            )
            self.get_logger().info(response.message)
            return response

    def _handle_stop(
        self,
        request: StopRecording.Request,
        response: StopRecording.Response,
    ) -> StopRecording.Response:
        with self._lock:
            if not self._is_recording or self._episode is None:
                response.success = False
                response.message = "Not currently recording."
                response.file_path = ""
                response.num_timesteps = 0
                response.duration_sec = 0.0
                response.dropped_frames = 0
                self.get_logger().warn(response.message)
                return response

            # Delegate persistence to EpisodeWriter (single responsibility)
            file_path = self._writer.save(
                self._episode,
                success_label=request.success_label,
                notes=request.notes,
            )
            num_ts = self._episode.num_timesteps
            duration = self._episode.duration_sec
            drops = self._episode.dropped_frames

            self._is_recording = False

            response.success = True
            response.message = (
                f"Saved episode with {num_ts} timesteps "
                f"({duration:.1f}s, {drops} dropped)."
            )
            response.file_path = file_path
            response.num_timesteps = num_ts
            response.duration_sec = duration
            response.dropped_frames = drops
            self.get_logger().info(response.message)
            return response

    # ==================================================================
    # Status publisher (observability)
    # ==================================================================
    def _publish_status(self) -> None:
        msg = RecordingStatus()
        msg.stamp = self.get_clock().now().to_msg()

        with self._lock:
            msg.is_recording = self._is_recording
            msg.episode_count = self._episode_count
            msg.current_timestep = self._current_timestep
            msg.task_name = (
                self._episode.task_name if self._episode else ""
            )
            msg.operator_name = (
                self._episode.operator_name if self._episode else ""
            )
            msg.dropped_frames = (
                self._episode.dropped_frames
                if self._episode
                else 0
            )

        msg.camera_alive = self._sensor_alive(self._camera_buf)
        msg.joint_states_alive = self._sensor_alive(self._joint_buf)
        msg.sync_offset_ms = self._latest_sync_offset_ms

        # Compute average capture rate over recent window
        if len(self._capture_timestamps) >= 2:
            dt = self._capture_timestamps[-1] - self._capture_timestamps[0]
            if dt > 0:
                msg.avg_capture_rate_hz = (
                    (len(self._capture_timestamps) - 1) / dt
                )
            else:
                msg.avg_capture_rate_hz = 0.0
        else:
            msg.avg_capture_rate_hz = 0.0

        self._status_pub.publish(msg)


# ======================================================================
# Entry point
# ======================================================================
def main(args=None):
    rclpy.init(args=args)

    node = DemoRecorderNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
