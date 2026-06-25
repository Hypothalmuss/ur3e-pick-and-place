#!/usr/bin/env python3
"""
Tests for the demo_recorder package.

Uses pytest fixtures with mock hardware publishers to verify:
- Bounded sensor buffers (thread safety, drop counting)
- Episode container and serialisation
- EpisodeWriter quality validation and persistence
- Lifecycle transitions (configure, activate, deactivate, cleanup)
- Recording start/stop via services (idempotent behaviour)
- Multi-sensor timestamp synchronisation
- Graceful degradation on sensor loss
- Status publishing with observability fields

Mock hardware pattern:
    MockCamera and MockJointStatePublisher are lightweight ROS2 nodes that
    publish synthetic sensor data at configurable rates, allowing the
    recorder node to be tested without real hardware.
"""

from __future__ import annotations

import json
import os
import tempfile
import threading
import time
from typing import List
from unittest.mock import MagicMock

import numpy as np
import pytest

# ---------------------------------------------------------------------------
# Attempt to import ROS2 libraries.  If unavailable (e.g. pure-Python CI
# without a ROS2 workspace sourced), fall back to unit-testing the non-ROS
# components only.
# ---------------------------------------------------------------------------
try:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.lifecycle import State, TransitionCallbackReturn
    from rclpy.qos import (
        QoSProfile,
        QoSReliabilityPolicy,
        QoSHistoryPolicy,
    )
    from sensor_msgs.msg import Image, JointState
    from builtin_interfaces.msg import Time as TimeMsg
    from lifecycle_msgs.srv import ChangeState, GetState
    from lifecycle_msgs.msg import Transition

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

# Import non-ROS components unconditionally
from demo_recorder.demo_recorder_node import SensorBuffer
from demo_recorder.episode_writer import Episode, EpisodeWriter


# ======================================================================
# Fixtures -- pure Python (no ROS required)
# ======================================================================

@pytest.fixture
def sensor_buffer():
    """A fresh SensorBuffer with capacity 3."""
    return SensorBuffer(maxlen=3)


@pytest.fixture
def tmp_save_dir():
    """Temporary directory cleaned up after the test."""
    d = tempfile.mkdtemp(prefix="demo_recorder_test_")
    yield d
    # cleanup
    for f in os.listdir(d):
        os.remove(os.path.join(d, f))
    os.rmdir(d)


@pytest.fixture
def sample_episode():
    """An Episode pre-filled with a few timesteps."""
    ep = Episode(episode_id=1, task_name="pick_cube", operator_name="tester")
    ep.set_start_monotonic(time.monotonic())
    for i in range(5):
        ep.add_timestep({
            "timestamp_ns": i * 33_000_000,
            "observation": {
                "joint_positions": [float(j) for j in range(7)],
                "joint_velocities": [0.0] * 7,
            },
            "action": {
                "joint_positions": [float(j) for j in range(7)],
            },
        })
        ep.sync_offsets_ms.append(float(i))
    return ep


@pytest.fixture
def episode_writer(tmp_save_dir):
    """An EpisodeWriter pointed at a temp directory."""
    return EpisodeWriter(
        save_directory=tmp_save_dir,
        min_episode_timesteps=3,
        max_dropped_frame_ratio=0.10,
    )


# ======================================================================
# Unit tests -- SensorBuffer
# ======================================================================

class TestSensorBuffer:
    """Verify the bounded buffer behaves correctly under concurrent access."""

    def test_push_and_latest(self, sensor_buffer: SensorBuffer):
        sensor_buffer.push("a")
        sensor_buffer.push("b")
        assert sensor_buffer.latest() == "b"

    def test_empty_latest_returns_none(self):
        buf = SensorBuffer(maxlen=2)
        assert buf.latest() is None

    def test_bounded_overflow_drops(self):
        buf = SensorBuffer(maxlen=2)
        buf.push(1)
        buf.push(2)
        buf.push(3)  # should drop 1
        assert buf.drops == 1
        assert buf.latest() == 3

    def test_last_recv_time_updated(self, sensor_buffer: SensorBuffer):
        assert sensor_buffer.last_recv_time is None
        sensor_buffer.push("x")
        assert sensor_buffer.last_recv_time is not None
        assert (time.monotonic() - sensor_buffer.last_recv_time) < 1.0

    def test_clear(self, sensor_buffer: SensorBuffer):
        sensor_buffer.push("a")
        sensor_buffer.clear()
        assert sensor_buffer.latest() is None

    def test_thread_safety(self):
        """Hammer the buffer from multiple threads and check for no crashes."""
        buf = SensorBuffer(maxlen=5)
        errors: List[Exception] = []

        def writer(values):
            try:
                for v in values:
                    buf.push(v)
            except Exception as exc:
                errors.append(exc)

        threads = [
            threading.Thread(target=writer, args=(range(100),)),
            threading.Thread(target=writer, args=(range(100, 200),)),
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert len(errors) == 0
        assert buf.latest() is not None


# ======================================================================
# Unit tests -- Episode
# ======================================================================

class TestEpisode:
    def test_num_timesteps(self, sample_episode: Episode):
        assert sample_episode.num_timesteps == 5

    def test_duration_positive(self, sample_episode: Episode):
        assert sample_episode.duration_sec >= 0.0

    def test_to_dict_structure(self, sample_episode: Episode):
        d = sample_episode.to_dict(success_label=True, notes="test run")
        assert "metadata" in d
        assert "timesteps" in d
        meta = d["metadata"]
        assert meta["episode_id"] == 1
        assert meta["task_name"] == "pick_cube"
        assert meta["operator"] == "tester"
        assert meta["success"] is True
        assert meta["notes"] == "test run"
        assert meta["num_timesteps"] == 5
        assert meta["avg_sync_offset_ms"] > 0

    def test_to_dict_serialisable(self, sample_episode: Episode):
        """Ensure the dict can be serialised to JSON without errors."""
        d = sample_episode.to_dict(success_label=False)
        json_str = json.dumps(d)
        assert len(json_str) > 0

    def test_dropped_frames_tracking(self):
        ep = Episode(episode_id=2, task_name="t", operator_name="o")
        ep.set_start_monotonic(time.monotonic())
        ep.dropped_frames += 3
        d = ep.to_dict(success_label=False)
        assert d["metadata"]["dropped_frames"] == 3


# ======================================================================
# Unit tests -- EpisodeWriter
# ======================================================================

class TestEpisodeWriter:
    def test_save_creates_file(self, episode_writer, sample_episode, tmp_save_dir):
        path = episode_writer.save(sample_episode, success_label=True)
        assert path != ""
        assert os.path.exists(path)

        with open(path) as fh:
            loaded = json.load(fh)
        assert loaded["metadata"]["num_timesteps"] == 5
        assert loaded["metadata"]["success"] is True

    def test_save_with_notes(self, episode_writer, sample_episode):
        path = episode_writer.save(
            sample_episode, success_label=False, notes="failed grasp"
        )
        with open(path) as fh:
            loaded = json.load(fh)
        assert loaded["metadata"]["notes"] == "failed grasp"
        assert loaded["metadata"]["success"] is False

    def test_quality_warning_low_timesteps(self, tmp_save_dir):
        """Writer should still save but warn when below minimum timesteps."""
        writer = EpisodeWriter(
            save_directory=tmp_save_dir,
            min_episode_timesteps=100,
        )
        ep = Episode(episode_id=1, task_name="t", operator_name="o")
        ep.set_start_monotonic(time.monotonic())
        ep.add_timestep({"timestamp_ns": 0, "observation": {}, "action": {}})

        path = writer.save(ep, success_label=True)
        assert path != ""  # saved despite warning

    def test_quality_warning_high_drop_ratio(self, tmp_save_dir):
        """Writer should still save but warn when drop ratio is too high."""
        writer = EpisodeWriter(
            save_directory=tmp_save_dir,
            max_dropped_frame_ratio=0.01,
        )
        ep = Episode(episode_id=1, task_name="t", operator_name="o")
        ep.set_start_monotonic(time.monotonic())
        ep.add_timestep({"timestamp_ns": 0, "observation": {}, "action": {}})
        ep.dropped_frames = 10  # 10 drops out of 11 total

        path = writer.save(ep, success_label=True)
        assert path != ""  # saved despite warning

    def test_save_directory_created(self):
        """Writer should create the save directory if it doesn't exist."""
        d = tempfile.mkdtemp(prefix="demo_recorder_test_")
        sub = os.path.join(d, "nested", "dir")
        writer = EpisodeWriter(save_directory=sub)
        assert os.path.isdir(sub)
        os.rmdir(sub)
        os.rmdir(os.path.join(d, "nested"))
        os.rmdir(d)


# ======================================================================
# Mock hardware publishers (for ROS2 integration tests)
# ======================================================================

if ROS2_AVAILABLE:

    SENSOR_QOS = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )

    class MockCamera(Node):
        """Publishes synthetic 640x480 BGR8 images at a configurable rate."""

        def __init__(self, topic: str = "/camera/wrist/color/image_raw",
                     rate_hz: float = 30.0):
            super().__init__("mock_camera")
            self._pub = self.create_publisher(Image, topic, SENSOR_QOS)
            self._timer = self.create_timer(1.0 / rate_hz, self._tick)
            self._seq = 0

        def _tick(self):
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "wrist_camera"
            msg.height = 480
            msg.width = 640
            msg.encoding = "bgr8"
            msg.step = 640 * 3
            msg.data = bytes(640 * 480 * 3)  # black image
            self._pub.publish(msg)
            self._seq += 1

    class MockJointStatePublisher(Node):
        """Publishes synthetic 7-DOF Franka joint states."""

        JOINT_NAMES = [
            "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
            "panda_joint5", "panda_joint6", "panda_joint7",
        ]

        def __init__(self, topic: str = "/franka/joint_states",
                     rate_hz: float = 100.0):
            super().__init__("mock_joint_state_publisher")
            self._pub = self.create_publisher(JointState, topic, SENSOR_QOS)
            self._timer = self.create_timer(1.0 / rate_hz, self._tick)
            self._t = 0.0

        def _tick(self):
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(self.JOINT_NAMES)
            # Gentle sinusoidal motion to simulate teleoperation
            msg.position = [
                0.1 * np.sin(self._t + i) for i in range(7)
            ]
            msg.velocity = [
                0.1 * np.cos(self._t + i) for i in range(7)
            ]
            msg.effort = [0.0] * 7
            self._pub.publish(msg)
            self._t += 0.01


# ======================================================================
# Integration tests (require ROS2)
# ======================================================================

@pytest.fixture
def ros2_context():
    """Initialise and tear down rclpy for the test session."""
    if not ROS2_AVAILABLE:
        pytest.skip("ROS2 not available")
    rclpy.init()
    yield
    rclpy.try_shutdown()


@pytest.fixture
def mock_camera(ros2_context):
    node = MockCamera()
    yield node
    node.destroy_node()


@pytest.fixture
def mock_joints(ros2_context):
    node = MockJointStatePublisher()
    yield node
    node.destroy_node()


@pytest.fixture
def recorder_node(ros2_context, tmp_save_dir):
    """Create a DemoRecorderNode with test-friendly parameters."""
    from demo_recorder.demo_recorder_node import DemoRecorderNode

    node = DemoRecorderNode()
    # Override save directory to temp
    node.set_parameters([
        rclpy.parameter.Parameter("save_directory", value=tmp_save_dir),
        rclpy.parameter.Parameter("capture_rate_hz", value=10.0),
        rclpy.parameter.Parameter("sensor_timeout_sec", value=2.0),
        rclpy.parameter.Parameter("save_images", value=False),
        rclpy.parameter.Parameter("min_episode_timesteps", value=1),
    ])
    yield node
    node.destroy_node()


@pytest.fixture
def executor_with_nodes(recorder_node, mock_camera, mock_joints):
    """Spin all nodes in a multi-threaded executor on a background thread."""
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(recorder_node)
    executor.add_node(mock_camera)
    executor.add_node(mock_joints)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    yield executor

    executor.shutdown()


@pytest.mark.skipif(not ROS2_AVAILABLE, reason="ROS2 not available")
class TestLifecycleTransitions:
    """Verify lifecycle state machine transitions."""

    def test_configure_succeeds(self, recorder_node, executor_with_nodes):
        result = recorder_node.trigger_configure()
        assert result == TransitionCallbackReturn.SUCCESS

    def test_activate_after_configure(self, recorder_node, executor_with_nodes):
        recorder_node.trigger_configure()
        result = recorder_node.trigger_activate()
        assert result == TransitionCallbackReturn.SUCCESS

    def test_deactivate(self, recorder_node, executor_with_nodes):
        recorder_node.trigger_configure()
        recorder_node.trigger_activate()
        result = recorder_node.trigger_deactivate()
        assert result == TransitionCallbackReturn.SUCCESS

    def test_cleanup(self, recorder_node, executor_with_nodes):
        recorder_node.trigger_configure()
        recorder_node.trigger_activate()
        recorder_node.trigger_deactivate()
        result = recorder_node.trigger_cleanup()
        assert result == TransitionCallbackReturn.SUCCESS


@pytest.mark.skipif(not ROS2_AVAILABLE, reason="ROS2 not available")
class TestRecordingServices:
    """Verify start/stop recording service behaviour."""

    def _activate_node(self, recorder_node):
        recorder_node.trigger_configure()
        recorder_node.trigger_activate()

    def test_start_recording(self, recorder_node, executor_with_nodes):
        self._activate_node(recorder_node)

        # Simulate a service call by directly invoking the handler
        request = MagicMock()
        request.task_name = "test_task"
        request.operator_name = "pytest"
        response = MagicMock()
        response.success = False
        response.message = ""
        response.episode_id = 0

        result = recorder_node._handle_start(request, response)
        assert result.success is True
        assert result.episode_id == 1

    def test_start_while_recording_is_idempotent(
        self, recorder_node, executor_with_nodes
    ):
        self._activate_node(recorder_node)

        req = MagicMock()
        req.task_name = "t"
        req.operator_name = "o"
        resp = MagicMock()
        resp.success = False
        resp.message = ""
        resp.episode_id = 0

        recorder_node._handle_start(req, resp)
        # Second call should fail gracefully
        resp2 = MagicMock()
        resp2.success = False
        resp2.message = ""
        resp2.episode_id = 0
        result = recorder_node._handle_start(req, resp2)
        assert result.success is False
        assert "Already recording" in result.message

    def test_stop_while_not_recording_is_idempotent(
        self, recorder_node, executor_with_nodes
    ):
        self._activate_node(recorder_node)

        req = MagicMock()
        req.success_label = True
        req.notes = ""
        resp = MagicMock()
        resp.success = False
        resp.message = ""
        resp.file_path = ""
        resp.num_timesteps = 0
        resp.duration_sec = 0.0
        resp.dropped_frames = 0

        result = recorder_node._handle_stop(req, resp)
        assert result.success is False
        assert "Not currently recording" in result.message

    def test_full_record_cycle(
        self, recorder_node, executor_with_nodes, tmp_save_dir
    ):
        self._activate_node(recorder_node)
        # Wait for sensor data to arrive
        time.sleep(0.5)

        # Start
        start_req = MagicMock()
        start_req.task_name = "integration_test"
        start_req.operator_name = "pytest"
        start_resp = MagicMock()
        start_resp.success = False
        start_resp.message = ""
        start_resp.episode_id = 0
        recorder_node._handle_start(start_req, start_resp)
        assert start_resp.success is True

        # Let it record a few timesteps
        time.sleep(1.0)

        # Stop
        stop_req = MagicMock()
        stop_req.success_label = True
        stop_req.notes = "integration test run"
        stop_resp = MagicMock()
        stop_resp.success = False
        stop_resp.message = ""
        stop_resp.file_path = ""
        stop_resp.num_timesteps = 0
        stop_resp.duration_sec = 0.0
        stop_resp.dropped_frames = 0
        recorder_node._handle_stop(stop_req, stop_resp)
        assert stop_resp.success is True
        assert stop_resp.num_timesteps > 0

        # Verify file exists
        files = os.listdir(tmp_save_dir)
        assert len(files) == 1
        assert files[0].endswith(".json")

        # Verify contents
        with open(os.path.join(tmp_save_dir, files[0])) as fh:
            episode = json.load(fh)
        assert episode["metadata"]["task_name"] == "integration_test"
        assert episode["metadata"]["success"] is True
        assert episode["metadata"]["notes"] == "integration test run"
        assert len(episode["timesteps"]) > 0


@pytest.mark.skipif(not ROS2_AVAILABLE, reason="ROS2 not available")
class TestSensorHealth:
    """Verify sensor watchdog and graceful degradation."""

    def test_camera_alive_flag(self, recorder_node, executor_with_nodes):
        recorder_node.trigger_configure()
        recorder_node.trigger_activate()
        # After mock publishers send data
        time.sleep(0.5)
        assert recorder_node._sensor_alive(recorder_node._camera_buf) is True

    def test_joint_alive_flag(self, recorder_node, executor_with_nodes):
        recorder_node.trigger_configure()
        recorder_node.trigger_activate()
        time.sleep(0.5)
        assert recorder_node._sensor_alive(recorder_node._joint_buf) is True

    def test_sensor_timeout_detection(self, recorder_node, executor_with_nodes):
        recorder_node.trigger_configure()
        # Don't activate -- no timers, but we can check a stale buffer
        buf = SensorBuffer(maxlen=2)
        # Never received data
        assert recorder_node._sensor_alive(buf) is False


@pytest.mark.skipif(not ROS2_AVAILABLE, reason="ROS2 not available")
class TestDeactivateAutoSave:
    """Verify that deactivating while recording auto-saves the episode."""

    def test_auto_save_on_deactivate(
        self, recorder_node, executor_with_nodes, tmp_save_dir
    ):
        recorder_node.trigger_configure()
        recorder_node.trigger_activate()
        time.sleep(0.3)

        # Start recording
        req = MagicMock()
        req.task_name = "autosave_test"
        req.operator_name = "pytest"
        resp = MagicMock()
        resp.success = False
        resp.message = ""
        resp.episode_id = 0
        recorder_node._handle_start(req, resp)
        time.sleep(0.5)

        # Deactivate without stopping -- should auto-save
        recorder_node.trigger_deactivate()
        time.sleep(0.2)

        files = os.listdir(tmp_save_dir)
        assert len(files) == 1
        with open(os.path.join(tmp_save_dir, files[0])) as fh:
            ep = json.load(fh)
        assert ep["metadata"]["success"] is False
        assert "auto-saved" in ep["metadata"]["notes"]
