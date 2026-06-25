#!/usr/bin/env python3
"""Demo Recorder Node - Records robot demonstration episodes during teleoperation."""

import json
import os
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge

from demo_recorder.msg import RecordingStatus
from demo_recorder.srv import StartRecording, StopRecording


class DemoRecorderNode(Node):
    def __init__(self):
        super().__init__('demo_recorder')

        # Parameters
        self.declare_parameter('save_directory', '/tmp/demo_recordings')
        self.declare_parameter('camera_topic', '/camera/color/image_raw')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('status_publish_rate', 1.0)

        self.save_dir = self.get_parameter('save_directory').value
        camera_topic = self.get_parameter('camera_topic').value
        joint_states_topic = self.get_parameter('joint_states_topic').value
        status_rate = self.get_parameter('status_publish_rate').value

        # State
        self.is_recording = False
        self.task_name = ''
        self.operator_name = ''
        self.episode_count = 0
        self.current_episode = []
        self.current_timestep = 0
        self.latest_image = None
        self.latest_joints = None

        # CV Bridge
        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, camera_topic, self.image_callback, 10)
        self.joint_sub = self.create_subscription(
            JointState, joint_states_topic, self.joint_callback, 10)

        # Services
        self.start_srv = self.create_service(
            StartRecording, '~/start_recording', self.start_recording_callback)
        self.stop_srv = self.create_service(
            StopRecording, '~/stop_recording', self.stop_recording_callback)

        # Publisher
        self.status_pub = self.create_publisher(
            RecordingStatus, '~/status', 10)

        # Timer for status publishing
        self.status_timer = self.create_timer(
            1.0 / status_rate, self.publish_status)

        # Timer for recording timesteps (30 Hz)
        self.record_timer = self.create_timer(1.0 / 30.0, self.record_timestep)

        os.makedirs(self.save_dir, exist_ok=True)
        self.get_logger().info('Demo Recorder Node initialized')

    def image_callback(self, msg):
        self.latest_image = msg

    def joint_callback(self, msg):
        self.latest_joints = msg

    def start_recording_callback(self, request, response):
        if self.is_recording:
            response.success = False
            response.message = 'Already recording'
            return response

        self.is_recording = True
        self.task_name = request.task_name
        self.operator_name = request.operator_name
        self.current_episode = []
        self.current_timestep = 0
        self.episode_count += 1

        response.success = True
        response.message = f'Started recording episode {self.episode_count}'
        self.get_logger().info(response.message)
        return response

    def stop_recording_callback(self, request, response):
        if not self.is_recording:
            response.success = False
            response.message = 'Not currently recording'
            response.file_path = ''
            return response

        self.is_recording = False
        file_path = self.save_episode(request.success_label)

        response.success = True
        response.message = f'Saved episode with {len(self.current_episode)} timesteps'
        response.file_path = file_path
        self.get_logger().info(response.message)
        return response

    def record_timestep(self):
        if not self.is_recording:
            return

        if self.latest_image is None or self.latest_joints is None:
            return

        timestep = {
            'timestamp': self.get_clock().now().nanoseconds,
            'observation': {
                'joint_positions': list(self.latest_joints.position),
                'joint_velocities': list(self.latest_joints.velocity),
                'joint_names': list(self.latest_joints.name),
            },
            'action': {
                'joint_positions': list(self.latest_joints.position),
            }
        }

        # Save image as numpy array
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, 'bgr8')
            timestep['observation']['image_shape'] = list(cv_image.shape)
        except Exception as e:
            self.get_logger().warn(f'Failed to convert image: {e}')

        self.current_episode.append(timestep)
        self.current_timestep += 1

    def save_episode(self, success_label):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'episode_{self.episode_count:04d}_{timestamp}.json'
        filepath = os.path.join(self.save_dir, filename)

        episode_data = {
            'metadata': {
                'task_name': self.task_name,
                'operator': self.operator_name,
                'timestamp': timestamp,
                'success': success_label,
                'num_timesteps': len(self.current_episode),
                'episode_id': self.episode_count,
            },
            'timesteps': self.current_episode
        }

        with open(filepath, 'w') as f:
            json.dump(episode_data, f, indent=2)

        self.get_logger().info(f'Saved episode to {filepath}')
        return filepath

    def publish_status(self):
        msg = RecordingStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.is_recording = self.is_recording
        msg.task_name = self.task_name
        msg.operator_name = self.operator_name
        msg.episode_count = self.episode_count
        msg.current_timestep = self.current_timestep
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DemoRecorderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
