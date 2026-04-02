#!/usr/bin/env python3
"""
camera_recorder.py
==================
ROS2 Humble node for diff_rover — records:
  1. Continuous MP4 video  →  output/video_<timestamp>.mp4
  2. JPEG snapshot every 10 s  →  output/snapshot_<timestamp>.jpg
  3. ROS2 bag (camera + lidar + odom + tf)  →  output/rosbag_<timestamp>/

Topics subscribed:
  /camera/image_raw   [sensor_msgs/msg/Image]
  /scan               [sensor_msgs/msg/LaserScan]
  /odom               [nav_msgs/msg/Odometry]
  /tf                 [tf2_msgs/msg/TFMessage]

Usage:
  # Install deps once:
  pip3 install opencv-python

  # Build / source your workspace, then:
  ros2 run my_robot_description camera_recorder

  # OR just run directly:
  python3 camera_recorder.py
"""

import os
import time
import datetime
import subprocess
import threading

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

import rosbag2_py


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def ts_str() -> str:
    """Return a filesystem-safe timestamp string."""
    return datetime.datetime.now().strftime('%Y%m%d_%H%M%S')


def ensure_dir(path: str) -> str:
    os.makedirs(path, exist_ok=True)
    return path


# ─────────────────────────────────────────────────────────────────────────────
# Main node
# ─────────────────────────────────────────────────────────────────────────────

class CameraRecorder(Node):

    # Snapshot interval in seconds
    SNAPSHOT_INTERVAL = 10.0

    # Video codec & FPS
    FOURCC   = cv2.VideoWriter_fourcc(*'mp4v')
    FPS      = 30.0
    FRAME_WH = (640, 480)          # must match camera sensor in URDF

    def __init__(self):
        super().__init__('camera_recorder')

        # ── output directories ────────────────────────────────────────────
        base_out   = ensure_dir(os.path.join(os.getcwd(), 'output'))
        run_tag    = ts_str()
        self._snap_dir  = ensure_dir(os.path.join(base_out, 'snapshots'))
        self._video_dir = ensure_dir(os.path.join(base_out, 'video'))
        self._bag_dir   = os.path.join(base_out, f'rosbag_{run_tag}')   # rosbag2 creates it

        # ── cv_bridge ────────────────────────────────────────────────────
        self._bridge = CvBridge()

        # ── video writer (opened on first frame so we know the true size) ─
        self._video_writer: cv2.VideoWriter | None = None
        self._video_path = os.path.join(
            self._video_dir, f'video_{run_tag}.mp4'
        )

        # ── snapshot timer state ──────────────────────────────────────────
        self._last_snap_time = time.monotonic()
        self._frame_lock     = threading.Lock()
        self._latest_frame   = None          # numpy BGR uint8

        # ── rosbag2 writer ────────────────────────────────────────────────
        self._bag_writer = self._init_bag(self._bag_dir)

        # ── QoS for sensor topics ─────────────────────────────────────────
        sensor_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # ── subscribers ───────────────────────────────────────────────────
        self.create_subscription(
            Image, '/camera/image_raw',
            self._image_cb, sensor_qos
        )
        self.create_subscription(
            LaserScan, '/scan',
            self._scan_cb, sensor_qos
        )
        self.create_subscription(
            Odometry, '/odom',
            self._odom_cb, reliable_qos
        )
        self.create_subscription(
            TFMessage, '/tf',
            self._tf_cb, reliable_qos
        )

        self.get_logger().info(
            f'\n'
            f'  CameraRecorder started\n'
            f'  Video    → {self._video_path}\n'
            f'  Snapshots→ {self._snap_dir}/\n'
            f'  ROS bag  → {self._bag_dir}/\n'
            f'  Snapshot interval: {self.SNAPSHOT_INTERVAL} s\n'
        )

    # ── rosbag2 init ──────────────────────────────────────────────────────

    def _init_bag(self, bag_dir: str) -> rosbag2_py.SequentialWriter:
        writer = rosbag2_py.SequentialWriter()

        storage_options = rosbag2_py.StorageOptions(
            uri=bag_dir,
            storage_id='sqlite3',
        )
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        )
        writer.open(storage_options, converter_options)

        # Register topics
        topics = [
            ('/camera/image_raw',  'sensor_msgs/msg/Image'),
            ('/scan',              'sensor_msgs/msg/LaserScan'),
            ('/odom',              'nav_msgs/msg/Odometry'),
            ('/tf',                'tf2_msgs/msg/TFMessage'),
        ]
        for topic_name, type_name in topics:
            topic_meta = rosbag2_py.TopicMetadata(
                name=topic_name,
                type=type_name,
                serialization_format='cdr',
            )
            writer.create_topic(topic_meta)

        return writer

    # ── bag writer helper ─────────────────────────────────────────────────

    def _write_to_bag(self, topic: str, msg):
        try:
            serialized = serialize_message(msg)
            stamp      = self.get_clock().now().nanoseconds
            self._bag_writer.write(topic, serialized, stamp)
        except Exception as exc:
            self.get_logger().warn(f'Bag write error on {topic}: {exc}')

    # ── image callback ────────────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        # Convert to OpenCV BGR
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge error: {exc}')
            return

        h, w = frame.shape[:2]

        # ── video writer (lazy init) ───────────────────────────────────
        if self._video_writer is None:
            self._video_writer = cv2.VideoWriter(
                self._video_path, self.FOURCC, self.FPS, (w, h)
            )
            self.get_logger().info(
                f'VideoWriter opened: {w}x{h} @ {self.FOURCC}'
            )

        self._video_writer.write(frame)

        # ── snapshot every SNAPSHOT_INTERVAL seconds ───────────────────
        now = time.monotonic()
        with self._frame_lock:
            self._latest_frame = frame.copy()

        if now - self._last_snap_time >= self.SNAPSHOT_INTERVAL:
            self._last_snap_time = now
            self._save_snapshot(frame)

        # ── write raw image msg to bag ─────────────────────────────────
        self._write_to_bag('/camera/image_raw', msg)

    def _save_snapshot(self, frame):
        filename = os.path.join(self._snap_dir, f'snapshot_{ts_str()}.jpg')
        ok = cv2.imwrite(filename, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
        if ok:
            self.get_logger().info(f'Snapshot saved → {filename}')
        else:
            self.get_logger().error(f'Failed to write snapshot: {filename}')

    # ── lidar callback ────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        self._write_to_bag('/scan', msg)

    # ── odometry callback ─────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self._write_to_bag('/odom', msg)

    # ── tf callback ───────────────────────────────────────────────────────

    def _tf_cb(self, msg: TFMessage):
        self._write_to_bag('/tf', msg)

    # ── cleanup ───────────────────────────────────────────────────────────

    def destroy_node(self):
        self.get_logger().info('Shutting down CameraRecorder …')
        if self._video_writer is not None:
            self._video_writer.release()
            self.get_logger().info(f'Video saved → {self._video_path}')
        # rosbag2 writer closes automatically when garbage-collected
        super().destroy_node()


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CameraRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()