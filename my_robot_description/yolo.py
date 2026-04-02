#!/usr/bin/env python3
"""
yolo_detector.py
================
ROS2 Humble node for diff_rover — runs YOLOv8 object detection on the
live camera stream and publishes annotated images + detection metadata.

Publishes:
  /detection/image_raw   [sensor_msgs/msg/Image]   — annotated BGR frame
  /detection/objects     [std_msgs/msg/String]       — JSON list of detections

Subscribes:
  /camera/image_raw      [sensor_msgs/msg/Image]

Requirements (install once):
  pip3 install ultralytics opencv-python

The first run downloads yolov8n.pt (~6 MB) automatically from Ultralytics.
You can override the model path via ROS2 param:
  ros2 run my_robot_description yolo_detector --ros-args -p model:=yolov8s.pt

Usage:
  python3 yolo_detector.py

  # With custom model:
  ros2 run my_robot_description yolo_detector --ros-args \
      -p model:=yolov8n.pt \
      -p conf_threshold:=0.4 \
      -p device:=cpu
"""

import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from std_msgs.msg import String

# ── YOLO import (graceful error if not installed) ─────────────────────────────
try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except ImportError:
    ULTRALYTICS_AVAILABLE = False


# ─────────────────────────────────────────────────────────────────────────────
# Colour palette for bounding boxes (one colour per class index, cycling)
# ─────────────────────────────────────────────────────────────────────────────

_PALETTE = [
    (56,  56,  255), (151, 157, 255), (31,  112, 255), (29,  178, 255),
    (49,  210, 207), (10,  249, 72),  (23,  204, 146), (134, 219, 61),
    (52,  147, 26),  (187, 212, 0),   (168, 153, 44),  (255, 194, 0),
    (147, 69,  52),  (255, 115, 100), (236, 24,  0),   (255, 56,  132),
    (133, 0,   82),  (255, 56,  203), (200, 149, 255), (199, 55,  255),
]


def _class_color(class_id: int):
    return _PALETTE[class_id % len(_PALETTE)]


# ─────────────────────────────────────────────────────────────────────────────
# Main node
# ─────────────────────────────────────────────────────────────────────────────

class YoloDetector(Node):

    def __init__(self):
        super().__init__('yolo_detector')

        if not ULTRALYTICS_AVAILABLE:
            self.get_logger().fatal(
                'ultralytics package not found.\n'
                'Install it with:  pip3 install ultralytics'
            )
            raise RuntimeError('ultralytics not installed')

        # ── ROS2 parameters ───────────────────────────────────────────────
        self.declare_parameter('model',          'yolov8n.pt')
        self.declare_parameter('conf_threshold', 0.35)
        self.declare_parameter('iou_threshold',  0.45)
        self.declare_parameter('device',         'cpu')      # 'cpu' or '0' for GPU
        self.declare_parameter('imgsz',          640)        # YOLO inference size
        self.declare_parameter('show_fps',       True)
        self.declare_parameter('max_det',        50)

        model_path  = self.get_parameter('model').value
        self._conf  = self.get_parameter('conf_threshold').value
        self._iou   = self.get_parameter('iou_threshold').value
        self._dev   = self.get_parameter('device').value
        self._imgsz = self.get_parameter('imgsz').value
        self._show_fps = self.get_parameter('show_fps').value
        self._max_det  = self.get_parameter('max_det').value

        # ── load model ────────────────────────────────────────────────────
        self.get_logger().info(f'Loading YOLO model: {model_path}  device={self._dev}')
        self._model = YOLO(model_path)
        # Warm up once so the first real frame isn't slow
        self._model.predict(
            np.zeros((self._imgsz, self._imgsz, 3), dtype=np.uint8),
            device=self._dev, verbose=False
        )
        self.get_logger().info('YOLO model ready ✓')

        # ── cv_bridge ────────────────────────────────────────────────────
        self._bridge = CvBridge()

        # ── FPS tracking ─────────────────────────────────────────────────
        self._fps_lock     = threading.Lock()
        self._fps_times    = []          # ring buffer of recent frame times
        self._fps_window   = 30         # frames to average over
        self._current_fps  = 0.0

        # ── QoS ──────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        pub_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # ── subscriber ───────────────────────────────────────────────────
        self.create_subscription(
            Image, '/camera/image_raw',
            self._image_cb, sensor_qos
        )

        # ── publishers ───────────────────────────────────────────────────
        self._img_pub = self.create_publisher(
            Image, '/detection/image_raw', pub_qos
        )
        self._json_pub = self.create_publisher(
            String, '/detection/objects', pub_qos
        )

        self.get_logger().info(
            '\n'
            '  YoloDetector running\n'
            f'  Model : {model_path}\n'
            f'  Conf  : {self._conf}   IOU: {self._iou}\n'
            f'  Device: {self._dev}    imgsz: {self._imgsz}\n'
            '  Pubs  : /detection/image_raw  /detection/objects\n'
        )

    # ─────────────────────────────────────────────────────────────────────────
    # Image callback — runs inference on every incoming frame
    # ─────────────────────────────────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        t0 = time.monotonic()

        # Convert ROS → OpenCV BGR
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge error: {exc}')
            return

        # ── YOLO inference ────────────────────────────────────────────────
        results = self._model.predict(
            frame,
            conf=self._conf,
            iou=self._iou,
            device=self._dev,
            imgsz=self._imgsz,
            max_det=self._max_det,
            verbose=False,
        )

        # ── parse detections ──────────────────────────────────────────────
        detections = []
        annotated  = frame.copy()

        if results and results[0].boxes is not None:
            boxes  = results[0].boxes
            names  = results[0].names        # class_id → class_name dict

            for box in boxes:
                cls_id  = int(box.cls[0].item())
                conf    = float(box.conf[0].item())
                xyxy    = box.xyxy[0].cpu().numpy().astype(int)
                x1, y1, x2, y2 = xyxy

                cls_name = names.get(cls_id, str(cls_id))
                colour   = _class_color(cls_id)

                # Draw bounding box
                cv2.rectangle(annotated, (x1, y1), (x2, y2), colour, 2)

                # Label background
                label      = f'{cls_name}  {conf:.2f}'
                (lw, lh), _ = cv2.getTextSize(
                    label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1
                )
                top_label = max(y1, lh + 6)
                cv2.rectangle(
                    annotated,
                    (x1, top_label - lh - 6),
                    (x1 + lw + 4, top_label),
                    colour, -1
                )
                cv2.putText(
                    annotated, label,
                    (x1 + 2, top_label - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1,
                    cv2.LINE_AA
                )

                # Build detection record
                detections.append({
                    'class_id':   cls_id,
                    'class_name': cls_name,
                    'confidence': round(conf, 4),
                    'bbox': {
                        'x1': int(x1), 'y1': int(y1),
                        'x2': int(x2), 'y2': int(y2),
                        'width':  int(x2 - x1),
                        'height': int(y2 - y1),
                        'cx':     int((x1 + x2) // 2),
                        'cy':     int((y1 + y2) // 2),
                    }
                })

        # ── FPS overlay ───────────────────────────────────────────────────
        elapsed = time.monotonic() - t0
        fps     = self._update_fps(elapsed)
        if self._show_fps:
            cv2.putText(
                annotated,
                f'FPS: {fps:.1f}   det: {len(detections)}',
                (10, 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA
            )

        # ── publish annotated image ───────────────────────────────────────
        try:
            out_msg = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out_msg.header = msg.header    # preserve timestamp + frame_id
            self._img_pub.publish(out_msg)
        except Exception as exc:
            self.get_logger().warn(f'publish image error: {exc}')

        # ── publish JSON detections ───────────────────────────────────────
        payload = {
            'stamp_sec':  msg.header.stamp.sec,
            'stamp_nsec': msg.header.stamp.nanosec,
            'frame_id':   msg.header.frame_id,
            'fps':        round(fps, 2),
            'count':      len(detections),
            'detections': detections,
        }
        self._json_pub.publish(String(data=json.dumps(payload)))

        # ── log summary (throttled to 1 Hz) ──────────────────────────────
        if len(detections) > 0:
            summary = ', '.join(
                f"{d['class_name']}({d['confidence']:.2f})"
                for d in detections
            )
            self.get_logger().info(
                f'[{fps:.1f} fps]  {len(detections)} object(s): {summary}',
                throttle_duration_sec=1.0
            )

    # ─────────────────────────────────────────────────────────────────────────
    # FPS helper
    # ─────────────────────────────────────────────────────────────────────────

    def _update_fps(self, elapsed: float) -> float:
        with self._fps_lock:
            self._fps_times.append(elapsed)
            if len(self._fps_times) > self._fps_window:
                self._fps_times.pop(0)
            avg = sum(self._fps_times) / len(self._fps_times)
            self._current_fps = 1.0 / avg if avg > 0 else 0.0
            return self._current_fps


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    try:
        node = YoloDetector()
    except RuntimeError:
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()