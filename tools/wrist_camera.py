#!/usr/bin/env python3
"""Publish the DaBai DC1 UVC color feed for the PiPER X workstation."""

import os
from pathlib import Path
import signal
import sys

import cv2
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image


def camera_device():
    override = os.environ.get('AGX_CAMERA_DEVICE')
    if override:
        return override
    devices = sorted(Path('/dev/v4l/by-id').glob('*_Dabai_DC1_*-video-index0'))
    if not devices:
        raise RuntimeError('No DaBai DC1 color device found. Check USB or set AGX_CAMERA_DEVICE.')
    if len(devices) > 1:
        raise RuntimeError('Multiple DaBai DC1 cameras found. Set AGX_CAMERA_DEVICE to one device.')
    return str(devices[0])


def publish_frames(capture, device):
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('wrist_camera')
    try:
        publisher = node.create_publisher(Image, 'wrist_camera/image_raw', qos_profile_sensor_data)
        node.get_logger().info(f'Publishing color frames from {device} on {publisher.topic_name}')
        while rclpy.ok():
            ok, frame = capture.read()
            if not rclpy.ok():
                break
            if not ok:
                raise RuntimeError(f'Could not read a frame from {device}. Check USB or device use.')
            image = Image()
            image.header.stamp = node.get_clock().now().to_msg()
            image.header.frame_id = 'wrist_camera_color_optical_frame'
            image.height, image.width = frame.shape[:2]
            image.encoding = 'bgr8'
            image.step = image.width * 3
            image.data = frame.tobytes()
            publisher.publish(image)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    try:
        device = camera_device()
        capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        try:
            if not capture.isOpened():
                raise RuntimeError(f'Could not open {device}. Check device permissions and use.')
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            capture.set(cv2.CAP_PROP_FPS, 30)
            publish_frames(capture, device)
        finally:
            capture.release()
    except KeyboardInterrupt:
        return 0
    except (RuntimeError, cv2.error) as exc:
        print(f'Wrist camera: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    sys.exit(main())
