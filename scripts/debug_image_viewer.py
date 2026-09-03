#!/usr/bin/env python3
"""
debug_image_viewer.py — 订阅 /auto_aim/debug_image (BEST_EFFORT QoS)
并在 Orin 桌面上用 OpenCV imshow 显示预览。

用法:
  DISPLAY=:0 python3 debug_image_viewer.py
"""
import sys
import os
import signal

sys.path.insert(0, '/opt/ros/humble/lib/python3.10/dist-packages')

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class DebugImageViewer(Node):
    def __init__(self):
        super().__init__('debug_image_viewer')
        # BEST_EFFORT QoS — 匹配 auto_aim_node 的 publisher
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/auto_aim/debug_image', self.callback, qos)
        self.count = 0
        self.get_logger().info('Subscribed to /auto_aim/debug_image (BEST_EFFORT)')

    def callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge failed: {e}')
            return
        cv2.imshow('Auto Aim Debug', cv_img)
        cv2.waitKey(1)
        self.count += 1
        if self.count <= 3:
            self.get_logger().info(f'Frame #{self.count}: {msg.width}x{msg.height}')


def main():
    rclpy.init()
    node = DebugImageViewer()
    signal.signal(signal.SIGINT, lambda *_: (node.destroy_node(), rclpy.shutdown()))

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
