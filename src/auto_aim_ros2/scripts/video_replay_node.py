#!/usr/bin/env python3

from pathlib import Path

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class VideoReplayNode(Node):
    def __init__(self) -> None:
        super().__init__('video_replay_node')
        project_root = Path(self.declare_parameter(
            'project_root', '/home/nvidia/game_26_current/sp_vision_25').value)
        video_path = Path(self.declare_parameter(
            'video_path', 'assets/demo/demo.avi').value)
        if not video_path.is_absolute():
            video_path = project_root / video_path

        self._topic = self.declare_parameter('image_topic', '/image_raw').value
        self._frame_id = self.declare_parameter('frame_id', 'camera_optical_frame').value
        self._loop = self.declare_parameter('loop', True).value
        requested_rate = float(self.declare_parameter('rate', 0.0).value)

        self._video_path = video_path
        self._capture = cv2.VideoCapture(str(self._video_path))
        if not self._capture.isOpened():
            raise RuntimeError(f'could not open video: {video_path}')

        source_fps = self._capture.get(cv2.CAP_PROP_FPS)
        self._rate = requested_rate if requested_rate > 0.0 else source_fps
        if self._rate <= 0.0:
            self._rate = 30.0

        self._bridge = CvBridge()
        self._publisher = self.create_publisher(Image, self._topic, qos_profile_sensor_data)
        self._timer = self.create_timer(1.0 / self._rate, self._publish_frame)
        self.get_logger().info(
            f'Publishing {video_path} on {self._topic} at {self._rate:.1f} FPS')

    def _publish_frame(self) -> None:
        ok, frame = self._capture.read()
        if not ok:
            if not self._loop:
                self.get_logger().info('Video replay completed')
                self._timer.cancel()
                return
            # Seeking back to frame zero is unreliable for some MJPEG AVI
            # files.  Reopening the file also clears the decoder's EOF state.
            self._capture.release()
            self._capture = cv2.VideoCapture(str(self._video_path))
            ok, frame = self._capture.read()
            if not ok:
                self.get_logger().error('Could not restart video replay')
                self._timer.cancel()
                return

        message = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        self._publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = VideoReplayNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
