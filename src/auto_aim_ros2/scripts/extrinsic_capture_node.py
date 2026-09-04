#!/usr/bin/env python3

from collections import deque
import json
from pathlib import Path
from threading import Lock, Thread

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from auto_aim_interfaces.msg import Vision


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class ExtrinsicCaptureNode(Node):
    def __init__(self) -> None:
        super().__init__('extrinsic_capture_node')

        image_topic = self.declare_parameter('image_topic', '/image_raw').value
        camera_info_topic = self.declare_parameter(
            'camera_info_topic', '/camera_info').value
        vision_topic = self.declare_parameter('vision_topic', '/Vision_data').value
        output_directory = self.declare_parameter('output_directory', '').value
        self._board_cols = int(self.declare_parameter('board_cols', 11).value)
        self._board_rows = int(self.declare_parameter('board_rows', 8).value)
        self._square_size_m = float(
            self.declare_parameter('square_size_m', 0.020).value)
        self._max_pose_age_ns = int(float(
            self.declare_parameter('max_pose_age_ms', 100.0).value) * 1_000_000)
        self._settle_time_ns = int(float(
            self.declare_parameter('settle_time_ms', 300.0).value) * 1_000_000)
        self._max_settle_angle_deg = float(
            self.declare_parameter('max_settle_angle_deg', 0.15).value)
        self._display_scale = float(
            self.declare_parameter('display_scale', 0.5).value)

        if not output_directory:
            raise RuntimeError('output_directory must not be empty')
        if self._board_cols < 2 or self._board_rows < 2:
            raise RuntimeError('board_cols and board_rows must both be at least 2')
        if self._square_size_m <= 0.0:
            raise RuntimeError('square_size_m must be positive')
        if self._max_pose_age_ns <= 0:
            raise RuntimeError('max_pose_age_ms must be positive')
        if self._settle_time_ns <= 0:
            raise RuntimeError('settle_time_ms must be positive')
        if self._max_settle_angle_deg <= 0.0:
            raise RuntimeError('max_settle_angle_deg must be positive')
        if not 0.1 <= self._display_scale <= 1.0:
            raise RuntimeError('display_scale must be between 0.1 and 1.0')

        self._output_directory = Path(output_directory).expanduser().resolve()
        self._output_directory.mkdir(parents=True, exist_ok=True)
        self._next_index = self._find_next_index()

        self._bridge = CvBridge()
        self._data_lock = Lock()
        self._latest_image = None
        self._latest_camera_info = None
        self._vision_history = deque(maxlen=2000)
        self._last_processed_image_stamp_ns = -1
        self._should_stop = False
        self._window_name = 'Extrinsic capture: s=save, q=quit'

        self._image_subscription = self.create_subscription(
            Image, image_topic, self._image_callback, qos_profile_sensor_data)
        self._camera_info_subscription = self.create_subscription(
            CameraInfo, camera_info_topic, self._camera_info_callback,
            qos_profile_sensor_data)
        self._vision_subscription = self.create_subscription(
            Vision, vision_topic, self._vision_callback, 50)

        waiting = np.zeros((540, 720, 3), dtype=np.uint8)
        cv2.putText(
            waiting, 'Waiting for camera image...', (55, 250),
            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 200, 255), 2, cv2.LINE_AA)
        cv2.putText(
            waiting, 'q: quit', (55, 300), cv2.FONT_HERSHEY_SIMPLEX,
            0.75, (220, 220, 220), 2, cv2.LINE_AA)
        cv2.namedWindow(self._window_name, cv2.WINDOW_AUTOSIZE)
        cv2.imshow(self._window_name, waiting)
        cv2.moveWindow(self._window_name, 40, 60)
        cv2.waitKey(1)

        self.get_logger().info(
            f'Extrinsic capture ready: board={self._board_cols}x{self._board_rows} '
            f'inner corners, square={self._square_size_m:.3f} m')
        self.get_logger().info(f'Output directory: {self._output_directory}')
        self.get_logger().info('Press s to save a valid pair, q to quit.')

    @property
    def should_stop(self) -> bool:
        return self._should_stop

    def _find_next_index(self) -> int:
        indices = []
        for path in self._output_directory.glob('*.jpg'):
            try:
                indices.append(int(path.stem))
            except ValueError:
                continue
        return max(indices, default=0) + 1

    def _image_callback(self, message: Image) -> None:
        with self._data_lock:
            self._latest_image = message

    def _camera_info_callback(self, message: CameraInfo) -> None:
        with self._data_lock:
            self._latest_camera_info = message

    def _vision_callback(self, message: Vision) -> None:
        q = np.asarray(message.quaternion, dtype=np.float64)
        norm = float(np.linalg.norm(q))
        if (
                not np.all(np.isfinite(q)) or norm < 1e-6 or
                abs(norm - 1.0) > 0.1):
            return
        q /= norm
        with self._data_lock:
            self._vision_history.append(
                (stamp_to_ns(message.header.stamp), q, message))

    @staticmethod
    def _nearest_vision(image_stamp_ns: int, vision_history):
        if not vision_history:
            return None
        return min(vision_history, key=lambda item: abs(item[0] - image_stamp_ns))

    def _imu_stability(self, image_stamp_ns: int, vision_history):
        window_start_ns = image_stamp_ns - self._settle_time_ns
        window = [
            item for item in vision_history
            if window_start_ns <= item[0] <= image_stamp_ns]
        if len(window) < 2:
            return None, False
        covered_ns = window[-1][0] - window[0][0]
        if covered_ns < int(self._settle_time_ns * 0.8):
            return None, False
        reference = window[-1][1]
        max_angle_deg = 0.0
        for _, q, _ in window:
            cosine = float(np.clip(abs(np.dot(reference, q)), 0.0, 1.0))
            angle_deg = float(np.degrees(2.0 * np.arccos(cosine)))
            max_angle_deg = max(max_angle_deg, angle_deg)
        return max_angle_deg, max_angle_deg <= self._max_settle_angle_deg

    def process_latest_image(self) -> None:
        with self._data_lock:
            message = self._latest_image
            camera_info = self._latest_camera_info
            vision_history = list(self._vision_history)
        if message is None:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self._should_stop = True
            return

        image_stamp_ns = stamp_to_ns(message.header.stamp)
        if image_stamp_ns == self._last_processed_image_stamp_ns:
            cv2.waitKey(1)
            return
        self._last_processed_image_stamp_ns = image_stamp_ns

        try:
            image = self._bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except Exception as error:
            self.get_logger().error(f'Could not convert camera image: {error}')
            return

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        pattern_size = (self._board_cols, self._board_rows)
        detection_scale = min(1.0, 800.0 / max(gray.shape))
        if detection_scale < 1.0:
            detection_image = cv2.resize(
                gray, None, fx=detection_scale, fy=detection_scale,
                interpolation=cv2.INTER_AREA)
        else:
            detection_image = gray
        flags = (
            cv2.CALIB_CB_ADAPTIVE_THRESH |
            cv2.CALIB_CB_NORMALIZE_IMAGE |
            cv2.CALIB_CB_FAST_CHECK)
        board_found, corners = cv2.findChessboardCorners(
            detection_image, pattern_size, flags)
        if board_found:
            if detection_scale < 1.0:
                corners /= detection_scale
            criteria = (
                cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-3)
            corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria)

        nearest = self._nearest_vision(image_stamp_ns, vision_history)
        pose_age_ns = None if nearest is None else abs(nearest[0] - image_stamp_ns)
        pose_ready = (
            nearest is not None and pose_age_ns is not None and
            pose_age_ns <= self._max_pose_age_ns)
        settle_angle_deg, imu_stable = self._imu_stability(
            image_stamp_ns, vision_history)
        camera_ready = self._valid_camera_info(camera_info, image)

        drawing = image.copy()
        if board_found:
            cv2.drawChessboardCorners(drawing, pattern_size, corners, True)

        self._draw_status(
            drawing, board_found, camera_ready, nearest, pose_age_ns, pose_ready,
            settle_angle_deg, imu_stable)
        preview = cv2.resize(
            drawing, None, fx=self._display_scale, fy=self._display_scale,
            interpolation=cv2.INTER_AREA)
        cv2.imshow(self._window_name, preview)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            self._should_stop = True
        elif key == ord('s'):
            self._save_sample(
                image, image_stamp_ns, board_found, camera_ready,
                nearest, pose_age_ns, pose_ready, settle_angle_deg, imu_stable)

    @staticmethod
    def _valid_camera_info(camera_info, image) -> bool:
        if camera_info is None:
            return False
        return (
            camera_info.width == image.shape[1] and
            camera_info.height == image.shape[0] and
            len(camera_info.k) == 9 and camera_info.k[0] > 0.0 and
            camera_info.k[4] > 0.0 and len(camera_info.d) >= 4)

    def _draw_status(
            self, drawing, board_found, camera_ready, nearest,
            pose_age_ns, pose_ready, settle_angle_deg, imu_stable) -> None:
        lines = [
            (f'board: {"OK" if board_found else "NOT FOUND"}', board_found),
            (f'camera_info: {"OK" if camera_ready else "NOT READY"}', camera_ready),
        ]
        if nearest is None or pose_age_ns is None:
            lines.append(('IMU: NOT READY', False))
        else:
            age_ms = pose_age_ns / 1_000_000.0
            lines.append((f'IMU pair age: {age_ms:.1f} ms', pose_ready))
            vision = nearest[2]
            lines.append((
                f'yaw={vision.yaw:.2f} pitch={vision.pitch:.2f} '
                f'roll={vision.roll:.2f}', pose_ready))
        if settle_angle_deg is None:
            lines.append(('IMU stillness: WAIT', False))
        else:
            lines.append((
                f'IMU stillness: {settle_angle_deg:.3f} deg / '
                f'{self._settle_time_ns / 1_000_000:.0f} ms', imu_stable))
        lines.extend([
            (f'samples: {self._next_index - 1}', True),
            ('hold still, then press s; q quits', True),
        ])

        for row, (text, ok) in enumerate(lines):
            color = (0, 255, 0) if ok else (0, 0, 255)
            cv2.putText(
                drawing, text, (30, 45 + row * 38), cv2.FONT_HERSHEY_SIMPLEX,
                0.85, color, 2, cv2.LINE_AA)

    def _save_sample(
            self, image, image_stamp_ns, board_found, camera_ready,
            nearest, pose_age_ns, pose_ready, settle_angle_deg,
            imu_stable) -> None:
        if not board_found:
            self.get_logger().warning('Not saved: chessboard was not detected.')
            return
        if not camera_ready:
            self.get_logger().warning(
                'Not saved: camera_info is missing or has the wrong resolution.')
            return
        if not pose_ready or nearest is None or pose_age_ns is None:
            self.get_logger().warning(
                'Not saved: no sufficiently close IMU quaternion was received.')
            return
        if not imu_stable or settle_angle_deg is None:
            self.get_logger().warning(
                'Not saved: hold the gimbal still until IMU stillness is green.')
            return

        index = self._next_index
        image_path = self._output_directory / f'{index}.jpg'
        quaternion_path = self._output_directory / f'{index}.txt'
        if not cv2.imwrite(str(image_path), image, [cv2.IMWRITE_JPEG_QUALITY, 100]):
            self.get_logger().error(f'Could not save image: {image_path}')
            return

        q = nearest[1]
        quaternion_path.write_text(
            f'{q[0]:.17g} {q[1]:.17g} {q[2]:.17g} {q[3]:.17g}\n',
            encoding='utf-8')
        self._write_metadata()

        vision = nearest[2]
        record = {
            'index': index,
            'image_stamp_ns': image_stamp_ns,
            'vision_stamp_ns': nearest[0],
            'pair_age_ms': pose_age_ns / 1_000_000.0,
            'settle_angle_deg': settle_angle_deg,
            'quaternion_wxyz': q.tolist(),
            'yaw': float(vision.yaw),
            'pitch': float(vision.pitch),
            'roll': float(vision.roll),
        }
        with (self._output_directory / 'samples.jsonl').open(
                'a', encoding='utf-8') as manifest:
            manifest.write(json.dumps(record, ensure_ascii=False) + '\n')

        self._next_index += 1
        self.get_logger().info(
            f'[{index}] saved; image/IMU delta={record["pair_age_ms"]:.1f} ms')

    def _write_metadata(self) -> None:
        info = self._latest_camera_info
        metadata = {
            'board_type': 'chessboard',
            'pattern_cols': self._board_cols,
            'pattern_rows': self._board_rows,
            'pattern_distance_mm': self._square_size_m * 1000.0,
            'image_width': int(info.width),
            'image_height': int(info.height),
            'camera_matrix': [float(value) for value in info.k],
            'distort_coeffs': [float(value) for value in info.d],
            'quaternion_order': 'wxyz',
        }
        (self._output_directory / 'metadata.json').write_text(
            json.dumps(metadata, indent=2) + '\n', encoding='utf-8')

    def close_windows(self) -> None:
        cv2.destroyAllWindows()


def main() -> None:
    rclpy.init()
    node = None
    executor = None
    spin_thread = None
    try:
        node = ExtrinsicCaptureNode()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        spin_thread = Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        while rclpy.ok() and not node.should_stop:
            node.process_latest_image()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=2.0)
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)
        if node is not None:
            node.close_windows()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
