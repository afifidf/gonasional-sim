#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import time
import os


class CameraDisplay(Node):

    def __init__(self):
        super().__init__('camera_display')

        self.bridge = CvBridge()

        # ==========================================================
        # SUBSCRIBE CAMERA
        # ==========================================================

        self.subscription = self.create_subscription(
            Image,
            '/usb_cam_node/image_raw',
            self.image_callback,
            10
        )

        # ==========================================================
        # FPS
        # ==========================================================

        self.last_time = time.time()
        self.frame_count = 0
        self.fps = 0.0

        # ==========================================================
        # RECORDING
        # ==========================================================

        self.recording = False
        self.video_writer = None
        self.record_start_time = None

        # Folder penyimpanan
        self.record_dir = os.path.expanduser('~/camera_recordings')

        os.makedirs(
            self.record_dir,
            exist_ok=True
        )

        self.get_logger().info(
            'Camera Display started'
        )

        self.get_logger().info(
            'Press R = start/stop recording'
        )

        self.get_logger().info(
            'Press Q or ESC = exit'
        )

    # ==============================================================
    # START RECORD
    # ==============================================================

    def start_recording(self, frame):

        height, width = frame.shape[:2]

        # Nama file berdasarkan waktu
        timestamp = time.strftime(
            '%Y%m%d_%H%M%S'
        )

        filename = os.path.join(
            self.record_dir,
            f'camera_{timestamp}.avi'
        )

        # ==========================================================
        # VIDEO CODEC
        # ==========================================================

        fourcc = cv2.VideoWriter_fourcc(
            *'MJPG'
        )

        # FPS video
        record_fps = 30.0

        self.video_writer = cv2.VideoWriter(
            filename,
            fourcc,
            record_fps,
            (width, height)
        )

        if not self.video_writer.isOpened():

            self.get_logger().error(
                'Gagal membuka VideoWriter!'
            )

            self.video_writer = None
            return

        self.recording = True
        self.record_start_time = time.time()

        self.current_filename = filename

        self.get_logger().info(
            f'RECORDING STARTED: {filename}'
        )

    # ==============================================================
    # STOP RECORD
    # ==============================================================

    def stop_recording(self):

        if self.video_writer is not None:

            self.video_writer.release()

            self.video_writer = None

        self.recording = False

        if self.record_start_time is not None:

            duration = (
                time.time()
                - self.record_start_time
            )

            self.get_logger().info(
                f'RECORDING STOPPED '
                f'({duration:.1f} seconds)'
            )

        self.record_start_time = None

    # ==============================================================
    # IMAGE CALLBACK
    # ==============================================================

    def image_callback(self, msg):

        try:

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

        except Exception as e:

            self.get_logger().error(
                f'CvBridge error: {e}'
            )

            return

        # ==========================================================
        # RAW FRAME
        # ==========================================================

        # frame_original TIDAK diberi overlay.
        #
        # Ini yang akan disimpan ke video.
        #
        frame_original = frame.copy()

        # ==========================================================
        # RECORD RAW IMAGE
        # ==========================================================

        if self.recording:

            if self.video_writer is not None:

                # SIMPAN RAW IMAGE
                #
                # Tidak ada FPS
                # Tidak ada tulisan
                # Tidak ada overlay
                #
                self.video_writer.write(
                    frame_original
                )

        # ==========================================================
        # FPS
        # ==========================================================

        self.frame_count += 1

        current_time = time.time()

        elapsed = (
            current_time
            - self.last_time
        )

        if elapsed >= 1.0:

            self.fps = (
                self.frame_count
                / elapsed
            )

            self.frame_count = 0

            self.last_time = current_time

        # ==========================================================
        # DISPLAY FRAME
        # ==========================================================

        # Buat copy khusus untuk display
        display_frame = frame_original.copy()

        height, width = display_frame.shape[:2]

        # FPS
        cv2.putText(
            display_frame,
            f'FPS: {self.fps:.1f}',
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

        # Resolution
        cv2.putText(
            display_frame,
            f'Resolution: {width}x{height}',
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )

        # Recording status
        if self.recording:

            cv2.putText(
                display_frame,
                '● RECORDING',
                (10, 95),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        # Topic
        cv2.putText(
            display_frame,
            '/usb_cam_node/image_raw',
            (10, height - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1
        )

        # ==========================================================
        # DISPLAY
        # ==========================================================

        cv2.imshow(
            'ROS2 Camera Display',
            display_frame
        )

        # ==========================================================
        # KEYBOARD
        # ==========================================================

        key = cv2.waitKey(1) & 0xFF

        # R = RECORD
        if key == ord('r'):

            if not self.recording:

                self.start_recording(
                    frame_original
                )

            else:

                self.stop_recording()

        # Q / ESC = EXIT
        elif key == ord('q') or key == 27:

            if self.recording:

                self.stop_recording()

            rclpy.shutdown()

    # ==============================================================
    # DESTROY
    # ==============================================================

    def destroy_node(self):

        if self.recording:

            self.stop_recording()

        cv2.destroyAllWindows()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = CameraDisplay()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()