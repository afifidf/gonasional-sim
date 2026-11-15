import math
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray, String, Float32
from sensor_msgs.msg import JointState

from .trigonometri import (
    hitung,
    get_offset_angle,
    dfov_to_hv_fov,
)


# ============================================================
# KONFIGURASI KAMERA
# ============================================================

IMG_WIDTH = 640
IMG_HEIGHT = 480

DFOV_DEG = 78.0


# ============================================================
# KONFIGURASI DETEKSI BOLA
# ============================================================

# Jika tidak menerima bbox bola selama waktu ini,
# maka distance akan di-reset menjadi 0.
BALL_TIMEOUT = 1.0  # detik


class TrigonometriNode(Node):

    def __init__(self):

        super().__init__('trigonometri_node')

        # ====================================================
        # FOV KAMERA
        # ====================================================

        self.hfov, self.vfov = dfov_to_hv_fov(
            DFOV_DEG,
            4,
            3
        )

        # ====================================================
        # HEAD TILT
        # ====================================================

        self.tilt_rad = 0.0

        # ====================================================
        # DATA DISTANCE
        # ====================================================

        self.distance_robot_cm = 0.0

        # Waktu terakhir bola terdeteksi
        self.last_ball_detected_time = 0.0

        # Timeout bola
        self.BALL_TIMEOUT = BALL_TIMEOUT

        # Status apakah bola sedang terdeteksi
        self.ball_detected = False

        # ====================================================
        # SUBSCRIBER BBOX
        # ====================================================

        self.bbox_sub = self.create_subscription(
            Float32MultiArray,
            '/vision/bola_cx_cy',
            self.bbox_callback,
            10
        )

        # ====================================================
        # SUBSCRIBER HEAD TILT
        # ====================================================

        self.joint_sub = self.create_subscription(
            JointState,
            '/robotis/goal_joint_states',
            self.joint_derajat_callback,
            10
        )

        # ====================================================
        # PUBLISHER HASIL LENGKAP
        # ====================================================

        self.result_pub = self.create_publisher(
            String,
            '/vision/trigono_result',
            10
        )

        # ====================================================
        # PUBLISHER DISTANCE
        # ====================================================

        self.servo_pub = self.create_publisher(
            Float32,
            '/vision/hasil_servo',
            10
        )

        # ====================================================
        # TIMER CEK BOLA
        # ====================================================

        # Timer berjalan setiap 0.1 detik
        #
        # Fungsinya untuk mengecek apakah bola sudah
        # tidak terdeteksi selama BALL_TIMEOUT.
        #

        self.reset_timer = self.create_timer(
            0.1,
            self.check_ball_timeout
        )

        # ====================================================
        # LOG
        # ====================================================

        self.get_logger().info(
            'Trigonometri Node jalan'
        )

        self.get_logger().info(
            f'Camera: {IMG_WIDTH}x{IMG_HEIGHT}'
        )

        self.get_logger().info(
            f'HFOV: {self.hfov:.3f} deg | '
            f'VFOV: {self.vfov:.3f} deg'
        )

        self.get_logger().info(
            f'Ball timeout: {self.BALL_TIMEOUT:.1f} detik'
        )


    # ========================================================
    # CALLBACK HEAD JOINT
    # ========================================================

    def joint_derajat_callback(
        self,
        msg: JointState
    ):

        for i, name in enumerate(msg.name):

            if name == 'head_tilt':

                self.tilt_rad = msg.position[i]

                break


    # ========================================================
    # CALLBACK BBOX
    # ========================================================

    def bbox_callback(
        self,
        msg: Float32MultiArray
    ):

        data = msg.data

        # ----------------------------------------------------
        # Validasi data
        # ----------------------------------------------------

        if len(data) < 4:

            self.get_logger().warn(
                f'Data bbox tidak lengkap: {list(data)}'
            )

            return


        # ====================================================
        # BOLA TERDETEKSI
        # ====================================================

        # Simpan waktu terakhir menerima bbox
        self.last_ball_detected_time = time.time()

        # Status bola
        self.ball_detected = True


        # ====================================================
        # AMBIL BBOX
        # ====================================================

        x1 = data[0]
        y1 = data[1]
        x2 = data[2]
        y2 = data[3]

        bbox = (
            x1,
            y1,
            x2,
            y2
        )


        # ====================================================
        # KALIBRASI HEAD
        # ====================================================

        kalibrasi = (
            math.degrees(self.tilt_rad)
            + 90.0
        )


        # ====================================================
        # HITUNG OFFSET SUDUT
        # ====================================================

        offset_x, offset_y = get_offset_angle(
            bbox,
            IMG_WIDTH,
            IMG_HEIGHT,
            self.hfov,
            self.vfov,
        )


        # ====================================================
        # HITUNG TRIGONOMETRI
        # ====================================================

        hasil_hitung = hitung(
            kalibrasi
            - 5.0
            - offset_y
        )


        # ====================================================
        # VALIDASI HASIL HITUNG
        # ====================================================

        if not isinstance(
            hasil_hitung,
            dict
        ):

            self.get_logger().error(
                f'Format hasil hitung tidak valid: '
                f'{hasil_hitung}'
            )

            return


        # ====================================================
        # CEK KEY DISTANCE
        # ====================================================

        if 'distance_robot_cm' not in hasil_hitung:

            self.get_logger().error(
                f'Key distance_robot_cm tidak ditemukan: '
                f'{hasil_hitung}'
            )

            return


        # ====================================================
        # AMBIL DISTANCE
        # ====================================================

        distance_robot_cm = float(
            hasil_hitung[
                'distance_robot_cm'
            ]
        )


        # ====================================================
        # SIMPAN DISTANCE TERAKHIR
        # ====================================================

        self.distance_robot_cm = (
            distance_robot_cm
        )


        # ====================================================
        # HASIL AKHIR
        # ====================================================

        hasil = {

            'offset_horizontal': offset_x,

            'offset_vertical': offset_y,

            'distance_robot_cm':
                distance_robot_cm,

        }


        # ====================================================
        # LOG
        # ====================================================

        # self.get_logger().info(

        #     f'bbox={bbox} | '

        #     f'tilt={self.tilt_rad:.3f} rad | '

        #     f'kalibrasi={kalibrasi:.3f} deg | '

        #     f'offset_h={offset_x:.3f} deg | '

        #     f'offset_v={offset_y:.3f} deg | '

        #     f'distance={distance_robot_cm:.3f} cm'

        # )


        # ====================================================
        # PUBLISH HASIL LENGKAP
        #
        # /vision/trigono_result
        # ====================================================

        out = String()

        out.data = (

            f'offset_h={offset_x},'

            f'offset_v={offset_y},'

            f'distance_robot_cm={distance_robot_cm}'

        )

        self.result_pub.publish(
            out
        )


        # ====================================================
        # PUBLISH FLOAT32
        #
        # /vision/hasil_servo
        #
        # Hanya distance_robot_cm
        # ====================================================

        out_servo = Float32()

        out_servo.data = float(
            distance_robot_cm
        )

        self.servo_pub.publish(
            out_servo
        )


    # ========================================================
    # CEK TIMEOUT BOLA
    # ========================================================

    def check_ball_timeout(self):

        # ----------------------------------------------------
        # Belum pernah menerima bola
        # ----------------------------------------------------

        if self.last_ball_detected_time == 0.0:

            return


        # ----------------------------------------------------
        # Hitung waktu sejak deteksi terakhir
        # ----------------------------------------------------

        elapsed = (
            time.time()
            - self.last_ball_detected_time
        )


        # ----------------------------------------------------
        # Jika masih dalam timeout
        # ----------------------------------------------------

        if elapsed <= self.BALL_TIMEOUT:

            return


        # ====================================================
        # BOLA HILANG
        # ====================================================

        # Jangan publish reset berulang-ulang
        if not self.ball_detected:

            return


        # Ubah status bola
        self.ball_detected = False


        # ====================================================
        # RESET DISTANCE
        # ====================================================

        self.distance_robot_cm = 0.0


        # ====================================================
        # LOG
        # ====================================================

        self.get_logger().warn(
            f'Bola tidak terdeteksi selama '
            f'{elapsed:.2f} detik. '
            f'Distance di-reset ke 0.'
        )


        # ====================================================
        # PUBLISH DISTANCE = 0
        #
        # /vision/hasil_servo
        # ====================================================

        out_servo = Float32()

        out_servo.data = 0.0

        self.servo_pub.publish(
            out_servo
        )


        # ====================================================
        # PUBLISH HASIL LENGKAP = 0
        #
        # /vision/trigono_result
        # ====================================================

        out = String()

        out.data = (
            'offset_h=0.0,'
            'offset_v=0.0,'
            'distance_robot_cm=0.0'
        )

        self.result_pub.publish(
            out
        )


# ============================================================
# MAIN
# ============================================================

def main():

    rclpy.init()

    node = TrigonometriNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


# ============================================================
# RUN
# ============================================================

if __name__ == '__main__':

    main()