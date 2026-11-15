import rclpy
from rclpy.node import Node
from robotis_controller_msgs.msg import WriteControlTable
import struct
import time

RGB_COLORS = {
            # =========================
            # BASIC
            # =========================
            'OFF':       (0, 0, 0),
            'WHITE':     (31, 31, 31), 
            'RED':       (31, 0, 0),
            'GREEN':     (0, 31, 0),
            'BLUE':      (0, 0, 31),

            # =========================
            # SECONDARY
            # =========================
            'YELLOW':    (31, 31, 0),
            'CYAN':      (0, 31, 31),
            'MAGENTA':   (31, 0, 31),

            # =========================
            # ORANGE / WARM
            # =========================
            'ORANGE':    (31, 16, 0),
            'DARK_ORANGE': (31, 8, 0),
            'LIGHT_ORANGE': (31, 24, 8),

            # =========================
            # RED
            # =========================
            'DARK_RED':  (15, 0, 0),
            'LIGHT_RED': (31, 8, 8),
            'PINK':      (31, 8, 16),
            'HOT_PINK':  (31, 0, 16),

            # =========================
            # GREEN
            # =========================
            'DARK_GREEN':  (0, 15, 0),
            'LIGHT_GREEN': (8, 31, 8),
            'LIME':        (16, 31, 0),
            'MINT':        (8, 31, 16),

            # =========================
            # BLUE
            # =========================
            'DARK_BLUE':   (0, 0, 15),
            'LIGHT_BLUE':  (8, 16, 31),
            'SKY_BLUE':    (8, 24, 31),
            'NAVY':        (0, 0, 8),

            # =========================
            # PURPLE
            # =========================
            'PURPLE':       (16, 0, 31),
            'DARK_PURPLE':  (8, 0, 16),
            'LIGHT_PURPLE': (24, 8, 31),
            'VIOLET':       (24, 0, 31),

            # =========================
            # YELLOW
            # =========================
            'DARK_YELLOW':  (20, 20, 0),
            'LIGHT_YELLOW': (31, 31, 16),
            'GOLD':         (31, 24, 0),

            # =========================
            # WHITE / GRAY
            # =========================
            'DARK_GRAY':    (8, 8, 8),
            'GRAY':         (16, 16, 16),
            'LIGHT_GRAY':   (24, 24, 24),
            'WARM_WHITE':   (31, 24, 16),
            'COOL_WHITE':   (24, 28, 31),
        }

class OpenCR_Led(Node):

    def __init__(self):
        super().__init__('opencr_led')

        self.pub = self.create_publisher(
            WriteControlTable,
            '/robotis/write_control_table',
            10
        )

        time.sleep(0.5)  # tunggu connection

    # ==========================================
    # Address 25 - LED BIASA
    # ==========================================

    def set_status_led(self, led1=None, led2=None, led3=None):
        """
        None  = tidak mengubah kondisi LED
        True  = ON
        False = OFF
        """

        if not hasattr(self, 'led1'):
            self.led1 = False
            self.led2 = False
            self.led3 = False

        if led1 is not None:
            self.led1 = led1

        if led2 is not None:
            self.led2 = led2

        if led3 is not None:
            self.led3 = led3

        value = (
            (int(self.led1) << 0) |
            (int(self.led2) << 1) |
            (int(self.led3) << 2)
        )

        msg = WriteControlTable()

        msg.joint_name = 'open-cr'
        msg.start_item_name = 'LED'
        msg.data_length = 1
        msg.data = [value & 0xFF]

        self.pub.publish(msg)

    # ==========================================
    # Address 26 - RGB LED
    # ==========================================
    def set_rgb_led(self, r=None, g=None, b=None):
        """
        r, g, b = 0-31
        None = pertahankan nilai sebelumnya
        """

        # Inisialisasi nilai awal
        if not hasattr(self, 'r'):
            self.r = 0
            self.g = 0
            self.b = 0

        # Update hanya channel yang diberikan
        if r is not None:
            self.r = r & 0x1F

        if g is not None:
            self.g = g & 0x1F

        if b is not None:
            self.b = b & 0x1F

        # Format RGB 5-bit
        rgb_value = (
            (self.r << 10) |
            (self.g << 5) |
            self.b
        )

        msg = WriteControlTable()

        msg.joint_name = 'open-cr'
        msg.start_item_name = 'LED_RGB'
        msg.data_length = 2

        # Little endian
        msg.data = list(
            struct.pack('<H', rgb_value)
        )

        self.pub.publish(msg)
        
    def set_rgb_color(self, color):
        """
        Mengatur RGB berdasarkan nama warna.
        """

        if color not in RGB_COLORS:
            self.get_logger().warn(
                f"RGB color '{color}' tidak ditemukan"
            )
            return

        r, g, b = RGB_COLORS[color]

        self.set_rgb_led(r, g, b)

    # ==========================================
    # MATIKAN SEMUA LED
    # ==========================================

    def led_off(self):

        self.set_status_led(
            False,
            False,
            False
        )

        self.set_rgb_led(
            0,
            0,
            0
        )


def main():
    rclpy.init()

    node = OpenCR_Led()

    try:
        while rclpy.ok():

            # LED 1 ON
            # node.set_status_led(True, False, False)
            # time.sleep(1)

            # # LED 2 ON
            # node.set_status_led(False, True, False)
            # time.sleep(1)

            # # LED 3 ON
            # node.set_status_led(False, False, True)
            # time.sleep(1)

            # # Semua LED ON
            # node.set_status_led(True, True, True)
            # time.sleep(1)

            # # Semua LED OFF
            # node.set_status_led(False, False, False)
            # time.sleep(1)
            node.set_rgb_color('WHITE')
            time.sleep(1)

            node.set_rgb_color('ORANGE')
            time.sleep(1)

    except KeyboardInterrupt:
        print("Program dihentikan.")

    finally:
        node.set_status_led(False, False, False)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()