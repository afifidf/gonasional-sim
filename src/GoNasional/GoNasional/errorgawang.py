import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float64

class GawangSearch(Node):
    def __init__(self):
        super().__init__('GawangSearch')

        self.create_subscription(Float64, '/robotis/open_cr/yaw', self.yaw_callback, 10)
        self.create_subscription(Float64,'/robotis/open_cr/sudut_reven', self.saved_yaw_callback, 10)
        self.create_subscription(String, '/robotis/open_cr/button', self.button_callback, 10)

        self.get_logger().info('GawangSearch Ready!!!!')

        self.current_yaw = 0.0
        self.saved_yaw = 0.0

    def button_callback(self, msg: String):
        button = msg.data
        now = time.time()

        if button == 'mode':
            self.saved_yaw = self.current_yaw
            self.get_logger().info (f"Yaw disimpan: {self.saved_yaw:.2f}")

        elif button == 'start':
            self.rotate_to_gol()

    def yaw_callback(self, msg: Float64):
        self.current_yaw = msg.data
        # self.get_logger().info(f"Yaw saat ini: {self.yaw}")

    def saved_yaw_callback(self, msg):
        self.saved_yaw = msg.data

    def normalize_error (self, error):
        while error > 180:
            error -= 360
        while error < -180: 
            error += 360
        return error
        
    def rotate_to_gol(self):
        error = self.normalize_error(self.saved_yaw - self.current_yaw)
        self.get_logger().info(f"saved_yaw: {self.saved_yaw:.2f}, current_yaw: {self.current_yaw:.2f}, error: {error:.2f}")
        return

def main(args=None):
    rclpy.init(args=args)

    node = GawangSearch()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

