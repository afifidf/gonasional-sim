import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

from std_msgs.msg import String, Int32, Float64,Float32
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu

from .main_head_control import HeadControl
from .sec_action import ActionPublisParam
from .ema_filter import DataFilter
from .led import OpenCR_Led



PAGE_NAIK = 80
PAGE_TURUN = 15

POSE_MODULE_DELAY = 0.5
POSE_READY_DELAY = 1.0
HEAD_READY_DELAY = 0.5
TICK_PERIOD = 0.05
FOLLOW_LOOP_PERIOD = 0.65
BALL_TIMEOUT = 0.5
TRACK_HOLD = 1.5

# Pose ideal head saat robot melakukan rotate (radian)
HEAD_ROTATE_PAN = 0.0
HEAD_ROTATE_TILT = -0.6

#KICKK
TARGET_2_KICK  = 27
KICK_KIRI = 155
KICK_KANAN= 156

TARGET_JARAK3 = 30

YAW_ERORR = 12
# YAW_ERORR = 12 #oke

THRESHOLD = 9 
ACTION_PAGE_FRONT = 142  # jatuh depan 
ACTION_PAGE_BACK  = 143  # jatuh belakang
TARGET_DISTANCE   = 65
# TARGET_DISTANCE   = 65 #oke

BALL_YAW_OFFSETS  = [0.0, 0.0, 0.0, 0.0] 
# BALL_YAW_OFFSETS  = [15.0, 13.5, 40.0, -37.0] 
# BALL_YAW_OFFSETS  = [0.0, 13.5, 42.0, -39.0] 
# BALL_YAW_OFFSETS  = [0.0, 13.5, 42.0, -39.0]  # sdh bagus
# BALL_YAW_OFFSETS  = [-50.0, -16.0, -39.0, -62.0] 
# BALL_YAW_OFFSETS  = [-44.0, -45.0, -53.0, -57.0] 


# BALL_YAW_OFFSETS  = [0.0, 0.0, 0.0, 0.0] 

class RobotActionState(Node):

    def __init__(self):
        super().__init__('robot_state')

        self.sensor_group = MutuallyExclusiveCallbackGroup()
        self.ball_group = MutuallyExclusiveCallbackGroup()
        self.imu_group = MutuallyExclusiveCallbackGroup()

        # Controller
        self.head = HeadControl(self)
        self.action = ActionPublisParam(self)
        self.led = OpenCR_Led()

        # Publishers
        self.module_pub = self.create_publisher(
            String, '/robotis/enable_ctrl_module', 10
        )
        self.action_pub = self.create_publisher(
            Int32, '/robotis/action/page_num', 10
        )
        self.head_pub = self.create_publisher(
            JointState, '/robotis/head_control/set_joint_states', 10
        )
        self.head_state_pub = self.create_publisher(
            String, '/communication/head_state', 10
        )

        # Subscribers
        self.create_subscription(
            String, '/robotis/open_cr/button',
            self.button_callback, 10
        )
        self.create_subscription(
            Float64, '/robotis/open_cr/yaw',
            self.current_yaw_callback, 10,
            callback_group=self.sensor_group
        )
        self.create_subscription(
            String, '/obj_detect',
            self.ball_callback, 10,
            callback_group=self.ball_group
        )
        self.create_subscription(
            Float32, '/vision/hasil_servo', self.jarak_callback, 10,
            callback_group=self.sensor_group
        )
        self.create_subscription(
            String, '/vision/bola_posisi', self.ball_position_callback, 10,
            callback_group=self.sensor_group
        )
        self.create_subscription(
            Imu, '/robotis/open_cr/imu', self.imu_callback, 10,
            callback_group=self.imu_group
        )
        
        self.led.set_status_led(True, True, True)
        time.sleep(2)
        self.led.set_status_led(False, False, False)

        # State machine
        self.state = 'IDLE'
        self.pose_target = None
        self.deadline = 0.0
        
        #Satet Button 
        self.button_user= True
        self.button_yaw= True 

        self.vision_posisi = ""

        # Yaw
        self.saved_yaw = 0.0
        self.current_yaw = 0.0
        self.error = 0.0

        # Ball tracking
        self.last_ball_time = None
        self.track_hold_start = None
        self._rotate_head_hold = False
        self.hold_pan = 0.0
        self.hold_tilt = 0.0
        self.jarak_bola=0.0

        # Head scan on/off (tombol mode)
        self.scan_enabled = False
        self.walking = False
        self.index = False

        self.head_pan= 0.0
        self.head_tilt= 0.0

        #filter 
        self.filter_data = DataFilter(
            window_size=5,
            alpha=0.2
        )  

        # Timer
        self.create_timer(TICK_PERIOD, self._tick)
        self.create_timer(FOLLOW_LOOP_PERIOD, self._follow_loop)

        self.get_logger().info('State Machine: IDLE')

    # ================================================
    # STATE MACHINE
    # ================================================

    def button_callback(self, msg):
        button = msg.data.strip().lower()

        if button == 'user':
            # if self.button_yaw ==True:
            #    self.saved_yaw = self.current_yaw 
            #    self.get_logger().info(f'YAW SAVE IN :{self.saved_yaw} DERAJAT ')
            #    self.button_yaw= False

            if self.button_user == True:
                self._start_pose('naik')
                self.button_user = False
                self.led.set_status_led(led3=True)
            elif self.button_user == False:
                self._start_pose('turun')
                self.button_user = True
                self.led.set_status_led(led3=False)


        elif button == 'mode':
            if self.button_yaw ==True:
                self.saved_yaw = self.current_yaw 
                self.get_logger().info(f'YAW SAVE IN :{self.saved_yaw} DERAJAT ')
            # self.set_head_scan(not self.scan_enabled)
            self.set_head_scan(True)

            self.led.set_rgb_color("RED")
            time.sleep(2.0)
            self.led.set_rgb_color("OFF")

            if self.index == False:
                self.action.ball_index = 0 
                self.get_logger().info("RESET INDEX BOLA")
                self.led.set_rgb_color("WHITE")
                # time.sleep(2.0)
                # self.led.set_rgb_color("OFF")
                self.index = True

            elif self.index == True:
                self.action.ball_index = (self.action.ball_index + 1) % 4
                self.get_logger().info(f"Index Bola = {self.action.ball_index}")
                if self.action.ball_index == 1:
                    self.led.set_rgb_color("GREEN")
                    # time.sleep(2.0)
                    # self.led.set_rgb_color("OFF")
                elif self.action.ball_index == 2:
                    self.led.set_rgb_color("DARK_PURPLE")
                    # time.sleep(2.0)
                    # self.led.set_rgb_color("OFF")
                elif self.action.ball_index == 3:
                    self.led.set_rgb_color("NAVY")
                    # time.sleep(2.0)
                    # self.led.set_rgb_color("OFF")

                if self.action.ball_index == 3:
                    self.index = False


        elif button == 'start':
            if self.walking:
                self._stop_walk()
                self.led.set_status_led(led2=False)
            else:
                self._start_walk(time.monotonic())
                self.led.set_status_led(led2=True)

    def _start_walk(self, now):
        if self.state != 'SCAN':
            self.get_logger().warn('START diabaikan: robot belum SCAN')
            return

        # if self.last_ball_time is None or now - self.last_ball_time > BALL_TIMEOUT:
        #     self.get_logger().warn('START diabaikan: bola belum terdeteksi')
        #     return

        self.walking = True
        self.head._stop_orbit()
        self.track_hold_start = None
        self.get_logger().info('START -> follow head walking')


    def _stop_walk(self):
        self.walking = False
        self.action.orbit.stop()
        self.head._stop_orbit()
        self.track_hold_start = None
        self.head._enter_scan_mode()
        self.get_logger().info('Follow head walking selesai -> SCAN')

    def set_head_scan(self, enabled):
        self.scan_enabled = bool(enabled)

        if not self.scan_enabled:
            if self.walking:
                self._stop_walk()
            self.head._stop_orbit()

        self.get_logger().info(
            f'Head scan {"ON" if self.scan_enabled else "OFF"}'
        )

    def _start_pose(self, target):
        if self.walking:
            self._stop_walk()
        self.head._stop_orbit()
        self.last_ball_time = None
        self.track_hold_start = None
        self.pose_target = target

        if target == 'naik':
            self.state = 'STAND_MODULE'
        else:
            self.state = 'LOWER_MODULE'

        self.deadline = time.monotonic() + POSE_MODULE_DELAY

        self._publish_module('action_module')

        self.get_logger().info(
            f'Pose {target} dimulai'
        )

    def _tick(self):
        now = time.monotonic()

        # Proses perpindahan state
        if now >= self.deadline:
            self._update_state(now)

        if self.state == 'SCAN':
            self._update_head(now)

    def _follow_loop(self):

        if not self.walking:
            return

        if self.action.is_running:
            return

        # =================================
        # ROTATING
        # =================================

        if self.action.is_rotating:
            self.action.handle_rotating(self.error,eror_yaw=YAW_ERORR)
            return

        if self.last_ball_time is None or time.monotonic() - self.last_ball_time > BALL_TIMEOUT:
            if self.action.orbit.running:
                self.action.orbit.stop()
            return

        # =================================
        # CEK YAW SEBELUM KICK
        # =================================
        if 0 < self.jarak_bola < TARGET_DISTANCE:

            if abs(self.error) >= YAW_ERORR:

                # Khusus bola ke-4
                if self.action.ball_index == 3 and self.jarak_bola > TARGET_JARAK3:
                    return

                self._enter_rotate_head_hold()
                self.action.stop_walking()
                time.sleep(0.5)
                self.action.init_pose(80)
                time.sleep(1.5)

                self.get_logger().info("KE POSISI WALK READY")
                self.action.start_rotating()
                return
        # =================================
        # KICK
        # =================================

        if 0.0 < self.jarak_bola <= TARGET_2_KICK:

            self.action.orbit.stop()

            self.filter_data = DataFilter(window_size=5, alpha=0.2)

            self.action.kick(
                jarak_bola=self.jarak_bola,
                target_to_kick=TARGET_2_KICK,
                kiri=KICK_KIRI,
                kanan=KICK_KANAN,
                posisi=self.vision_posisi,
                # head_pan tidak pernah di-update (joint_derajat tidak di-subscribe),
                # pakai perintah pan terakhir dari HeadControl
                pan=self.head.pan,
                error=self.error,
                error_yaw=YAW_ERORR
            )
            return

        # =================================
        # FOLLOW HEAD
        # =================================

        self.action.follow_head(
            target_pan=0.0,
            target_distance=TARGET_2_KICK,
            derajat_cam=self.head.pan,
            jarak_bola=self.jarak_bola
        )

    def _enter_rotate_head_hold(self):
        self._rotate_head_hold = True
        self.track_hold_start = None
        if self.head.orbit_running:
            self.head._stop_orbit()
        self.head.head_mode = "hold"
        for pid in self.head.pid:
            pid.Init()
        self.head.pan = HEAD_ROTATE_PAN
        self.head.tilt = HEAD_ROTATE_TILT
        self.head._publish_servo(HEAD_ROTATE_PAN, HEAD_ROTATE_TILT)

    def _update_head(self, now):
        if self.action.is_rotating:
            if not self._rotate_head_hold:
                self._enter_rotate_head_hold()
            self.head._publish_servo(HEAD_ROTATE_PAN, HEAD_ROTATE_TILT)
            return

        if self._rotate_head_hold:
            self._rotate_head_hold = False

        if self.head.orbit_running:
            return

        recent = (
            self.last_ball_time is not None
            and now - self.last_ball_time <= BALL_TIMEOUT
        )

        if recent:
            self.track_hold_start = None
            if self.head.head_mode != "track":
                self.head._enter_track_mode()
            self.head._track_head()
            return

        if self.walking:
            self.track_hold_start = None
            self.head._scan_head(is_walking=True)
            return

        if self.head.head_mode == "track":
            if self.track_hold_start is None:
                self.track_hold_start = now
                self.hold_pan = self.head.pan
                self.hold_tilt = self.head.tilt
                self.head.head_state("HOLDING")

            if now - self.track_hold_start < TRACK_HOLD:
                self.head._publish_servo(self.hold_pan, self.hold_tilt)
                return

            self.track_hold_start = None
            if not self.scan_enabled:
                self.head.head_mode = "hold"
                return
            self.head._enter_scan_mode(from_last_position=False)
            # self.head._enter_scan_mode()

        else:
            if not self.scan_enabled:
                if self.head.head_mode != "hold":
                    self.head.head_mode = "hold"
                    self.head.head_state("HOLDING")
                return

            if self.head.head_mode != "scan":
                self.head._enter_scan_mode()

        self.head._scan_head()

    def _update_state(self, now):

        if self.state == 'STAND_MODULE':
            self._publish_page(PAGE_NAIK)
            self.state = 'STAND_ACTION'
            self.deadline = now + POSE_READY_DELAY

        elif self.state == 'STAND_ACTION':
            self.head.enable_head_module()
            self.state = 'STAND_HEAD'
            self.deadline = now + HEAD_READY_DELAY

        elif self.state == 'STAND_HEAD':
            self.state = 'SCAN'
            self.pose_target = None
            self._reset_scan()
            self.head._enter_scan_mode()
            self.last_ball_time = None
            self.track_hold_start = None

            self.get_logger().info('Robot siap: SCAN')

        elif self.state == 'LOWER_MODULE':
            self._publish_page(PAGE_TURUN)
            self.state = 'LOWER_ACTION'
            self.deadline = now + POSE_READY_DELAY

        elif self.state == 'LOWER_ACTION':
            self.state = 'IDLE'
            self.pose_target = None

            self.get_logger().info('Robot selesai turun: IDLE')

    # ================================================
    # HELPERS
    # ================================================

    def _reset_scan(self):
        self.head.pan = 0.0
        self.head.tilt = -0.3
        self.head.scan_dir = 1
        self.head.tilt_dir = 1
        self.head.scan_count = 0

    def head_state(self, state):
        msg = String()
        msg.data = state
        self.head_state_pub.publish(msg)

    def _publish_servo(self, pan, tilt):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['head_pan', 'head_tilt']
        msg.position = [pan, tilt]
        self.head_pub.publish(msg)

    def _publish_page(self, page):
        msg = Int32()
        msg.data = int(page)
        self.action_pub.publish(msg)

    def _publish_module(self, module):
        msg = String()
        msg.data = module
        self.module_pub.publish(msg)

    def ball_position_callback(self, msg):

        self.vision_posisi = msg.data.lower().strip()

    def ball_callback(self, msg: String):
        if self.state != 'SCAN' or not self.head.head_enabled:
            return

        if self.action.is_rotating:
            self.last_ball_time = time.monotonic()
            return

        if not self.head.on_detection(msg):
            return

        self.last_ball_time = time.monotonic()

        if self.head.orbit_running:
            self.head._stop_orbit()

        if self.head.head_mode != "track":
            self.head._enter_track_mode()


    def jarak_callback(self, msg):
        if msg.data <= 0.0:
            self.filter_data = DataFilter(window_size=5, alpha=0.2)
            self.jarak_bola = 0.0
            return
        self.jarak_bola = self.filter_data.update(msg.data)

    def joint_derajat(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name == 'head_pan':
                self.head_pan = msg.position[i]
                # self.get_logger().info(f'head_pan = {self.derajat_kamera_pan}')
            if name == 'head_tilt':
                self.head_tilt = msg.position[i]

    def imu_callback(self,msg:Imu):
        # Robot yang sedang duduk/IDLE tidak boleh memainkan page bangun
        # hanya karena diangkat atau dimiringkan.
        if self.state != 'SCAN':
            return
        acc_x = msg.linear_acceleration.x
        self.action.fall(acc_x=acc_x,
                         treshold=THRESHOLD,
                         action_page_back=ACTION_PAGE_BACK,
                         action_page_front=ACTION_PAGE_FRONT,
                         )


    # ================================================
    # YAW
    # ================================================

    @staticmethod
    def normalize_error(error):
        return (error + 180.0) % 360.0 - 180.0

    def get_target_yaw(self):
        idx = self.action.ball_index % len(BALL_YAW_OFFSETS)
        # idx = min(self.ball_index, len(BALL_YAW_OFFSETS) - 1)
        return self.saved_yaw + BALL_YAW_OFFSETS[idx]

    def get_yaw_error(self):
        return self.normalize_error(self.get_target_yaw() - self.current_yaw)

    def current_yaw_callback(self, msg):
        self.current_yaw = msg.data
        self.error = self.get_yaw_error()

def main():
    rclpy.init()
    node = RobotActionState()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
