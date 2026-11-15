import time
import math
import threading
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import String, Int32, Float64, Float32
from geometry_msgs.msg import Twist
from op3_walking_module_msgs.msg import WalkingParam
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from .op3_orbit_walk import OP3OrbitWalker
from collections import deque
from .main_head_control import HeadControl
from .led import OpenCR_Led
from .ema_filter import DataFilter
# from .yaw_filter import YawFilter

 
KANAN             = 156  # Kick kanan 
KIRI              = 155   # kick kiri 

ACTION_DURATION   = 1.0  # dusari action

# Action module menolak page baru selama page sebelumnya masih jalan
# ("Can not play page ... (Now playing)"), jadi page berikutnya menunggu
# /robotis/movement_done. Timeout hanya pengaman kalau sinyal tidak datang.
PAGE_DONE_TIMEOUT = 4.0

Y_ROTATED         = 0.03 # param untuk y sebelum rotate 
YAW_ROTATED       = 5.3  # param untuk yaw sebelum rotate

# Batas waktu maksimum robot boleh berada di state ROTATING.
# Kalau lewat waktu ini tapi belum juga align, paksa balik ke FOLLOW_HEAD
# supaya robot tidak macet selamanya di fase rotate (misal karena yaw
# noise/drift atau target tidak pernah tercapai).
ROTATE_TIMEOUT = 10.0  # waktu ratate untuk berhenti 

class ActionPublisParam:
    def __init__(self, node):
        self.node = node

        self.orbit = OP3OrbitWalker(self.node)
        # self.yaw_filter = YawFilter()
        self.led = OpenCR_Led()
        self.filter= DataFilter()
        
        self.walking_vel_pub = self.node.create_publisher(Twist, '/robotis/walking/velocity', 10)
        self.walking_param_pub = self.node.create_publisher(WalkingParam, '/robotis/walking/set_params', 10)
        self.walking_command_pub = self.node.create_publisher(String, '/robotis/walking/command', 10)
        self.module_pub = self.node.create_publisher(String, '/robotis/enable_ctrl_module', 10)
        self.action_pub = self.node.create_publisher(Int32, '/robotis/action/page_num', 10)
        self.state_pub = self.node.create_publisher(String, '/communication/robot_state', 10)
        self.pub_landmarak = self.node.create_publisher(Int32,'/landmarak/posisi',10)
        self.head_command_pub = self.node.create_publisher(String, '/communication/head_command', 10)

        self.acc_buffer = []

        self.walking_state = "stop"
        self.is_running = False
        self._action_lock = threading.Lock()
        self._reset_timer = None
        self._head_enable_timer = None
        self.is_recovery = False
        self.is_rotating = False
        self.rotate_direction = None
        self.rotation_target_yaw = None
        self.rotate_initial_error = None
        self.follow_head_off= False 
        self.kick_happened = False 
        self.ball_posisi= String
       
        self.derajat_kamera  = 0.0
        self.jarak_bola = 0.0
        self.kick_cooldown = 2.0
        self.kick_last_time = 0.0
        self.current_yaw = 0.0
        self.saved_yaw = 0.0
        self.acc_x_filtered = 0.0
        self.acc_y_filtered = 0.0
        self.alpha = 0.2
        self.error = 0.0
        self.ball_index = 0


        self.ball_posisi = ""

        # Menyimpan 5 frame terakhir
        self.ball_position_history = deque(maxlen=5)
        self.filter_data = DataFilter(
            window_size=5,
            alpha=0.2
        )  

        # Posisi bola yang sudah difilter
        self.ball_posisi_stable = ""

        # Minimal jumlah vote agar posisi dianggap valid
        self.ball_position_min_votes = 3

        # State machine: FOLLOW_HEAD -> ROTATING -> FOLLOW_HEAD -> ... -> KICKING
        self.head_active = False
        self.rotate_start_time = None

        self.node.create_subscription(
            JointState, '/robotis/goal_joint_states', self.joint_derajat, 10,
            callback_group=self.node.sensor_group)
        self.node.create_subscription(
            Float32, '/vision/jarak_kamera', self.jarak_callback, 10,
            callback_group=self.node.sensor_group)

        # Group sendiri supaya sinyal page selesai tetap diterima saat
        # kick()/fall() sedang menunggu di thread lain.
        self._page_done = threading.Event()
        self._page_done.set()
        self.node.create_subscription(
            String, '/robotis/movement_done', self._movement_done_callback, 10,
            callback_group=MutuallyExclusiveCallbackGroup())

        self.node.get_logger().info('button_soccer_node GaitController siap ???')

    def enable_head_once(self):
        msg = String()
        msg.data = "head_control_module"
        self.module_pub.publish(msg)
        # self.node.get_logger().info("Head module ONN")

    def _enable_head_once_timer(self):
        # create_timer di rclpy periodik, jadi hapus timer setelah sekali jalan
        if self._head_enable_timer is not None:
            self.node.destroy_timer(self._head_enable_timer)
            self._head_enable_timer = None
        self.enable_head_once()

    def _movement_done_callback(self, msg):
        if msg.data in ("action", "action_failed"):
            self._page_done.set()

    def _play_page(self, page):
        """Mainkan action page dan tandai belum selesai sampai movement_done datang."""
        self._page_done.clear()
        self.init_pose(page)

    def _wait_page_done(self):
        if not self._page_done.wait(PAGE_DONE_TIMEOUT):
            self.node.get_logger().warn(
                f"Page belum selesai setelah {PAGE_DONE_TIMEOUT}s, lanjut")

    def start_walking(self):
        self.walking_param("start")
        twist = Twist()
        twist.linear.x = 0.03
        self.walking_vel_pub.publish(twist)

    def stop_walking(self):
        self.walking_param("stop")

    def joint_derajat(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name == 'head_pan':
                self.derajat_kamera = msg.position[i]

    def jarak_callback(self, msg: Float32):
        if msg.data <= 0.0:
            self.reset_ball_tracking()
            return
        self.jarak_bola = self.filter_data.update(float(msg.data))

    def reset_ball_tracking(self):
        self.filter_data = DataFilter(window_size=5, alpha=0.2)
        self.ball_position_history.clear()
        self.ball_posisi_stable = ""
        self.ball_posisi = ""
        self.jarak_bola = 0.0

    # ==========================================
    # SENSOR CALLBACK — HANYA UPDATE NILAI
    # ==========================================

    def disable_follow_head(self):
        if self.jarak_bola == None :
            self.orbit.stop()
            return

    def _claim_action(self):
        with self._action_lock:
            if self.is_running:
                return False
            self.is_running = True
            return True

    def start_rotating(self):
        self.is_rotating = True
        self.rotate_direction = None
        self.rotate_start_time = time.monotonic()

        # self.stop_walking()
        # time.sleep(0.2)

        # self.init_pose(80)

        # time.sleep(0.5)

        self.disable_head_control()


    def handle_rotating(self, error ,eror_yaw):

        # Timeout
        if time.monotonic() - self.rotate_start_time >= ROTATE_TIMEOUT:
            self.node.get_logger().warn("ROTATE TIMEOUT")
            self.finish_rotating()
            return

        # Yaw sudah align
        if abs(error) < eror_yaw:
            self.node.get_logger().info("ROTATION SELESAI")
            self.finish_rotating()
            return

        # Lakukan rotasi
        self.rotate_to_goal(error)


    def finish_rotating(self):

        self.orbit.stop()

        self.is_rotating = False
        self.rotate_direction = None
        self.rotate_start_time = None

        self.enable_head_control()

    def rotate_to_goal(self, error):

        if self.rotate_direction is None:
            self.rotate_direction = -1 if error < 0 else 1

        direction = self.rotate_direction

        # if self.rotate_direction == -1:
        #     x_amp = -0.006
        # else :
        #     x_amp = -0.004

        x_amp = -0.004

        y_amp = -direction * Y_ROTATED
        yaw_amp_deg = direction * YAW_ROTATED

        self.node.get_logger().info(
            f"ROTATE | Error: {error:.2f} | "
            f"Direction: {direction} | "
            f"Y: {y_amp:.3f} | "
            f"Yaw: {yaw_amp_deg:.2f}"
        )

        self.orbit.run_orbit(
            x_amp=x_amp,
            y_amp=y_amp,
            yaw_amp_deg=yaw_amp_deg,
            time_period=0.640
        )

    def enable_head_control(self):
        msg = String()
        msg.data = "head_control_module"
        self.module_pub.publish(msg)
        self.head_active = True

    def disable_head_control(self):
        # Sesuaikan nama module ini dengan module yang benar dipakai
        # oleh self.orbit.run_orbit() di setup robot kamu.
        msg = String()
        msg.data = "walking_module"
        self.module_pub.publish(msg)
        self.head_active = False
    

    def follow_head(self, target_pan=0.0, target_distance=0.0, derajat_cam=0.0 ,jarak_bola=0.0 ):
        if self.follow_head_off:
            return
        
        kp_pan = 0.50
        kp_x = 0.0015

        error_pan = math.degrees(target_pan - derajat_cam)
        error_dist = jarak_bola - target_distance

        yaw = kp_pan * error_pan
        yaw = max(min(yaw, 10.0), -10.0)

        if error_dist <= 0:
            x_amp = 0.0
        else:
            x_amp = kp_x * error_dist
            x_amp = max(min(x_amp, 0.01), 0.002)

        self.orbit.run_orbit(
            x_amp=x_amp,
            y_amp=0.0,
            yaw_amp_deg=-yaw
    )
    def normalize_error(self, error):
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        return error

    def decide_kick_side(self, posisi, pan, PAN =0.1):

        # 1. PAN menjadi prioritas jika melewati batas aman
        if pan > PAN:
            return KIRI

        elif pan < -PAN:
            return KANAN

        # 2. Jika PAN masih di tengah, gunakan voting frame
        # Tanpa data /vision/bola_posisi: pakai posisi stabil terakhir,
        # kalau belum ada perlakukan sebagai "tengah" (kick kanan)
        if posisi not in ("kiri", "tengah", "kanan"):
            if self.ball_posisi_stable == "kiri":
                return KIRI
            return KANAN

        self.ball_position_history.append(posisi)

        votes = {
            "kiri": self.ball_position_history.count("kiri"),
            "tengah": self.ball_position_history.count("tengah"),
            "kanan": self.ball_position_history.count("kanan")
        }

        stable_pos = max(votes, key=votes.get)

        if votes[stable_pos] >= self.ball_position_min_votes:
            self.ball_posisi_stable = stable_pos

        # 3. Tentukan arah tendangan berdasarkan frame stabil
        if self.ball_posisi_stable == "kiri":
            return KIRI

        elif self.ball_posisi_stable == "kanan":
            return KANAN

        elif self.ball_posisi_stable == "tengah":
            return KANAN

        return None
                
    def kick(self ,jarak_bola,error ,target_to_kick,error_yaw ,kiri ,kanan , posisi , pan,):    

        if self.is_running:
            return

        if time.time() - self.kick_last_time < self.kick_cooldown:
            return

        if jarak_bola == 0.0:
            self.node.get_logger().info("Menunggu jarak bola...")
            return

        if jarak_bola < target_to_kick and abs(error) < error_yaw:
            kick_side = self.decide_kick_side(posisi, pan)
            if kick_side is None:
                return

            if not self._claim_action():
                return

            self.node.get_logger().info(
                f"READY KICK | "
                f"jarak={jarak_bola:.2f} | "
                f"yaw_error={error:.2f} | "
                f"camera={self.ball_posisi_stable} | "
                f"pan={self.derajat_kamera:.2f}"
            )

            self.kick_happened = True

            self.reset_ball_tracking()

            self.stop_walking()

            self.publish_head_command("CENTER")

            time.sleep(0.2)

            self._play_page(80)

            time.sleep(0.5)

            if kick_side == KIRI:
                self.node.get_logger().info(f"jarak bola kick = {jarak_bola}")
                self.node.get_logger().info("KICK KIRI")
                self.start_recovery(kiri)

            elif kick_side == KANAN:
                self.node.get_logger().info(f"jarak bola kick = {jarak_bola}")
                self.node.get_logger().info("KICK KANAN")
                self.start_recovery(kanan)

    def fall(self, acc_x ,treshold,action_page_front,action_page_back ):
        self.acc_x_filtered = (self.alpha * acc_x + (1.0 - self.alpha) * self.acc_x_filtered)
       
        if self.is_running:
            return
        if self.acc_x_filtered < -treshold or self.acc_x_filtered > treshold:
            if not self._claim_action():
                return
            self.node.get_logger().warn("Jatuh terdeteksi, memulai recovery...")
            self.node.get_logger().info(f"acc = {self.acc_x_filtered}")
            
            self.stop_walking()
            self.publish_head_command("CENTER")

            time.sleep(0.2)

            self._play_page(80)

            time.sleep(0.5)

            time.sleep(0.5)  # kasih waktu module aktif
            if self.acc_x_filtered < -treshold:
                self.node.get_logger().warn("Jatuh DEPAN")
                self.start_recovery(action_page_front)
            elif self.acc_x_filtered > treshold:
                self.node.get_logger().warn("Jatuh BELAKANG")
                self.start_recovery(action_page_back)

    def start_recovery(self, page):
        self.publish_state("RECOVER")
        # time.sleep(0.5)
        # self.publish_head_command("CENTER")
  
        time.sleep(0.5)
        # aktifkan action module
        self.action_param("action_module")
        time.sleep(0.5)
        # Page 80 dari kick()/fall() harus selesai dulu, kalau tidak page ini ditolak
        self._wait_page_done()
        self._play_page(page)

        if self._reset_timer:
            self._reset_timer.cancel()

        self._reset_timer = self.node.create_timer(
            ACTION_DURATION,
            self.finish_recovery
        )
        self.led.set_rgb_color('GREEN')
        time.sleep(1.0)
        self.led.set_rgb_color('OFF')


    def finish_recovery(self):
        self.node.get_logger().info("Recovery selesai -> balik NORMAL")
        self.kick_last_time = time.time()

        # Jangan ganti module sebelum page tendang/bangun selesai
        self._wait_page_done()
        self.action_param("walking_module")
        if self._head_enable_timer is not None:
            self.node.destroy_timer(self._head_enable_timer)
        self._head_enable_timer = self.node.create_timer(0.5, self._enable_head_once_timer)
        self.publish_state("WALK")
        time.sleep(0.5)
        self.publish_head_command("UNLOCK")
        time.sleep(2.0)  # kasih waktu module aktif

        # Jalan lagi hanya kalau sebelumnya robot memang sedang follow-head
        if getattr(self.node, "walking", False):
            self.node.get_logger().info("Recovery selesai -> mulai jalan lagi")
            self.start_walking()

        if self.kick_happened:
            self.ball_index = (self.ball_index + 1) % 4
            self.kick_happened = False
            self.node.get_logger().info(f"Pindah ke bola ke-{self.ball_index + 1}")

        self.reset_ball_tracking()
        self.is_running = False
        # Balik ke fase awal setelah recovery/kick selesai
        self.head_active = False  # akan di-enable lagi oleh enable_head_once/handle_follow_head
        self.is_rotating = False
        self.rotate_direction = None
        self.rotate_initial_error = None
        self.rotate_start_time = None

        self.init_pose(80)
        if self._reset_timer:
            self._reset_timer.cancel()
            self._reset_timer = None

    def ball_position(self,posisi):

        if posisi not in ["kiri", "tengah", "kanan"]:
            self.node.get_logger().warn(
                f"Posisi bola tidak dikenal: {posisi}"
            )
            return

        # Simpan frame terbaru
        self.ball_position_history.append(posisi)

        # Hitung jumlah vote
        kiri = self.ball_position_history.count("kiri")
        tengah = self.ball_position_history.count("tengah")
        kanan = self.ball_position_history.count("kanan")

        votes = {
            "kiri": kiri,
            "tengah": tengah,
            "kanan": kanan
        }

        posisi_terbanyak = max(votes, key=votes.get)
        jumlah_vote = votes[posisi_terbanyak]

        # Hanya ubah posisi stabil kalau sudah cukup kuat
        if jumlah_vote >= self.ball_position_min_votes:

            self.ball_posisi_stable = posisi_terbanyak

            self.node.get_logger().debug(
                f"BALL FILTER | "
                f"K={kiri} "
                f"T={tengah} "
                f"Kn={kanan} "
                f"-> {self.ball_posisi_stable}"
            )

        # Posisi raw tetap disimpan
        self.ball_posisi = posisi

    def publish_head_command(self, command):
        msg = String()
        msg.data = command
        self.head_command_pub.publish(msg)

    def walking_param(self, param):
        cmd = String()
        cmd.data = param
        self.walking_command_pub.publish(cmd)

    def action_param(self, param):
        cmd = String()
        cmd.data = param
        self.module_pub.publish(cmd)

    def init_pose(self, action_page):
        page = Int32()
        page.data = int(action_page)
        self.action_pub.publish(page)

    def publish_state(self, state):
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)
