import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from sensor_msgs.msg import JointState
from .op3_orbit_walk import OP3OrbitWalker
from .main_PID import PIDControl
import json
import time
import os
from .led import OpenCR_Led
import time
import math
from enum import Enum
# from .sec_action import create_param

HEAD_PAN      = 0
HEAD_TILT     = 1
YAW_SCAN      = -16.0
DURATION_SCAN = 2.0
TIME_SCAN     = 0.750

PID_CONFIG_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "pid_config.json")

DEFAULT_PID = {
    "pan":  {"kp": 0.7,  "ki": 0.0, "kd": 0.025},
    "tilt": {"kp": 0.77, "ki": 0.0, "kd": 0.015},
}


def load_pid_config():
    if os.path.exists(PID_CONFIG_FILE):
        try:
            with open(PID_CONFIG_FILE, "r") as f:
                data = json.load(f)
            for axis in ("pan", "tilt"):
                for param in ("kp", "ki", "kd"):
                    float(data[axis][param])
            return {axis: {p: float(data[axis][p]) for p in ("kp", "ki", "kd")}
                    for axis in ("pan", "tilt")}
        except (OSError, ValueError, KeyError, TypeError) as e:
            print(f"[PID] Gagal baca {PID_CONFIG_FILE}, pakai default: {e}")
    return {axis: dict(DEFAULT_PID[axis]) for axis in ("pan", "tilt")}

class HeadControl:
    
    def __init__(self, node):
        self.node = node

        self.orbit = OP3OrbitWalker(self.node)
        self.led = OpenCR_Led()

        self.scan_count = 0
        self.max_scan = 4      # jumlah scan sebelum orbit
        self.scan_dir = 1
        self.tilt_dir = 1

        # ── State dari sistem ────────────────────────────────────────────
        self.robot_state  = "NORMAL"
        self.head_mode    = "scan"
        self.head_enabled = False   # akan di-set True oleh enable_head_module()
        self.is_walking = False     # ngikut robot_state == "WALK" (lihat state_callback)
        self.orbit_running = False  # True selagi orbit sedang berjalan
        self._orbit_timer = None    # handle timer one-shot penanda orbit selesai
        self.ball_detected = False
        self.search_timer = None
        self.kick_lock = False

        # ── Subscriber ──────────────────────────────────────────────────
        self.node.create_subscription(String, '/communication/head_command', self.head_command_callback, 10)
                
        # self.create_subscription(
        #     String, '/communication/robot_state',
        #     self.state_callback, 10)

        # self.create_subscription(
        #     String, '/obj_detect',
        #     self.obj_callback, 10)

        self.node.create_subscription(
            String, '/pid_params',
            self._pid_params_callback, 10)

        # ── Publisher ────────────────────────────────────────────────────
        self.module_pub = self.node.create_publisher(
            String, '/robotis/enable_ctrl_module', 10)

        self._page_pub = self.node.create_publisher(
            Int32, '/robotis/action/page_num', 10)

        self.head_pub = self.node.create_publisher(
            JointState, '/robotis/head_control/set_joint_states', 10)
        
        self.state_pub = self.node.create_publisher(String, '/communication/head_state', 10)

        # ── PID ──────────────────────────────────────────────────────────
        self.pid = [PIDControl() for _ in range(2)]

        # self.pid[HEAD_TILT].setConstant(Kp=1.0, Ki=0.01,Kd=0.025)
        # self.pid[HEAD_TILT].setTime(Ti=10.0, Td=10.0)
        # self.pid[HEAD_TILT].setRange(0, 337, -1.5, 0.4)
        # self.pid[HEAD_TILT].setSetPoints(168.5)

        # self.pid[HEAD_PAN].setConstant(Kp=1.0, Ki=0.0001, Kd=0.02)
        # self.pid[HEAD_PAN].setTime(Ti=10.0, Td=10.0)
        # self.pid[HEAD_PAN].setRange(0, 450, -1.5, 1.2)
        # self.pid[HEAD_PAN].setSetPoints(225.0)

        pid_cfg = load_pid_config()
        pid_source = PID_CONFIG_FILE if os.path.exists(PID_CONFIG_FILE) else "default"
        self.node.get_logger().info(f"PID config ({pid_source}): {pid_cfg}")

        self.pid[HEAD_TILT].setConstant(
            Kp=pid_cfg["tilt"]["kp"], Ki=pid_cfg["tilt"]["ki"], Kd=pid_cfg["tilt"]["kd"])
        self.pid[HEAD_TILT].setTime(Ti=10.0, Td=10.0)
        self.pid[HEAD_TILT].setRange(0, 480, -1.5, 0.4)
        self.pid[HEAD_TILT].setSetPoints(240)

        self.pid[HEAD_PAN].setConstant(
            Kp=pid_cfg["pan"]["kp"], Ki=pid_cfg["pan"]["ki"], Kd=pid_cfg["pan"]["kd"])
        self.pid[HEAD_PAN].setTime(Ti=10.0, Td=10.0)
        self.pid[HEAD_PAN].setRange(0, 640, -1.5, 1.2)
        self.pid[HEAD_PAN].setSetPoints(320)

        for i in range(2):
            self.pid[i].Init()
            self.pid[i].setEnableWindUpLimit()
            self.pid[i].setEnableWindUpCrossing()

        # ── Scan parameter ───────────────────────────────────────────────
        self.pan       = 0.0
        self.tilt      = -0.3
        self.scan_dir  = 1

        self.last_detection_time = self.node.get_clock().now()
        self.scan_timeout        = 0.5

        # ── Transisi Track → Scan (hold posisi terakhir) ────────────────
        self.track_to_scan_start = None
        self.track_to_scan_delay = 1.5   # detik tahan sebelum scan
        self.hold_pan  = 0.0
        self.hold_tilt = 0.0

        self.node.get_logger().info("HeadControl READY")

   

    def on_detection(self, cxcy):
        found = False
        lines = cxcy.data.strip().split('\n')
        for line in lines:
            parts = line.strip().split(',')
            if len(parts) != 2:
                continue
            try:
                cx = int(parts[0])
                cy = int(parts[1])
            except ValueError:
                self.node.get_logger().warn(f"Gagal parse obj_detect: '{line}'")
                continue

            if cx <= 0 or cy <= 0:
                continue

            found = True

            CENTER_X, CENTER_Y = 225, 168
            DEADBAND = 8

            if abs(cx - CENTER_X) >= DEADBAND:
                self.pid[HEAD_PAN].setSetPoints(CENTER_X)
                self.pid[HEAD_PAN].calculate(cx)
                self.pan = self.pid[HEAD_PAN].getOutput()

            if abs(cy - CENTER_Y) >= DEADBAND:
                self.pid[HEAD_TILT].setSetPoints(CENTER_Y)
                self.pid[HEAD_TILT].calculate(cy)
                self.tilt = self.pid[HEAD_TILT].getOutput() - 0.3

        return found

    # ════════════════════════════════════════════════════════════════════
    #  PID TUNER CALLBACK
    # ════════════════════════════════════════════════════════════════════

    def _pid_params_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            pan  = data.get("pan",  {})
            tilt = data.get("tilt", {})

            if pan:
                self.pid[HEAD_PAN].setConstant(
                    Kp=float(pan.get("kp", self.pid[HEAD_PAN].Kp)),
                    Ki=float(pan.get("ki", self.pid[HEAD_PAN].Ki)),
                    Kd=float(pan.get("kd", self.pid[HEAD_PAN].Kd)),
                )
                self.pid[HEAD_PAN].Init()
                self.node.get_logger().info(f"[PID PAN]  {pan}")

            if tilt:
                self.pid[HEAD_TILT].setConstant(
                    Kp=float(tilt.get("kp", self.pid[HEAD_TILT].Kp)),
                    Ki=float(tilt.get("ki", self.pid[HEAD_TILT].Ki)),
                    Kd=float(tilt.get("kd", self.pid[HEAD_TILT].Kd)),
                )
                self.pid[HEAD_TILT].Init()
                self.node.get_logger().info(f"[PID TILT] {tilt}")

        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self.node.get_logger().warn(f"[PID Tuner] Gagal parse: {e}")

    # ════════════════════════════════════════════════════════════════════
    #  MODE TRANSITION
    # ════════════════════════════════════════════════════════════════════

    def _enter_scan_mode(self, from_last_position=False):
        self.head_mode = "scan"
        for i in range(2):
            self.pid[i].Init()
        
        if from_last_position:
            # Mulai scan dari posisi terakhir tracking (tidak reset ke center)
            self.scan_dir = -1
            self.tilt_dir = 1
            self.node.get_logger().info(
                f"HEAD → SCAN dari posisi last ({self.pan:.2f}, {self.tilt:.2f})"
            )
            self._scan_head()
        else:
            self.pan  = 0.0
            self.tilt = -0.3
            self.scan_dir = -1
            self.tilt_dir = 1
            self.node.get_logger().info("HEAD → SCAN MODE")
            self._scan_head()


    def _enter_track_mode(self):
        self.head_mode = "track"
        self.head_state("TRACKING")
        for i in range(2):
            self.pid[i].Init()
        self.node.get_logger().info("HEAD → TRACK MODE")

    # ════════════════════════════════════════════════════════════════════
    #  TRACK / SCAN
    # ════════════════════════════════════════════════════════════════════

    def _track_head(self):
        self._publish_servo(self.pan, self.tilt)

    def _scan_head(self,is_walking=False):
        # self.head_state("SCANNING")

        # =========================
        # PAN SCANNING KIRI-KANAN
        # =========================
        self.pan += 0.02 * self.scan_dir

        if self.pan >= 1.2:
            self.pan = 1.2
            self.scan_dir = -1

        elif self.pan <= -1.2:
            self.pan = -1.2
            self.scan_dir = 1

        self.tilt += 0.04 * self.tilt_dir
                
        if self.tilt >= 0.1:
            self.tilt = 0.1
            self.tilt_dir = -1

        elif self.tilt <= -1.2:
            self.tilt = -1.2
            self.tilt_dir = 1

            # satu kali sweep selesai (kiri -> kanan -> kiri)
            self.scan_count += 1
            self.node.get_logger().info(
                f"Scan selesai {self.scan_count}/{self.max_scan}"
            )
            self.orbit.stop()
            if self.scan_count >= self.max_scan:
                self.scan_count = 0

                # is_walking dikontrol dari luar (via /communication/robot_state,
                # lihat state_callback). Selama masih True dan belum ada orbit
                # yang aktif, picu orbit (pola: jalan sambil cari bola).
                if is_walking == True:
                
                    time.sleep(0.2)
                    self.pan = 0.0
                    self.tilt = -0.4
                    # self._publish_servo(self.pan, self.tilt)
                    self.node.get_logger().info(f"Scan orbit pan gass {self.pan:.2f} tilt {self.tilt:.2f}")
                    
                    self._start_orbit(
                        yaw_amp_deg=YAW_SCAN,
                        duration=DURATION_SCAN,
                        time_period=TIME_SCAN)
                    
                    self._publish_servo(self.pan, self.tilt)
                
                   
                    # time.sleep(6.2)

        self._publish_servo(self.pan, self.tilt)
        # self.node.get_logger().info(f"publis srvo {self.pan:.2f} tilt {self.tilt:.2f}")

    # ════════════════════════════════════════════════════════════════════
    #  ORBIT CONTROL
    # ════════════════════════════════════════════════════════════════════

    def _start_orbit(self, yaw_amp_deg, duration, time_period):
        """Mulai orbit dan tandai orbit_running=True sampai selesai/di-interupsi."""
        self.orbit_running = True
        self.orbit.run_orbit(
            yaw_amp_deg=yaw_amp_deg,
            duration=duration,
            time_period=time_period
        )

        # Timer one-shot: kalau tidak ada bola yang menginterupsi,
        # orbit dianggap selesai normal setelah `duration` detik.
        if self._orbit_timer is not None:
            self._orbit_timer.cancel()
        self._orbit_timer = self.node.create_timer(duration, self._on_orbit_finished)

    def _on_orbit_finished(self):
        """Dipanggil saat orbit selesai secara normal (tidak diinterupsi bola)."""
        if self._orbit_timer is not None:
            self._orbit_timer.cancel()
            self._orbit_timer = None

        if self.orbit_running:
            self.orbit_running = False
            # is_walking TIDAK direset di sini — is_walking murni ngikut
            # robot_state == "WALK" (state_callback). Kalau state masih WALK,
            # siklus scan→orbit akan otomatis berulang lagi.
            self.node.get_logger().info("Orbit selesai")

    def _stop_orbit(self):
        self.orbit.stop()
        self.orbit_running = False
        if self._orbit_timer is not None:
            self._orbit_timer.cancel()
            self._orbit_timer = None

    # ════════════════════════════════════════════════════════════════════
    #  MODULE CONTROL
    # ════════════════════════════════════════════════════════════════════

    def enable_action_module(self):
        mode = String()
        mode.data = "action_module"
        self.module_pub.publish(mode)

    def enable_head_module(self):
        mode = String()
        mode.data = "head_control_module"
        self.module_pub.publish(mode)
        self.head_enabled = True

    def play_page(self, num):
        cmd = Int32()
        cmd.data = num
        self._page_pub.publish(cmd)
    
    def head_state(self, state):
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)
        # self.node.get_logger().info(f"HEAD STATE PUBLISH: {state}")

    # ════════════════════════════════════════════════════════════════════
    #  SERVO PUBLISH
    # ════════════════════════════════════════════════════════════════════

    def _publish_servo(self, pan: float, tilt: float):
        if not self.head_enabled:
            return
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name     = ['head_pan', 'head_tilt']
        msg.position = [pan, tilt]
        self.head_pub.publish(msg)

    def head_command_callback(self, msg):

        command = msg.data.strip().upper()

        if command == "CENTER":

            self.kick_lock = True
            self.head_mode = "center"

            # Reset PID supaya error lama tidak langsung menggerakkan kepala
            for pid in self.pid:
                pid.Init()

            self.pan = 0.0

            self._publish_servo(0.0, self.tilt)

        elif command == "UNLOCK":

            self.kick_lock = False
            self.pan = 0.0