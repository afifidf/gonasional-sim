#!/usr/bin/env python3

import math

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from op3_walking_module_msgs.msg import WalkingParam


class OP3OrbitWalker:

    def __init__(self, node):
        self.node = node

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.module_pub = self.node.create_publisher(
            String,
            "/robotis/enable_ctrl_module",
            qos,
        )

        self.command_pub = self.node.create_publisher(
            String,
            "/robotis/walking/command",
            qos,
        )

        self.param_pub = self.node.create_publisher(
            WalkingParam,
            "/robotis/walking/set_params",
            qos,
        )

        self._timer = None
        self._orbit_param = None

    @property
    def running(self):
        return self._timer is not None

    # ======================================================
    # Walking Parameter
    # ======================================================

    def build_param(self, x_amp=0.0, y_amp=0.0, yaw_amp_deg=0.0,time_preiod=0.650,z_amp=0.06):

        p = WalkingParam()

        # Initial Pose
        p.init_x_offset = -0.020
        p.init_y_offset = 0.030
        p.init_z_offset = 0.020

        p.init_roll_offset = 0.0
        p.init_pitch_offset = 0.0
        p.init_yaw_offset = 0.0

        p.hip_pitch_offset = math.radians(10.0)

        # Walking
        p.period_time = time_preiod
        p.dsp_ratio = 0.20
        p.step_fb_ratio = 0.28
        p.move_aim_on = False

        p.x_move_amplitude = x_amp
        p.y_move_amplitude = y_amp
        p.z_move_amplitude = z_amp
        p.angle_move_amplitude = math.radians(yaw_amp_deg)

        # Balance
        p.balance_enable = True
        p.balance_hip_roll_gain = 0.35
        p.balance_knee_gain = 0.30
        p.balance_ankle_roll_gain = 0.70
        p.balance_ankle_pitch_gain = 0.90

        p.y_swap_amplitude = 0.028
        p.z_swap_amplitude = 0.006

        p.arm_swing_gain = 0.6
        p.pelvis_offset = 0.0

        return p

    # def apply_param(self, x_amp=0.0, y_amp=0.0, yaw_amp_deg=0.0,
    #                 time_period=0.650, z_amp=0.06):
    #     """Apply param ke walking module di bringup (publish /robotis/walking/set_params)."""
    #     self.param = self.build_param(
    #         x_amp=x_amp,
    #         y_amp=y_amp,
    #         yaw_amp_deg=yaw_amp_deg,
    #         z_amp=z_amp,
    #         time_preiod=time_period,
    #     )

    #     self.param_pub.publish(self.param)

    #     self.node.get_logger().info(
    #         f"APPLY param x={x_amp:.3f}, "
    #         f"y={y_amp:.3f}, "
    #         f"yaw={yaw_amp_deg:.1f}, "
    #         f"period={time_period:.3f}"
    #     )

    # ======================================================
    # Helper
    # ======================================================

    def enable_walking_module(self):
        msg = String()
        msg.data = "walking_module"
        self.module_pub.publish(msg)

    def init_gyro(self):
        msg = String()
        msg.data = "balance_on"
        self.command_pub.publish(msg)

    def start(self):
        msg = String()
        msg.data = "start"
        self.command_pub.publish(msg)

    def stop(self):
        msg = String()
        msg.data = "stop"
        self.command_pub.publish(msg)

        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

    def _stop_orbit(self):

        self.node.get_logger().info("STOP ORBIT")

        self.stop()

        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

        if self._stop_timer is not None:
            self._stop_timer.cancel()
            self._stop_timer = None

    # ======================================================
    # Orbit
    # ======================================================

    def run_orbit(self,x_amp=0.000,y_amp=0.000,yaw_amp_deg=16.0,publish_hz=2.0,duration=0.0,
                  time_period=0.650, z_amp=0.06,
    ):

        # self.node.get_logger().info("RUN ORBIT")

        self.enable_walking_module()
        self.init_gyro()

        self._orbit_param = self.build_param(
            x_amp=x_amp,
            y_amp=y_amp,
            z_amp=z_amp,
            yaw_amp_deg=yaw_amp_deg,
            time_preiod=time_period,
        )

        self.node.get_logger().info(
            f"x={self._orbit_param.x_move_amplitude:.3f}, "
            f"y={self._orbit_param.y_move_amplitude:.3f}, "
            f"yaw={math.degrees(self._orbit_param.angle_move_amplitude):.1f}"
        )

        self.param_pub.publish(self._orbit_param)

        self.start()

        # Timer publish parameter
        if self._timer is not None:
            self._timer.cancel()

        self._timer = self.node.create_timer(
            1.0 / publish_hz,
            self._publish_orbit_param,
        )

        # Timer stop (opsional)
        if hasattr(self, "_stop_timer") and self._stop_timer is not None:
            self._stop_timer.cancel()

        if duration > 0:
            self._stop_timer = self.node.create_timer(
                duration,
                self._stop_orbit
            )

    def _publish_orbit_param(self):

        self.param_pub.publish(self._orbit_param)