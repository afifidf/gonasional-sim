"""Single owner of the command topics of the ROBOTIS OP3 framework.

Everything that makes the robot move goes through this class, so two parts of
the code can no longer fight over /robotis/walking/command or switch control
modules under each other. op3_manager exposes the same topics in Webots
(simulation:=true) and on the robot, so this layer is identical in both.
"""

import math

from op3_walking_module_msgs.msg import WalkingParam
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, String

WALKING_PARAM_FIELDS = (
    'init_x_offset', 'init_y_offset', 'init_z_offset', 'init_roll_offset',
    'init_pitch_offset', 'init_yaw_offset', 'period_time', 'dsp_ratio',
    'step_fb_ratio', 'z_move_amplitude', 'balance_hip_roll_gain',
    'balance_knee_gain', 'balance_ankle_roll_gain', 'balance_ankle_pitch_gain',
    'y_swap_amplitude', 'z_swap_amplitude', 'arm_swing_gain', 'pelvis_offset',
)

# Gait tuned by the team on the real robot (was hardcoded in op3_orbit_walk.py).
DEFAULT_WALKING = {
    'init_x_offset': -0.020, 'init_y_offset': 0.030, 'init_z_offset': 0.020,
    'init_roll_offset': 0.0, 'init_pitch_offset': 0.0, 'init_yaw_offset': 0.0,
    'hip_pitch_offset_deg': 10.0, 'period_time': 0.650, 'dsp_ratio': 0.20,
    'step_fb_ratio': 0.28, 'z_move_amplitude': 0.06,
    'balance_hip_roll_gain': 0.35, 'balance_knee_gain': 0.30,
    'balance_ankle_roll_gain': 0.70, 'balance_ankle_pitch_gain': 0.90,
    'y_swap_amplitude': 0.028, 'z_swap_amplitude': 0.006,
    'arm_swing_gain': 0.6, 'pelvis_offset': 0.0,
}


class Op3Interface:

    def __init__(self, node, walking=None):
        self.node = node
        self.walking = dict(DEFAULT_WALKING)
        if walking:
            self.walking.update(walking)

        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._module_pub = node.create_publisher(String, '/robotis/enable_ctrl_module', reliable)
        self._page_pub = node.create_publisher(Int32, '/robotis/action/page_num', reliable)
        self._walk_cmd_pub = node.create_publisher(String, '/robotis/walking/command', reliable)
        self._walk_param_pub = node.create_publisher(
            WalkingParam, '/robotis/walking/set_params', reliable)
        self._head_pub = node.create_publisher(
            JointState, '/robotis/head_control/set_joint_states', reliable)

        self.page_done = True
        self.page_failed = False
        self.head_pan = 0.0
        self.head_tilt = 0.0
        self.joints_stamp = None
        node.create_subscription(String, '/robotis/movement_done', self._on_movement_done, reliable)
        node.create_subscription(
            JointState, '/robotis/present_joint_states', self._on_joints, 10)

    # ------------------------------------------------------------- feedback

    def _on_movement_done(self, msg):
        if msg.data in ('action', 'action_failed'):
            self.page_done = True
            self.page_failed = msg.data == 'action_failed'

    def _on_joints(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name == 'head_pan':
                self.head_pan = position
            elif name == 'head_tilt':
                self.head_tilt = position
        self.joints_stamp = self.node.get_clock().now()

    # ------------------------------------------------------------- commands

    def enable_module(self, module):
        self._module_pub.publish(String(data=module))

    def play_page(self, page):
        self.page_done = False
        self.page_failed = False
        self._page_pub.publish(Int32(data=int(page)))

    def walk_start(self):
        self._walk_cmd_pub.publish(String(data='start'))

    def walk_stop(self):
        self._walk_cmd_pub.publish(String(data='stop'))

    def set_walk(self, x_amp, y_amp, yaw_amp):
        """Step amplitudes: x, y in metres, yaw in radians, per step."""
        w = self.walking
        p = WalkingParam()
        for name in WALKING_PARAM_FIELDS:
            setattr(p, name, float(w[name]))
        p.hip_pitch_offset = math.radians(w['hip_pitch_offset_deg'])
        p.move_aim_on = False
        p.balance_enable = True
        p.x_move_amplitude = float(x_amp)
        p.y_move_amplitude = float(y_amp)
        p.angle_move_amplitude = float(yaw_amp)
        self._walk_param_pub.publish(p)

    def set_head(self, pan, tilt):
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = ['head_pan', 'head_tilt']
        msg.position = [float(pan), float(tilt)]
        self._head_pub.publish(msg)
