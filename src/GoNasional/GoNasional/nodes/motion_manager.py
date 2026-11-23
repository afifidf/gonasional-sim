"""Motion manager: the only node that commands the OP3 framework.

Inputs (same in simulation and on the robot):
  /cmd_vel          geometry_msgs/Twist    walking velocity, m/s and rad/s
  /motion/command   std_msgs/String        stand | sit | stop
  /motion/kick      std_msgs/String        left | right
  /head/mode        std_msgs/String        track | scan | center
  /perception/ball_offset  geometry_msgs/Vector3Stamped  ball angle off the optical axis
  /robotis/open_cr/imu     sensor_msgs/Imu

Outputs:
  /motion/status         std_msgs/String   IDLE STANDING_UP READY WALKING KICKING GETTING_UP SITTING_DOWN
  /motion/walk_feedback  geometry_msgs/TwistStamped  velocity actually being walked (for odometry)
  /diagnostics

Nothing in here sleeps: stand-up, kick and get-up are step sequences advanced
from one 50 Hz timer, so fall detection keeps running during every one of them
and pre-empts whatever the robot was doing.
"""

import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist, TwistStamped, Vector3Stamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from ..core.fall_detector import FallDetector, FallState
from ..core.geometry import clamp
from ..core.head_controller import HeadController
from ..core.sequencer import Sequencer, Step
from ..hw.op3_interface import DEFAULT_WALKING, Op3Interface

TICK_HZ = 50.0


class MotionManager(Node):

    def __init__(self):
        super().__init__('motion_manager')
        p = self.declare_parameter

        # Action pages of the team's motion file.
        self.page_walk_ready = p('pages.walk_ready', 80).value
        self.page_sit = p('pages.sit', 15).value
        self.page_kick_left = p('pages.kick_left', 155).value
        self.page_kick_right = p('pages.kick_right', 156).value
        self.page_getup_front = p('pages.getup_front', 142).value
        self.page_getup_back = p('pages.getup_back', 143).value

        # cmd_vel -> step amplitude. speed = gain * amplitude, measured per robot.
        self.gain_x = p('walk.gain_x', 3.0).value          # (m/s) per m of x amplitude
        self.gain_y = p('walk.gain_y', 1.5).value
        self.gain_yaw = p('walk.gain_yaw', 1.5).value      # (rad/s) per rad of yaw amplitude
        self.max_x_amp = p('walk.max_x_amp', 0.010).value
        self.max_y_amp = p('walk.max_y_amp', 0.015).value
        self.max_yaw_amp = p('walk.max_yaw_amp', math.radians(10.0)).value
        self.amp_slew = p('walk.amp_slew_per_s', 0.03).value
        self.yaw_slew = p('walk.yaw_slew_per_s', 0.6).value
        self.cmd_timeout = p('walk.cmd_timeout', 0.5).value
        self.walk_stop_settle = p('walk.stop_settle', 1.0).value
        self.cmd_deadband = p('walk.cmd_deadband', 1e-3).value

        self.imu_timeout = p('safety.imu_timeout', 1.0).value
        self.max_getup_attempts = p('safety.max_getup_attempts', 3).value
        self.fall = FallDetector(
            fall_angle_deg=p('safety.fall_angle_deg', 55.0).value,
            recover_angle_deg=p('safety.recover_angle_deg', 30.0).value,
            debounce_s=p('safety.fall_debounce', 0.15).value,
            alpha=p('safety.accel_alpha', 0.2).value,
            forward_sign=p('safety.forward_acc_sign', -1.0).value)

        self.head = HeadController(
            pan_limit=p('head.pan_limit', 1.2).value,
            tilt_min=p('head.tilt_min', -1.2).value,
            tilt_max=p('head.tilt_max', 0.1).value,
            scan_rows=p('head.scan_rows', [-0.25, -0.70, -1.10]).value,
            scan_pan_rate=p('head.scan_pan_rate', 1.0).value,
            track_gain=p('head.track_gain', 0.5).value,
            lost_hold_s=p('head.lost_hold', 1.0).value,
            center_tilt=p('head.center_tilt', -0.3).value,
            scan_step=p('head.scan_step', 0.6).value,
            scan_dwell=p('head.scan_dwell', 0.5).value)

        walking = {name: p(f'walking.{name}', float(value)).value
                   for name, value in DEFAULT_WALKING.items()}
        self.io = Op3Interface(self, walking)
        self.seq = Sequencer(on_timeout=self._on_step_timeout)

        self.state = 'IDLE'
        self.head_enabled = False
        self._cmd = (0.0, 0.0, 0.0)
        self._cmd_time = None
        self._amp = [0.0, 0.0, 0.0]
        self._walking = False
        self._last_start = 0.0
        self._kick_request = None
        self._getup_attempts = 0
        self._getup_page = self.page_getup_front
        self._imu_time = None
        self._last_tick = None
        self._max_dt = 0.0
        self._tick_count = 0

        self.status_pub = self.create_publisher(String, '/motion/status', 10)
        self.feedback_pub = self.create_publisher(TwistStamped, '/motion/walk_feedback', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(String, '/motion/command', self._on_command, 10)
        self.create_subscription(String, '/motion/kick', self._on_kick, 10)
        self.create_subscription(String, '/head/mode', self._on_head_mode, 10)
        self.create_subscription(
            Vector3Stamped, '/perception/ball_offset', self._on_ball_offset,
            qos_profile_sensor_data)
        self.create_subscription(
            Imu, '/robotis/open_cr/imu', self._on_imu, qos_profile_sensor_data)

        self.create_timer(1.0 / TICK_HZ, self._tick)
        self.create_timer(1.0, self._publish_diagnostics)
        self.get_logger().info('motion_manager ready (state IDLE)')

    # ------------------------------------------------------------ callbacks

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd_vel(self, msg):
        self._cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self._cmd_time = self._now()

    def _on_command(self, msg):
        command = msg.data.strip().lower()
        if command == 'stand' and self.state == 'IDLE':
            if not self._joints_alive():
                # op3_manager not up yet (or the simulator not connected):
                # commands sent now are lost and can leave its modules stuck.
                self.get_logger().warn(
                    'stand ignored: no /robotis/present_joint_states yet (op3_manager running?)',
                    throttle_duration_sec=5.0)
                return
            self._start_stand()
        elif command == 'sit' and self.state in ('READY', 'WALKING'):
            self._start_sit()
        elif command == 'stop':
            self._cmd = (0.0, 0.0, 0.0)

    def _joints_alive(self, max_age=1.0):
        stamp = self.io.joints_stamp
        return stamp is not None and \
            (self.get_clock().now() - stamp).nanoseconds * 1e-9 < max_age

    def _on_kick(self, msg):
        side = msg.data.strip().lower()
        if side in ('left', 'right') and self.state in ('READY', 'WALKING'):
            self._kick_request = side

    def _on_head_mode(self, msg):
        self.head.set_mode(msg.data.strip().lower())

    def _on_ball_offset(self, msg):
        self.head.observe(msg.vector.x, msg.vector.y,
                          Time.from_msg(msg.header.stamp).nanoseconds * 1e-9)

    def _on_imu(self, msg):
        now = self._now()
        self._imu_time = now
        a = msg.linear_acceleration
        self.fall.update(a.x, a.y, a.z, now)

    def _on_step_timeout(self, sequence, step):
        self.get_logger().warn(f'{sequence}: step "{step}" timed out, continuing')

    # ----------------------------------------------------------------- tick

    def _tick(self):
        now = self._now()
        if self._last_tick is not None:
            self._max_dt = max(self._max_dt, now - self._last_tick)
        self._last_tick = now
        self._tick_count += 1

        # Safety first: a fall pre-empts any sequence except one that is
        # already getting the robot up, or the robot sitting on purpose.
        if self.fall.fallen and self.state not in ('IDLE', 'GETTING_UP', 'SITTING_DOWN'):
            self.get_logger().warn(
                f'Fall detected ({self.fall.state.value}, tilt '
                f'{math.degrees(self.fall.tilt):.0f} deg) while {self.state}')
            self._getup_attempts = 0
            self._start_getup()

        if self.seq.active:
            self.seq.tick(now)
        elif self.state in ('READY', 'WALKING'):
            if self._kick_request is not None:
                self._start_kick(self._kick_request)
            else:
                self._update_walk(now)
        self._kick_request = None

        if self.head_enabled and self._tick_count % 2 == 0:
            self.io.set_head(*self.head.update(now, (self.io.head_pan, self.io.head_tilt)))

        if self._tick_count % 5 == 0:
            self._publish_status(now)

    def _set_state(self, state):
        if state != self.state:
            self.get_logger().info(f'{self.state} -> {state}')
            self.state = state
            self.status_pub.publish(String(data=state))

    def _publish_status(self, now):
        self.status_pub.publish(String(data=self.state))
        fb = TwistStamped()
        fb.header.stamp = self.get_clock().now().to_msg()
        fb.header.frame_id = 'base_footprint'
        if self._walking:
            fb.twist.linear.x = self._amp[0] * self.gain_x
            fb.twist.linear.y = self._amp[1] * self.gain_y
            fb.twist.angular.z = self._amp[2] * self.gain_yaw
        self.feedback_pub.publish(fb)

    # -------------------------------------------------------------- walking

    def _update_walk(self, now):
        cmd = self._cmd
        stale = self._cmd_time is None or now - self._cmd_time > self.cmd_timeout
        imu_lost = self._imu_time is None or now - self._imu_time > self.imu_timeout
        if stale or imu_lost:
            # Watchdog: nobody is steering, or we are blind to falls. Stand still.
            cmd = (0.0, 0.0, 0.0)

        target = (
            clamp(cmd[0] / self.gain_x, -self.max_x_amp, self.max_x_amp),
            clamp(cmd[1] / self.gain_y, -self.max_y_amp, self.max_y_amp),
            clamp(cmd[2] / self.gain_yaw, -self.max_yaw_amp, self.max_yaw_amp),
        )
        moving = any(abs(c) > self.cmd_deadband for c in cmd)

        dt = 1.0 / TICK_HZ
        limits = (self.amp_slew * dt, self.amp_slew * dt, self.yaw_slew * dt)
        goal = target if moving else (0.0, 0.0, 0.0)
        for i in range(3):
            self._amp[i] += clamp(goal[i] - self._amp[i], -limits[i], limits[i])

        if moving:
            if not self._walking or now - self._last_start > 1.0:
                # "start" is idempotent in the walking module; repeating it
                # covers a message lost while control modules were switching.
                self.io.walk_start()
                self._last_start = now
            self._walking = True
            self._set_state('WALKING')
        elif self._walking and all(abs(a) < 1e-4 for a in self._amp):
            self._stop_walking()
            self._set_state('READY')

        if self._walking and self._tick_count % 5 == 0:
            self.io.set_walk(*self._amp)

    def _stop_walking(self):
        self._amp = [0.0, 0.0, 0.0]
        self.io.set_walk(0.0, 0.0, 0.0)
        self.io.walk_stop()
        self._walking = False

    # ------------------------------------------------------------ sequences

    def _enable_action_module(self):
        self.head_enabled = False
        self.io.enable_module('action_module')

    def _enable_head_module(self):
        self.io.enable_module('head_control_module')
        self.head.reset(self.io.head_pan, self.io.head_tilt)
        self.head_enabled = True

    def _resume_steps(self):
        """Tail shared by every sequence that ends with the robot ready to walk."""
        return [
            Step('walking_module', lambda: self.io.enable_module('walking_module'),
                 min_time=0.5),
            Step('head_module', self._enable_head_module, min_time=0.3),
        ]

    def _page_step(self, name, page, timeout, min_time=0.5):
        return Step(name, lambda: self.io.play_page(page),
                    done=lambda: self.io.page_done, min_time=min_time, timeout=timeout)

    def _start_stand(self):
        self._set_state('STANDING_UP')
        self.fall.reset()
        self.seq.start('stand', [
            Step('action_module', self._enable_action_module, min_time=0.5),
            self._page_step('walk_ready', self.page_walk_ready, timeout=6.0),
            *self._resume_steps(),
        ], on_finish=lambda: self._set_state('READY'))

    def _start_sit(self):
        self._set_state('SITTING_DOWN')
        self.seq.start('sit', [
            Step('stop_walk', self._stop_walking, min_time=self.walk_stop_settle),
            Step('action_module', self._enable_action_module, min_time=0.5),
            self._page_step('sit', self.page_sit, timeout=6.0),
        ], on_finish=lambda: self._set_state('IDLE'))

    def _start_kick(self, side):
        page = self.page_kick_left if side == 'left' else self.page_kick_right
        self._set_state('KICKING')
        self.seq.start('kick', [
            Step('stop_walk', self._stop_walking, min_time=self.walk_stop_settle),
            Step('action_module', self._enable_action_module, min_time=0.3),
            self._page_step('kick_page', page, timeout=8.0, min_time=1.0),
            *self._resume_steps(),
        ], on_finish=lambda: self._set_state('READY'))

    def _start_getup(self):
        self._getup_attempts += 1
        self._set_state('GETTING_UP')

        def choose_page():
            # Decide once the robot has come to rest, not at the first impact.
            front = self.fall.state is not FallState.FALLEN_BACK
            self._getup_page = self.page_getup_front if front else self.page_getup_back
            self._enable_action_module()

        self.seq.start('getup', [
            Step('stop_walk', self._stop_walking, min_time=1.0),
            Step('action_module', choose_page, min_time=0.5),
            Step('getup_page', lambda: self.io.play_page(self._getup_page),
                 done=lambda: self.io.page_done, min_time=1.0, timeout=15.0),
            Step('verify', self._verify_getup, min_time=0.5),
            self._page_step('walk_ready', self.page_walk_ready, timeout=6.0),
            *self._resume_steps(),
        ], on_finish=lambda: self._set_state('READY'))

    def _verify_getup(self):
        if not self.fall.fallen:
            return
        if self._getup_attempts < self.max_getup_attempts:
            self.get_logger().warn(
                f'Still down after get-up attempt {self._getup_attempts}, retrying')
            self._start_getup()
        else:
            # Give up rather than thrash the servos; a human has to help.
            self.get_logger().error('Get-up failed, staying down (send "stand" to retry)')
            self.seq.abort()
            self._set_state('IDLE')

    # ---------------------------------------------------------- diagnostics

    def _publish_diagnostics(self):
        now = self._now()
        imu_age = math.inf if self._imu_time is None else now - self._imu_time
        cmd_age = math.inf if self._cmd_time is None else now - self._cmd_time
        status = DiagnosticStatus(name='motion_manager', hardware_id='op3')
        if imu_age > self.imu_timeout:
            status.level, status.message = DiagnosticStatus.ERROR, 'IMU silent, walking inhibited'
        elif self._max_dt > 2.5 / TICK_HZ:
            status.level, status.message = DiagnosticStatus.WARN, 'control loop jitter'
        else:
            status.level, status.message = DiagnosticStatus.OK, self.state
        status.values = [
            KeyValue(key='state', value=self.state),
            KeyValue(key='sequence_step', value=str(self.seq.step_name)),
            KeyValue(key='imu_age_s', value=f'{imu_age:.3f}'),
            KeyValue(key='cmd_age_s', value=f'{cmd_age:.3f}'),
            KeyValue(key='tilt_deg', value=f'{math.degrees(self.fall.tilt):.1f}'),
            KeyValue(key='max_tick_dt_ms', value=f'{self._max_dt * 1e3:.1f}'),
            KeyValue(key='getup_attempts', value=str(self._getup_attempts)),
        ]
        self._max_dt = 0.0
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diag_pub.publish(array)


def main():
    rclpy.init()
    node = MotionManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
