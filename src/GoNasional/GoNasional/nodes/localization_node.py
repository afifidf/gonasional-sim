"""World model: robot pose (particle filter) and ball state (Kalman filter).

Inputs:
  /motion/walk_feedback   geometry_msgs/TwistStamped   walking odometry
  /robotis/open_cr/yaw    std_msgs/Float64 (degrees)   IMU heading
  /perception/landmarks   aroc_interfaces/LandmarkArray
  /perception/ball        geometry_msgs/PointStamped (base_footprint)
  /initialpose            geometry_msgs/PoseWithCovarianceStamped (RViz "2D Pose Estimate")
  /localization/reset     std_msgs/String   start | reentry | uniform

Outputs:
  /world_model            aroc_interfaces/WorldModel
  /localization/pose      geometry_msgs/PoseWithCovarianceStamped (frame: field)
  /localization/particles geometry_msgs/PoseArray
  tf  field -> base_footprint
"""

import math

import rclpy
from aroc_interfaces.msg import LandmarkArray, WorldModel
from geometry_msgs.msg import (Pose, PoseArray, PoseWithCovarianceStamped,
                               PointStamped, TransformStamped, TwistStamped)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64, String
from tf2_ros import TransformBroadcaster

from ..core.ball_filter import BallFilter
from ..core.field_model import FieldModel
from ..core.geometry import to_field, to_robot, wrap
from ..core.mcl import LOST, TRACKING, UNCERTAIN, MonteCarloLocalization


def _yaw_quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class LocalizationNode(Node):

    def __init__(self):
        super().__init__('localization')
        p = self.declare_parameter
        localized_std = p('localized_std_xy', 0.45).value
        self.field = FieldModel(
            length=p('field.length', 9.0).value,
            width=p('field.width', 6.0).value,
            goal_width=p('field.goal_width', 2.6).value,
            penalty_mark_distance=p('field.penalty_mark_distance', 1.5).value)
        self.mcl = MonteCarloLocalization(
            self.field,
            n_particles=p('mcl.particles', 300).value,
            odom_noise_xy=p('mcl.odom_noise_xy', 0.25).value,
            odom_noise_theta=p('mcl.odom_noise_theta', 0.10).value,
            range_sigma_base=p('mcl.range_sigma_base', 0.10).value,
            range_sigma_rel=p('mcl.range_sigma_rel', 0.15).value,
            bearing_sigma=p('mcl.bearing_sigma', 0.10).value,
            max_random_fraction=p('mcl.max_random_fraction', 0.10).value,
            gate_chi2=p('mcl.gate_chi2', 13.8).value,
            tracking_std_xy=localized_std,
            lost_after=p('mcl.lost_after', 5).value)
        self.ball = BallFilter(
            friction=p('ball.friction', 0.6).value,
            visible_timeout=p('ball.visible_timeout', 0.5).value,
            forget_timeout=p('ball.forget_timeout', 6.0).value)

        self.start_pose = p('start_pose', [-0.5, 0.0, 0.0]).value
        self.start_std_xy = p('start_std_xy', 0.3).value
        self.localized_std = localized_std
        self.yaw_sign = p('imu.yaw_sign', 1.0).value
        self.imu_timeout = p('imu.timeout', 0.5).value
        self.min_confidence = p('landmarks.min_confidence', 0.3).value
        self.publish_tf = p('publish_tf', True).value
        # Without any accepted landmark for this long the estimate is only dead
        # reckoning (or the robot was carried away unnoticed): not localized.
        self.landmark_timeout = p('landmark_timeout', 15.0).value
        self._last_landmark_time = None
        rate = p('rate_hz', 20.0).value

        self.mcl.init_pose(*self.start_pose, std_xy=self.start_std_xy, std_theta=0.1)
        self._twist = (0.0, 0.0, 0.0)
        self._twist_time = None
        self._imu_yaw = None
        self._imu_time = None
        self._imu_yaw_used = None
        self._last_step = None
        self._estimate = (*self.start_pose, self.start_std_xy, 0.1)
        self._state_logged = None
        self._started_at = None

        self.wm_pub = self.create_publisher(WorldModel, '/world_model', 10)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/localization/pose', 10)
        self.particles_pub = self.create_publisher(PoseArray, '/localization/particles', 2)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(TwistStamped, '/motion/walk_feedback', self._on_twist, 10)
        self.create_subscription(
            Float64, '/robotis/open_cr/yaw', self._on_yaw, qos_profile_sensor_data)
        self.create_subscription(LandmarkArray, '/perception/landmarks', self._on_landmarks, 10)
        self.create_subscription(PointStamped, '/perception/ball', self._on_ball, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self._on_initial_pose, 1)
        self.create_subscription(String, '/localization/reset', self._on_reset, 1)

        self.create_timer(1.0 / rate, self._step)
        self.create_timer(0.5, self._publish_particles)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------ callbacks

    def _on_twist(self, msg):
        self._twist = (msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)
        self._twist_time = self._now()

    def _on_yaw(self, msg):
        self._imu_yaw = math.radians(self.yaw_sign * msg.data)
        self._imu_time = self._now()

    def _on_landmarks(self, msg):
        observations = [(lm.type, lm.range, lm.bearing) for lm in msg.landmarks
                        if lm.confidence >= self.min_confidence]
        if observations and self.mcl.update_landmarks(observations):
            self._last_landmark_time = self._now()

    def _on_ball(self, msg):
        now = self._now()
        pose = self._estimate[:3]
        fx, fy = to_field(pose, (msg.point.x, msg.point.y))
        self.ball.update(fx, fy, math.hypot(msg.point.x, msg.point.y), now)

    def _on_initial_pose(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.mcl.init_pose(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.ball.reset()
        self._last_landmark_time, self._started_at = None, self._now()
        self.get_logger().info('Pose reset from /initialpose')

    def _on_reset(self, msg):
        mode = msg.data.strip().lower()
        theta = self._estimate[2]
        if mode == 'start':
            self.mcl.init_pose(*self.start_pose, std_xy=self.start_std_xy, std_theta=0.1)
        elif mode == 'reentry':
            # Back from a penalty: somewhere on either touchline of our own
            # half. The heading is NOT assumed: the IMU kept integrating while
            # the robot was carried, so the current heading estimate is still
            # right and is what tells the two touchlines apart.
            hw = self.field.width / 2.0
            xs = [-self.field.length * f for f in (0.1, 0.2, 0.3, 0.4)]
            poses = [(x, side * hw, theta) for x in xs for side in (1.0, -1.0)]
            self.mcl.init_hypotheses(poses, std_xy=0.3, std_theta=0.15)
        elif mode == 'uniform':
            self.mcl.init_uniform(theta=theta)
        else:
            return
        self.ball.reset()
        self._last_landmark_time, self._started_at = None, self._now()
        self.get_logger().info(f'Localization reset: {mode}')

    # ----------------------------------------------------------------- step

    def _step(self):
        now = self._now()
        if self._started_at is None:
            self._started_at = now
        dt = 0.0 if self._last_step is None else min(now - self._last_step, 0.2)
        self._last_step = now

        vx, vy, wz = self._twist
        if self._twist_time is None or now - self._twist_time > 0.5:
            vx = vy = wz = 0.0

        # Heading change straight from the gyro-integrated yaw when available;
        # it is far better than counting steps.
        imu_fresh = self._imu_time is not None and now - self._imu_time <= self.imu_timeout
        if imu_fresh and self._imu_yaw_used is not None:
            dtheta = wrap(self._imu_yaw - self._imu_yaw_used)
        else:
            dtheta = wz * dt
        if imu_fresh:
            self._imu_yaw_used = self._imu_yaw
        else:
            self._imu_yaw_used = None

        if dt > 0.0:
            self.mcl.predict(vx * dt, vy * dt, dtheta)
            self.ball.predict(dt)
        self._estimate = self.mcl.estimate()
        self._publish(now)

    def _publish(self, now):
        x, y, theta, std_xy, std_theta = self._estimate
        stamp = self.get_clock().now().to_msg()
        qz, qw = _yaw_quat(theta)

        wm = WorldModel()
        wm.header.stamp = stamp
        wm.header.frame_id = 'field'
        wm.pose.x, wm.pose.y, wm.pose.theta = x, y, theta
        wm.pose_std_xy = float(std_xy)
        wm.pose_std_theta = float(std_theta)
        state = self.mcl.state
        last = self._last_landmark_time if self._last_landmark_time is not None \
            else self._started_at
        stale = last is not None and now - last > self.landmark_timeout
        if stale and state == TRACKING:
            state = UNCERTAIN
        wm.localized = bool(std_xy < self.localized_std and state != LOST and not stale)
        wm.localization_state = int(state)
        if state != self._state_logged:
            self._state_logged = state
            self.get_logger().info(
                'localization: ' + ('TRACKING', 'UNCERTAIN', 'LOST')[state]
                + f' (rejected landmarks so far: {self.mcl.rejected})')
        wm.ball_known = bool(self.ball.known(now))
        wm.ball_visible = bool(self.ball.visible(now))
        age = self.ball.age(now)
        wm.ball_age = float(age) if math.isfinite(age) else -1.0
        if self.ball.initialized:
            bx, by = self.ball.position
            wm.ball_field.x, wm.ball_field.y = bx, by
            wm.ball_velocity.x, wm.ball_velocity.y = self.ball.velocity
            wm.ball_robot.x, wm.ball_robot.y = to_robot((x, y, theta), (bx, by))
            wm.ball_std = self.ball.std
        self.wm_pub.publish(wm)

        pose = PoseWithCovarianceStamped()
        pose.header = wm.header
        pose.pose.pose.position.x, pose.pose.pose.position.y = x, y
        pose.pose.pose.orientation.z, pose.pose.pose.orientation.w = qz, qw
        pose.pose.covariance[0] = pose.pose.covariance[7] = std_xy ** 2 / 2.0
        pose.pose.covariance[35] = std_theta ** 2
        self.pose_pub.publish(pose)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header = wm.header
            tf.child_frame_id = 'base_footprint'
            tf.transform.translation.x, tf.transform.translation.y = x, y
            tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
            self.tf_broadcaster.sendTransform(tf)

    def _publish_particles(self):
        if self.particles_pub.get_subscription_count() == 0:
            return
        out = PoseArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'field'
        for x, y, theta in self.mcl.particles[::3]:
            pose = Pose()
            pose.position.x, pose.position.y = float(x), float(y)
            pose.orientation.z, pose.orientation.w = _yaw_quat(float(theta))
            out.poses.append(pose)
        self.particles_pub.publish(out)


def main():
    rclpy.init()
    node = LocalizationNode()
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
