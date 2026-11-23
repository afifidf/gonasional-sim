"""Turns the legacy detector outputs into typed, metric observations.

Legacy topics (published by main_vision + trigonocak on the robot and by the
Webots ball_tracker in simulation):
  /obj_detect          std_msgs/String  "cx,cy" pixel of the ball
  /vision/hasil_servo  std_msgs/Float32 ball distance in cm (0 = lost)

  /vision/detections   aroc_interfaces/DetectionArray  all YOLO boxes (main_vision)

Published here:
  /perception/ball_offset  geometry_msgs/Vector3Stamped
        x, y = angle of the ball off the optical axis (rad, left/up positive)
  /perception/ball         geometry_msgs/PointStamped in base_footprint (m)
  /perception/landmarks    aroc_interfaces/LandmarkArray (goalposts, marks, corners)

The ball position comes from inverse perspective mapping (core/ipm.py) when
the URDF transforms are available (robot_state_publisher, see
aroc_soccer.launch.py): the pixel ray is intersected with the plane at ball
height, using the camera pose relative to the soles. Without transforms it
falls back to the legacy distance plus bearing = head pan + pixel offset.

When /vision/detections is flowing, the ball also comes from it: every ball
box is checked against geometry (size vs IPM range, not inside the feet)
before the most confident survivor is used; /obj_detect is then ignored.

Landmarks always need the transforms (core/landmarks.py picks the floor pixel
of each box). Frames taken while the head pans or the body turns quickly are
skipped: blur plus joint-angle lag would put a post far from where it is.
"""

import math

import numpy as np
import rclpy
from aroc_interfaces.msg import DetectionArray, Landmark, LandmarkArray
from geometry_msgs.msg import PointStamped, Vector3Stamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformException, TransformListener

from ..core.ipm import CameraModel, camera_in_footprint, footprint_from_feet, pixel_to_ground
from ..core.landmarks import (BallConfig, Box, LandmarkConfig, MotionGate, extract_landmarks,
                              select_ball)

BODY = 'body_link'
CAMERA = 'cam_link'
LEFT_ANKLE = 'l_ank_roll_link'
RIGHT_ANKLE = 'r_ank_roll_link'


def _pose(transform):
    """geometry_msgs/Transform -> (3x3 rotation, position)."""
    q, t = transform.rotation, transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    rot = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    return rot, np.array([t.x, t.y, t.z])


class PerceptionBridge(Node):

    def __init__(self):
        super().__init__('perception_bridge')
        p = self.declare_parameter
        width = p('camera.width', 450).value
        height = p('camera.height', 337).value
        self.camera = CameraModel.from_params(
            width, height, p('camera.hfov_deg', 65.2).value,
            source_width=p('camera.source_width', 0).value,
            source_height=p('camera.source_height', 0).value,
            fx=p('camera.fx', 0.0).value, fy=p('camera.fy', 0.0).value,
            cx=p('camera.cx', 0.0).value, cy=p('camera.cy', 0.0).value)
        self.max_distance_age = p('distance_max_age', 0.3).value
        self.max_range = p('max_range', 5.0).value
        self.ball_radius = p('ball_radius', 0.07).value
        self.use_ipm = p('ipm.enabled', True).value
        # present_joint_states can stall ~0.35 s when the CPU is busy (measured in sim).
        self.max_tf_age = p('ipm.max_tf_age', 0.5).value
        # Read on every detection so it can be switched off at runtime to check IPM alone.
        p('ipm.legacy_fallback', True)
        self.width, self.height = width, height
        self.landmark_config = LandmarkConfig(
            min_confidence=p('landmarks.min_confidence', 0.5).value,
            max_range_post=p('landmarks.max_range_post', 5.5).value,
            max_range_mark=p('landmarks.max_range_mark', 4.0).value)
        self.gate = MotionGate(max_head_rate=p('landmarks.max_head_rate', 0.6).value,
                               max_body_rate=p('landmarks.max_body_rate', 0.8).value,
                               hold=p('landmarks.motion_hold', 0.3).value)
        self.ball_config = BallConfig(
            min_confidence=p('ball.min_confidence', 0.5).value,
            radius=self.ball_radius,
            size_ratio_max=p('ball.size_ratio_max', 1.8).value,
            max_range=self.max_range)
        self._detections_time = None
        self._stats = {'frames': 0, 'moving': 0, 'no_tf': 0, 'landmarks': 0, 'rejected': 0,
                       'ball_rejected': 0}

        self.head_pan = 0.0
        self._distance = 0.0
        self._distance_time = None
        self._ipm_active = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.offset_pub = self.create_publisher(Vector3Stamped, '/perception/ball_offset', 10)
        self.ball_pub = self.create_publisher(PointStamped, '/perception/ball', 10)
        self.landmark_pub = self.create_publisher(LandmarkArray, '/perception/landmarks', 10)
        self.create_subscription(DetectionArray, '/vision/detections', self._on_detections, 10)
        self.create_subscription(Imu, '/robotis/open_cr/imu', self._on_imu, 10)
        self.create_timer(10.0, self._log_stats)
        self.create_subscription(String, '/obj_detect', self._on_pixel, 10)
        self.create_subscription(Float32, '/vision/hasil_servo', self._on_distance, 10)
        self.create_subscription(
            JointState, '/robotis/present_joint_states', self._on_joints, 10)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_joints(self, msg):
        tilt = None
        for name, position in zip(msg.name, msg.position):
            if name == 'head_pan':
                self.head_pan = position
            elif name == 'head_tilt':
                tilt = position
        if tilt is not None:
            # Message stamps, not arrival time: joint states arrive in bursts
            # when the CPU is busy, which would fake huge head rates.
            stamp = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
            self.gate.on_head(self.head_pan, tilt, stamp if stamp > 0.0 else self._now())

    def _on_imu(self, msg):
        self.gate.on_gyro(msg.angular_velocity.z, self._now())

    def _on_distance(self, msg):
        self._distance = msg.data / 100.0
        self._distance_time = self._now()

    def _camera_in_footprint(self, stamp=None):
        """Camera pose in base_footprint from the URDF transforms, or None.

        stamp: image capture time; falls back to the latest transforms when the
        buffer has nothing for that instant (e.g. clocks of camera and joints differ).
        """
        try:
            poses = {}
            for frame in (CAMERA, LEFT_ANKLE, RIGHT_ANKLE):
                tf = None
                if stamp is not None:
                    try:
                        tf = self.tf_buffer.lookup_transform(BODY, frame, stamp)
                    except TransformException:
                        tf = None
                if tf is None:
                    tf = self.tf_buffer.lookup_transform(BODY, frame, Time())
                age = self._now() - Time.from_msg(tf.header.stamp).nanoseconds * 1e-9
                if age > self.max_tf_age:
                    return None
                poses[frame] = _pose(tf.transform)
        except TransformException:
            return None
        footprint = footprint_from_feet(poses[LEFT_ANKLE], poses[RIGHT_ANKLE])
        return camera_in_footprint(footprint, poses[CAMERA])

    def _publish_ball(self, u, v, point, stamp):
        offset = Vector3Stamped()
        offset.header.stamp = stamp
        offset.header.frame_id = 'camera'
        offset.vector.x, offset.vector.y = self.camera.angles(u, v)
        self.offset_pub.publish(offset)
        if point is None:
            return
        ball = PointStamped()
        ball.header.stamp = stamp
        ball.header.frame_id = 'base_footprint'
        ball.point.x, ball.point.y = point
        self.ball_pub.publish(ball)

    def _on_detections(self, msg):
        self._stats['frames'] += 1
        self._detections_time = self._now()
        boxes = [Box(d.label, d.confidence, d.x1, d.y1, d.x2, d.y2) for d in msg.detections]
        width, height = msg.width or self.width, msg.height or self.height
        if any(b.label == 'bola' for b in boxes):
            camera_fp = self._camera_in_footprint() if self.use_ipm else None
            if camera_fp is not None:
                self._set_mode(True)
                ball = select_ball(boxes, self.camera, camera_fp, width, height, self.ball_config)
                if ball is None:
                    self._stats['ball_rejected'] += 1
                else:
                    self._publish_ball(ball[0], ball[1], ball[2],
                                       self.get_clock().now().to_msg())
        if not any(b.label != 'bola' for b in boxes):
            return
        stamp_s = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if not self.gate.steady(stamp_s if stamp_s > 0.0 else self._now()):
            self._stats['moving'] += 1
            return
        stamp = Time.from_msg(msg.header.stamp)
        camera_fp = self._camera_in_footprint(stamp if stamp.nanoseconds > 0 else None)
        if camera_fp is None:
            self._stats['no_tf'] += 1
            return
        found = extract_landmarks(boxes, self.camera, camera_fp, width, height,
                                  self.landmark_config)
        if not found:
            self._stats['rejected'] += 1
            return
        self._stats['landmarks'] += len(found)
        out = LandmarkArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'base_footprint'
        for type_id, rng, bearing, conf in found:
            out.landmarks.append(Landmark(type=int(type_id), range=float(rng),
                                          bearing=float(bearing), confidence=float(conf)))
        self.landmark_pub.publish(out)

    def _log_stats(self):
        s = self._stats
        if s['frames']:
            self.get_logger().info(
                f"landmarks: {s['landmarks']} from {s['frames']} frames "
                f"(skipped {s['moving']} moving, {s['no_tf']} without transforms, "
                f"{s['rejected']} with only unusable boxes); "
                f"frames whose ball boxes all failed geometry: {s['ball_rejected']}")
        self._stats = dict.fromkeys(s, 0)

    def _set_mode(self, ipm):
        if ipm != self._ipm_active:
            self._ipm_active = ipm
            self.get_logger().info(
                'ball position: ' + ('IPM from URDF transforms' if ipm else
                                     'legacy distance + bearing (no URDF transforms)'))

    def _on_pixel(self, msg):
        try:
            cx, cy = (float(v) for v in msg.data.strip().split('\n')[0].split(','))
        except ValueError:
            self.get_logger().warn(f'Bad /obj_detect payload: {msg.data!r}',
                                   throttle_duration_sec=2.0)
            return
        if cx <= 0.0 or cy <= 0.0:
            return
        if self._detections_time is not None and self._now() - self._detections_time < 1.0:
            return   # the ball comes from /vision/detections (geometry-checked)

        stamp = self.get_clock().now().to_msg()
        offset_pan, offset_tilt = self.camera.angles(cx, cy)
        offset = Vector3Stamped()
        offset.header.stamp = stamp
        offset.header.frame_id = 'camera'
        offset.vector.x = offset_pan
        offset.vector.y = offset_tilt
        self.offset_pub.publish(offset)

        point = None
        camera_fp = self._camera_in_footprint() if self.use_ipm else None
        if camera_fp is not None:
            self._set_mode(True)
            point = pixel_to_ground(self.camera, cx, cy, camera_fp,
                                    plane_z=self.ball_radius, max_range=self.max_range)
        elif self.get_parameter('ipm.legacy_fallback').value:
            self._set_mode(False)
            fresh = (self._distance_time is not None
                     and self._now() - self._distance_time <= self.max_distance_age)
            if fresh and 0.0 < self._distance <= self.max_range:
                bearing = self.head_pan + offset_pan
                point = (self._distance * math.cos(bearing), self._distance * math.sin(bearing))
        if point is None:
            return

        ball = PointStamped()
        ball.header.stamp = stamp
        ball.header.frame_id = 'base_footprint'
        ball.point.x, ball.point.y = point
        self.ball_pub.publish(ball)


def main():
    rclpy.init()
    node = PerceptionBridge()
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
