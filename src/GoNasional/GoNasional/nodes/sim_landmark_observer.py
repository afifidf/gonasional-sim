"""Simulation stand-in for the landmark detector.

Takes the ground-truth robot pose from the Webots supervisor and publishes the
landmarks the camera could plausibly see, with noise and dropouts, on the same
/perception/landmarks topic a real goalpost/line detector will use. Nothing
downstream knows the difference.
"""

import math
import random

import rclpy
from aroc_interfaces.msg import Landmark, LandmarkArray
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from ..core.field_model import GOALPOST, FieldModel
from ..core.geometry import wrap


class SimLandmarkObserver(Node):

    def __init__(self):
        super().__init__('sim_landmark_observer')
        p = self.declare_parameter
        self.field = FieldModel(
            length=p('field.length', 9.0).value,
            width=p('field.width', 6.0).value,
            goal_width=p('field.goal_width', 2.6).value,
            penalty_mark_distance=p('field.penalty_mark_distance', 1.5).value)
        self.hfov = math.radians(p('camera.hfov_deg', 65.2).value)
        self.vfov = math.radians(p('camera.vfov_deg', 51.0).value)
        self.camera_height = p('camera.height_m', 0.46).value
        self.max_range = p('landmarks.max_range', 4.5).value
        self.range_noise_rel = p('landmarks.range_noise_rel', 0.08).value
        self.bearing_noise = p('landmarks.bearing_noise', 0.03).value
        self.dropout = p('landmarks.dropout', 0.15).value
        self.rng = random.Random(p('landmarks.seed', 1).value)

        self.pose = None
        self.head_pan = 0.0
        self.head_tilt = 0.0
        self.pub = self.create_publisher(LandmarkArray, '/perception/landmarks', 10)
        self.create_subscription(PoseStamped, '/aroc_sim/robot_pose', self._on_pose, 10)
        self.create_subscription(
            JointState, '/robotis/present_joint_states', self._on_joints, 10)
        self.create_timer(1.0 / p('landmarks.rate_hz', 10.0).value, self._publish)

    def _on_pose(self, msg):
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.position.x, msg.pose.position.y, yaw)

    def _on_joints(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name == 'head_pan':
                self.head_pan = position
            elif name == 'head_tilt':
                self.head_tilt = position

    def _visible(self, type_id, rng_m, bearing):
        if not 0.3 < rng_m < self.max_range:
            return False
        if abs(wrap(bearing - self.head_pan)) > self.hfov / 2.0 - 0.05:
            return False
        # Elevation of the landmark's base as seen from the camera.
        elevation = -math.atan2(self.camera_height, rng_m)
        if elevation < self.head_tilt - self.vfov / 2.0:
            return False
        # Flat marks must be inside the image; a goalpost is tall enough that
        # part of it is in view whenever its base is below the top edge.
        if type_id != GOALPOST and elevation > self.head_tilt + self.vfov / 2.0:
            return False
        return True

    def _publish(self):
        if self.pose is None:
            return
        x, y, yaw = self.pose
        out = LandmarkArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_footprint'
        for type_id, points in self.field.landmarks.items():
            for lx, ly in points:
                rng_m = math.hypot(lx - x, ly - y)
                bearing = wrap(math.atan2(ly - y, lx - x) - yaw)
                if not self._visible(type_id, rng_m, bearing):
                    continue
                if self.rng.random() < self.dropout:
                    continue
                out.landmarks.append(Landmark(
                    type=int(type_id),
                    range=float(rng_m * (1.0 + self.rng.gauss(0.0, self.range_noise_rel))),
                    bearing=float(bearing + self.rng.gauss(0.0, self.bearing_noise)),
                    confidence=0.9))
        self.pub.publish(out)


def main():
    rclpy.init()
    node = SimLandmarkObserver()
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
