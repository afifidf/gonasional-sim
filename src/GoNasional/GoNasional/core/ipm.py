"""Inverse perspective mapping: image pixel -> point on the ground. numpy only, no ROS imports.

Frames
  body    : body_link of the OP3 URDF (input poses are expressed in it)
  footprint (base_footprint): origin on the ground between the soles, z up
            (normal of the soles), x = body forward projected on the ground
  camera  : cam_link of the URDF, optical convention (z forward, x right, y down)

The detector works on a resized frame (450x337 in main_vision). When the camera
is 16:9 (C920 at 1280x720) that resize squashes the image, so the horizontal
and vertical focal lengths in the resized frame differ; CameraModel keeps them
separate.
"""

import math

import numpy as np

# Sole of OP3 foot in the frame of {l,r}_ank_roll_link: centre of the 8 mm
# collision box (op3_description), bottom face. y is mirrored for the left foot.
SOLE_OFFSET_RIGHT = np.array([0.024, -0.0125, -0.0305])
SOLE_OFFSET_LEFT = np.array([0.024, 0.0125, -0.0305])


class CameraModel:
    """Pinhole intrinsics in the frame the detector reports pixels in."""

    def __init__(self, fx, fy, cx, cy):
        self.fx, self.fy, self.cx, self.cy = float(fx), float(fy), float(cx), float(cy)

    @classmethod
    def from_params(cls, width, height, hfov_deg, source_width=0, source_height=0,
                    fx=0.0, fy=0.0, cx=0.0, cy=0.0):
        """Intrinsics for a width x height detector frame resized from the source image.

        fx/fy/cx/cy are at source resolution (calibration); when fx <= 0 they are
        derived from the horizontal FOV assuming square source pixels. Without a
        source size the detector frame itself is taken as the source.
        """
        sw = source_width or width
        sh = source_height or height
        if fx <= 0.0:
            fx = (sw / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
            fy = fx
        if cx <= 0.0:
            cx, cy = sw / 2.0, sh / 2.0
        sx, sy = width / sw, height / sh
        return cls(fx * sx, fy * sy, cx * sx, cy * sy)

    def ray(self, u, v):
        """Unit-less ray (x right, y down, z forward) through pixel (u, v)."""
        return np.array([(u - self.cx) / self.fx, (v - self.cy) / self.fy, 1.0])

    def angles(self, u, v):
        """Angles of the pixel off the optical axis: (left positive, up positive), rad."""
        return (math.atan2(self.cx - u, self.fx), math.atan2(self.cy - v, self.fy))


def footprint_from_feet(left_ank, right_ank):
    """Footprint pose in the body frame from the two ankle-roll link poses.

    Each argument is (R, p): 3x3 rotation and position of the link in body.
    Returns (R_fp, origin): footprint axes as columns, origin on the lower sole.
    """
    soles = []
    normal = np.zeros(3)
    for (rot, pos), offset in ((left_ank, SOLE_OFFSET_LEFT), (right_ank, SOLE_OFFSET_RIGHT)):
        rot = np.asarray(rot, dtype=float)
        soles.append(np.asarray(pos, dtype=float) + rot @ offset)
        normal += rot[:, 2]
    normal /= np.linalg.norm(normal)

    forward = np.array([1.0, 0.0, 0.0]) - normal[0] * normal
    forward /= np.linalg.norm(forward)
    left = np.cross(normal, forward)
    rot_fp = np.column_stack([forward, left, normal])

    # Midpoint between the soles, lowered onto the support (lowest) sole so a
    # lifted swing foot does not raise the ground plane.
    mid = 0.5 * (soles[0] + soles[1])
    lowest = min(float(normal @ s) for s in soles)
    origin = mid - (float(normal @ mid) - lowest) * normal
    return rot_fp, origin


def camera_in_footprint(footprint, camera):
    """Camera pose (R, p) in body -> camera pose in footprint."""
    rot_fp, origin = footprint
    rot_cam, pos_cam = camera
    return rot_fp.T @ np.asarray(rot_cam, dtype=float), rot_fp.T @ (np.asarray(pos_cam) - origin)


def pixel_to_ground(model, u, v, camera_fp, plane_z=0.0, max_range=8.0):
    """Ground point (x, y) in footprint seen at pixel (u, v), or None.

    camera_fp: camera pose (R, p) in footprint. plane_z: height of the point
    above the ground (ball radius for a ball centre). None when the ray does
    not hit the plane in front of the camera or lands beyond max_range.
    """
    rot, pos = camera_fp
    direction = rot @ model.ray(u, v)
    if direction[2] >= -1e-6:
        return None
    t = (plane_z - pos[2]) / direction[2]
    if t <= 0.0:
        return None
    point = pos + t * direction
    if math.hypot(point[0], point[1]) > max_range:
        return None
    return float(point[0]), float(point[1])


def ground_to_pixel(model, point, camera_fp):
    """Inverse of pixel_to_ground for tests and gating: footprint point -> (u, v) or None."""
    rot, pos = camera_fp
    local = rot.T @ (np.asarray(point, dtype=float) - pos)
    if local[2] <= 1e-6:
        return None
    return (model.cx + model.fx * local[0] / local[2],
            model.cy + model.fy * local[1] / local[2])
