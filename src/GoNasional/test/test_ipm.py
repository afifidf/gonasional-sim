"""Unit tests for core/ipm.py. Run: python3 -m pytest test/test_ipm.py"""

import math

import numpy as np

from GoNasional.core.ipm import (SOLE_OFFSET_LEFT, SOLE_OFFSET_RIGHT, CameraModel,
                                 camera_in_footprint, footprint_from_feet, ground_to_pixel,
                                 pixel_to_ground)

# Optical frame (x right, y down, z forward) of a camera looking along +x with no tilt.
OPTICAL = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def tilted_camera(height, tilt_down):
    """Camera pose in footprint: at `height`, looking forward and `tilt_down` rad below horizon."""
    c, s = math.cos(tilt_down), math.sin(tilt_down)
    pitch = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    return pitch @ OPTICAL, np.array([0.0, 0.0, height])


def test_squashed_intrinsics_keep_angles():
    # C920 1280x720 resized to 450x337: same field of view, different fx and fy.
    model = CameraModel.from_params(450, 337, 70.0, 1280, 720)
    assert math.isclose(model.cx, 225.0) and math.isclose(model.cy, 168.5)
    assert math.isclose(model.fy / model.fx, (337 / 720) / (450 / 1280))
    # The image edge is still at half the horizontal field of view.
    assert math.isclose(model.angles(0.0, model.cy)[0], math.radians(35.0), rel_tol=1e-9)


def test_pixel_ground_roundtrip():
    model = CameraModel.from_params(450, 337, 78.0, 640, 360)
    camera = tilted_camera(0.45, math.radians(35.0))
    for point in ((0.5, 0.0, 0.07), (1.5, -0.4, 0.07), (3.0, 0.8, 0.07)):
        u, v = ground_to_pixel(model, point, camera)
        x, y = pixel_to_ground(model, u, v, camera, plane_z=0.07)
        assert math.isclose(x, point[0], abs_tol=1e-9) and math.isclose(y, point[1], abs_tol=1e-9)


def test_pixel_above_horizon_has_no_ground_point():
    model = CameraModel.from_params(450, 337, 78.0, 640, 360)
    camera = tilted_camera(0.45, 0.0)
    assert pixel_to_ground(model, 225.0, 10.0, camera) is None   # sky
    assert pixel_to_ground(model, 225.0, 330.0, camera, max_range=1.0) is None  # too far


def test_footprint_between_flat_feet():
    eye = np.eye(3)
    left = (eye, np.array([0.0, 0.035, -0.25]))
    right = (eye, np.array([0.0, -0.035, -0.25]))
    rot, origin = footprint_from_feet(left, right)
    assert np.allclose(rot, eye)
    expected = 0.5 * (left[1] + SOLE_OFFSET_LEFT + right[1] + SOLE_OFFSET_RIGHT)
    assert np.allclose(origin, expected)

    # Camera 0.2 m above the body origin sits 0.2 + 0.2805 m above the ground.
    _, cam = camera_in_footprint((rot, origin), (eye, np.array([0.0, 0.0, 0.2])))
    assert math.isclose(cam[2], 0.2 + 0.25 + 0.0305)


def test_lifted_swing_foot_does_not_raise_ground():
    eye = np.eye(3)
    left = (eye, np.array([0.0, 0.035, -0.25]))
    right = (eye, np.array([0.03, -0.035, -0.21]))  # swing foot 4 cm up
    _, origin = footprint_from_feet(left, right)
    assert math.isclose(origin[2], -0.25 - 0.0305)


def test_body_tilt_is_removed_by_footprint():
    # Body pitched 10 deg forward: in body coordinates the feet are rotated back.
    a = math.radians(10.0)
    pitch = np.array([[math.cos(a), 0.0, -math.sin(a)], [0.0, 1.0, 0.0],
                      [math.sin(a), 0.0, math.cos(a)]])
    left = (pitch, pitch @ np.array([0.0, 0.035, -0.25]))
    right = (pitch, pitch @ np.array([0.0, -0.035, -0.25]))
    rot, _ = footprint_from_feet(left, right)
    # Footprint z is the soles' normal, i.e. gravity-up, not the body's z.
    assert np.allclose(rot[:, 2], pitch[:, 2])
