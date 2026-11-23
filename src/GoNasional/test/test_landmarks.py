"""Unit tests for core/landmarks.py. Run: python3 -m pytest test/test_landmarks.py"""

import math

import numpy as np

from GoNasional.core.field_model import FIELD_CORNER, GOALPOST, PENALTY_MARK
from GoNasional.core.ipm import CameraModel, ground_to_pixel
from GoNasional.core.landmarks import Box, MotionGate, extract_landmarks

OPTICAL = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
W, H = 450, 337
MODEL = CameraModel.from_params(W, H, 78.0, 640, 360)


def camera(height=0.45, tilt_down=math.radians(30.0)):
    c, s = math.cos(tilt_down), math.sin(tilt_down)
    pitch = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    return pitch @ OPTICAL, np.array([0.0, 0.0, height])


def post_box(x, y, cam, height=0.85, radius=0.05):
    u0, v0 = ground_to_pixel(MODEL, (x, y, 0.0), cam)
    u1, v1 = ground_to_pixel(MODEL, (x, y, height), cam)
    half = MODEL.fx * radius / math.hypot(x, y)
    return Box('tiang_gawang', 0.9, u0 - half, min(v0, v1), u0 + half, max(v0, v1))


def mark_box(label, x, y, cam, half=0.1):
    pts = [ground_to_pixel(MODEL, (x + dx, y + dy, 0.0), cam)
           for dx in (-half, half) for dy in (-half, half)]
    us, vs = zip(*pts)
    return Box(label, 0.8, min(us), min(vs), max(us), max(vs))


def test_post_foot_gives_range_and_bearing():
    cam = camera()
    out = extract_landmarks([post_box(2.5, 0.6, cam)], MODEL, cam, W, H)
    assert len(out) == 1
    type_id, rng, bearing, _ = out[0]
    assert type_id == GOALPOST
    assert math.isclose(rng, math.hypot(2.5, 0.6), rel_tol=0.01)
    assert math.isclose(bearing, math.atan2(0.6, 2.5), abs_tol=0.01)


def test_flat_marks_use_box_centre():
    cam = camera()
    boxes = [mark_box('titik_penalti', 1.5, -0.3, cam), mark_box('sudut_lapangan', 2.0, 0.5, cam)]
    out = sorted(extract_landmarks(boxes, MODEL, cam, W, H))
    assert [o[0] for o in out] == [PENALTY_MARK, FIELD_CORNER]
    for (_, rng, bearing, _), (x, y) in zip(out, [(1.5, -0.3), (2.0, 0.5)]):
        assert abs(rng - math.hypot(x, y)) < 0.05
        assert abs(bearing - math.atan2(y, x)) < 0.02


def test_cut_boxes_and_ball_are_ignored():
    cam = camera()
    cut_bottom = Box('tiang_gawang', 0.9, 200, 100, 210, H)          # foot below the frame
    cut_side = Box('titik_tengah', 0.9, 0, 200, 15, 210)
    ball = Box('bola', 0.9, 200, 200, 230, 230)
    weak = mark_box('titik_penalti', 1.5, 0.0, cam)
    weak.confidence = 0.2
    assert extract_landmarks([cut_bottom, cut_side, ball, weak], MODEL, cam, W, H) == []


def test_motion_gate_blocks_fast_head_then_releases():
    gate = MotionGate(max_head_rate=0.6, hold=0.2)
    gate.on_head(0.0, -0.5, 0.0)
    gate.on_head(0.2, -0.5, 0.10)        # 2 rad/s pan
    assert not gate.steady(0.15)
    gate.on_head(0.2, -0.5, 0.20)        # stopped
    assert gate.steady(0.31)


def ball_box(x, y, cam, radius=0.07, scale=1.0, conf=0.9):
    u, v = ground_to_pixel(MODEL, (x, y, radius), cam)
    dist = math.dist((x, y, radius), cam[1])
    half = scale * MODEL.fx * radius / dist
    return Box('bola', conf, u - half, v - half * MODEL.fy / MODEL.fx,
               u + half, v + half * MODEL.fy / MODEL.fx)


def test_select_ball_prefers_geometrically_valid_box():
    from GoNasional.core.landmarks import select_ball
    cam = camera(tilt_down=math.radians(45.0))
    real = ball_box(0.8, 0.1, cam, conf=0.6)
    shin = ball_box(1.0, -0.2, cam, scale=3.0, conf=0.95)       # far too big for its range
    u, v, (x, y), conf = select_ball([shin, real], MODEL, cam, W, H)
    assert conf == 0.6 and abs(x - 0.8) < 0.03 and abs(y - 0.1) < 0.03


def test_select_ball_rejects_own_feet():
    from GoNasional.core.landmarks import select_ball
    cam = camera(tilt_down=math.radians(70.0))
    foot = ball_box(0.05, 0.05, cam, conf=0.95)
    assert select_ball([foot], MODEL, cam, W, H) is None
