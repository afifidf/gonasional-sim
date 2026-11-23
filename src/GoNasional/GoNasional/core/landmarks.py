"""Field landmarks from detector boxes, through inverse perspective mapping.

Pure Python + numpy, no ROS. The detector (main_vision) gives boxes in the
450x337 frame; each class maps to a landmark type of field_model and to the
pixel that lies on the floor:

  tiang_gawang    bottom centre of the box = foot of the post
  titik_penalti,  centre of the box (flat marks lie on the floor)
  titik_tengah,
  sudut_lapangan

Boxes cut by the image border are dropped: their reference pixel is not on
the object any more, which biases range (post foot below the frame) or
bearing (half a mark at the side). Very far landmarks are dropped too,
because one pixel of error there is tens of centimetres on the floor.
"""

import math
from dataclasses import dataclass

from .field_model import CENTER_MARK, FIELD_CORNER, GOALPOST, PENALTY_MARK
from .ipm import pixel_to_ground

LABEL_TYPES = {
    'tiang_gawang': GOALPOST,
    'titik_penalti': PENALTY_MARK,
    'titik_tengah': CENTER_MARK,
    'sudut_lapangan': FIELD_CORNER,
}


@dataclass
class LandmarkConfig:
    min_confidence: float = 0.5
    edge_margin_px: float = 3.0
    max_range_post: float = 5.5
    max_range_mark: float = 4.0
    min_range: float = 0.25
    min_post_height_px: float = 8.0     # thinner/shorter boxes are usually net or noise


@dataclass
class Box:
    label: str
    confidence: float
    x1: float
    y1: float
    x2: float
    y2: float


def ground_pixel(box):
    """Pixel of the box that lies on the floor, for a landmark label."""
    if LABEL_TYPES.get(box.label) == GOALPOST:
        return (box.x1 + box.x2) / 2.0, box.y2
    return (box.x1 + box.x2) / 2.0, (box.y1 + box.y2) / 2.0


def extract_landmarks(boxes, model, camera_fp, width, height, config=LandmarkConfig()):
    """[(type, range_m, bearing_rad, confidence)] in base_footprint.

    boxes: iterable of Box (non-landmark labels such as bola are ignored).
    model: ipm.CameraModel for the frame the boxes refer to.
    camera_fp: camera pose in base_footprint (rotation, position).
    """
    out = []
    m = config.edge_margin_px
    for box in boxes:
        type_id = LABEL_TYPES.get(box.label)
        if type_id is None or box.confidence < config.min_confidence:
            continue
        if box.x1 < m or box.x2 > width - m or box.y2 > height - m:
            continue   # cut by the left, right or bottom border
        if type_id != GOALPOST and box.y1 < m:
            continue   # a flat mark cut at the top: centre is off
        if type_id == GOALPOST and box.y2 - box.y1 < config.min_post_height_px:
            continue
        u, v = ground_pixel(box)
        point = pixel_to_ground(model, u, v, camera_fp, plane_z=0.0,
                                max_range=config.max_range_post if type_id == GOALPOST
                                else config.max_range_mark)
        if point is None:
            continue
        rng = math.hypot(*point)
        if rng < config.min_range:
            continue
        out.append((type_id, rng, math.atan2(point[1], point[0]), float(box.confidence)))
    return out


class MotionGate:
    """Rejects landmark frames taken while the head or body turns quickly.

    During a fast pan the image is blurred and the joint angles used for IPM
    no longer match the instant the frame was exposed, so a post can land a
    metre off. Rates come from successive joint / gyro readings.
    """

    def __init__(self, max_head_rate=0.6, max_body_rate=0.8, hold=0.15):
        self.max_head_rate = max_head_rate
        self.max_body_rate = max_body_rate
        self.hold = hold
        self._last_head = None
        self._head_rate = 0.0
        self._body_rate = 0.0
        self._fast_until = -math.inf

    def on_head(self, pan, tilt, t, window=0.08):
        # Rate over a short window: simulated and real servos report positions
        # in small jumps, so sample-to-sample rates swing wildly.
        if self._last_head is None or t < self._last_head[2]:
            self._last_head = (pan, tilt, t)
            return
        p0, t0, time0 = self._last_head
        dt = t - time0
        if dt >= window:
            self._head_rate = math.hypot(pan - p0, tilt - t0) / dt
            self._last_head = (pan, tilt, t)
            self._check(t)

    def on_gyro(self, yaw_rate, t):
        self._body_rate = abs(yaw_rate)
        self._check(t)

    def _check(self, t):
        if self._head_rate > self.max_head_rate or self._body_rate > self.max_body_rate:
            self._fast_until = t + self.hold

    def steady(self, t):
        return t >= self._fast_until


@dataclass
class BallConfig:
    min_confidence: float = 0.5
    radius: float = 0.07
    size_ratio_max: float = 1.8      # box width vs width expected at the IPM range
    min_forward: float = 0.09        # m; closer than this the "ball" is inside the feet
    max_range: float = 5.0
    edge_margin_px: float = 3.0


def select_ball(boxes, model, camera_fp, width, height, config=BallConfig()):
    """Best plausible ball: (u, v, (x, y), confidence) or None.

    Detections are checked against geometry before trusting the confidence:
    the box must be about as wide as a ball would look at the range IPM gives
    (a dark robot shin or foot detected as a ball is far too big or too small),
    and the ball cannot lie inside the robot's own feet.
    """
    from .ipm import pixel_to_ground
    best = None
    m = config.edge_margin_px
    for box in boxes:
        if box.label != 'bola' or box.confidence < config.min_confidence:
            continue
        u, v = (box.x1 + box.x2) / 2.0, (box.y1 + box.y2) / 2.0
        point = pixel_to_ground(model, u, v, camera_fp, plane_z=config.radius,
                                max_range=config.max_range)
        if point is None:
            continue
        rng = math.hypot(*point)
        if point[0] < config.min_forward and abs(point[1]) < 0.15:
            continue
        cut = box.x1 < m or box.x2 > width - m or box.y1 < m or box.y2 > height - m
        # Diameter in pixels of a sphere of `radius` seen from the camera; the
        # box height is rescaled because the frame is squashed (fx != fy).
        cam_dist = max(0.05, math.hypot(rng, camera_fp[1][2] - config.radius))
        expected = 2.0 * model.fx * config.radius / cam_dist
        size = max(box.x2 - box.x1, (box.y2 - box.y1) * model.fx / model.fy) / expected
        if size > config.size_ratio_max:
            continue      # too big, even when cut by the border
        if not cut and size < 1.0 / config.size_ratio_max:
            continue      # too small (a cut box may legitimately be small)
        if best is None or box.confidence > best[3]:
            best = (u, v, point, float(box.confidence))
    return best
