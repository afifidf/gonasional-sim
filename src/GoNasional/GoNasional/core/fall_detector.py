"""Fall detection from the accelerometer gravity vector. No ROS imports."""

import math
from enum import Enum


class FallState(Enum):
    UPRIGHT = 'upright'
    FALLEN_FRONT = 'fallen_front'
    FALLEN_BACK = 'fallen_back'


class FallDetector:
    """Debounced tilt check on a low-pass filtered accelerometer reading.

    The legacy check compared acc_x with +-9 m/s^2, which misses sideways falls
    and anything that ends up less than ~66 deg from vertical. Here the tilt is
    the angle between measured gravity and the body z axis, so every direction
    counts, and the front/back decision only picks which get-up page to play.

    forward_sign: sign of acc_x when the robot lies on its front. The AROC
    OpenCR firmware reports negative acc_x in that case (legacy: acc_x < -9
    played the "fallen front" page), hence the default of -1.
    """

    def __init__(self, fall_angle_deg=55.0, recover_angle_deg=30.0,
                 debounce_s=0.15, alpha=0.2, forward_sign=-1.0):
        self.fall_angle = math.radians(fall_angle_deg)
        self.recover_angle = math.radians(recover_angle_deg)
        self.debounce_s = debounce_s
        self.alpha = alpha
        self.forward_sign = forward_sign
        self._acc = None
        self._over_since = None
        self.state = FallState.UPRIGHT
        self.tilt = 0.0

    def reset(self):
        self._acc = None
        self._over_since = None
        self.state = FallState.UPRIGHT
        self.tilt = 0.0

    def update(self, ax, ay, az, now):
        """Feed one accelerometer sample (m/s^2) taken at time `now` (s)."""
        if self._acc is None:
            self._acc = [ax, ay, az]
        else:
            a = self.alpha
            self._acc = [a * new + (1.0 - a) * old
                         for new, old in zip((ax, ay, az), self._acc)]

        fx, fy, fz = self._acc
        norm = math.sqrt(fx * fx + fy * fy + fz * fz)
        if norm < 1e-3:
            # Free fall or a dead sensor: no direction information, keep state.
            return self.state
        self.tilt = math.acos(max(-1.0, min(1.0, fz / norm)))

        if self.state is FallState.UPRIGHT:
            if self.tilt > self.fall_angle:
                if self._over_since is None:
                    self._over_since = now
                if now - self._over_since >= self.debounce_s:
                    front = self.forward_sign * fx >= 0.0
                    self.state = (FallState.FALLEN_FRONT if front
                                  else FallState.FALLEN_BACK)
            else:
                self._over_since = None
        elif self.tilt < self.recover_angle:
            self.state = FallState.UPRIGHT
            self._over_since = None
        else:
            # Still down: keep the direction current, the robot may roll over.
            front = self.forward_sign * fx >= 0.0
            self.state = (FallState.FALLEN_FRONT if front
                          else FallState.FALLEN_BACK)
        return self.state

    @property
    def fallen(self):
        return self.state is not FallState.UPRIGHT
