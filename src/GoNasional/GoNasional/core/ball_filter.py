"""Kalman filter for the ball in the field frame. numpy only, no ROS imports."""

import math

import numpy as np


class BallFilter:
    """Constant-velocity Kalman filter with rolling friction.

    State [x, y, vx, vy] in the field frame. Keeping the ball in the field
    frame means the estimate stays put while the robot walks or turns its head
    away, so the behavior can keep approaching a ball it no longer sees.
    """

    def __init__(self, friction=0.6, accel_noise=1.5, meas_sigma_base=0.04,
                 meas_sigma_rel=0.08, gate_sigma=4.0, visible_timeout=0.5,
                 forget_timeout=6.0):
        self.friction = friction
        self.accel_noise = accel_noise
        self.meas_sigma_base = meas_sigma_base
        self.meas_sigma_rel = meas_sigma_rel
        self.gate_sigma = gate_sigma
        self.visible_timeout = visible_timeout
        self.forget_timeout = forget_timeout
        self.x = np.zeros(4)
        self.P = np.eye(4)
        self.initialized = False
        self.last_seen = None
        self._rejects = 0

    def reset(self):
        self.initialized = False
        self.last_seen = None
        self._rejects = 0

    def predict(self, dt):
        if not self.initialized or dt <= 0.0:
            return
        decay = math.exp(-self.friction * dt)
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        F[2, 2] = F[3, 3] = decay
        q = self.accel_noise ** 2
        Q = np.zeros((4, 4))
        Q[0, 0] = Q[1, 1] = 0.25 * dt ** 4 * q
        Q[0, 2] = Q[2, 0] = Q[1, 3] = Q[3, 1] = 0.5 * dt ** 3 * q
        Q[2, 2] = Q[3, 3] = dt ** 2 * q
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, x, y, distance, now):
        """Fuse one observation (field frame, m) seen from `distance` m away."""
        sigma = self.meas_sigma_base + self.meas_sigma_rel * distance
        R = np.eye(2) * sigma ** 2
        z = np.array([x, y])
        if not self.initialized:
            self._seed(z, sigma)
            self.last_seen = now
            return

        H = np.zeros((2, 4))
        H[0, 0] = H[1, 1] = 1.0
        innovation = z - H @ self.x
        S = H @ self.P @ H.T + R
        S_inv = np.linalg.inv(S)
        mahalanobis = float(innovation @ S_inv @ innovation)
        if mahalanobis > self.gate_sigma ** 2:
            # One wild reading is a false detection; several in a row mean the
            # ball really is somewhere else (kicked, or replaced by a referee).
            self._rejects += 1
            if self._rejects >= 3:
                self._seed(z, sigma)
                self.last_seen = now
            return
        self._rejects = 0
        K = self.P @ H.T @ S_inv
        self.x = self.x + K @ innovation
        self.P = (np.eye(4) - K @ H) @ self.P
        self.last_seen = now

    def _seed(self, z, sigma):
        self.x = np.array([z[0], z[1], 0.0, 0.0])
        self.P = np.diag([sigma ** 2, sigma ** 2, 1.0, 1.0])
        self.initialized = True
        self._rejects = 0

    def age(self, now):
        return math.inf if self.last_seen is None else now - self.last_seen

    def known(self, now):
        return self.initialized and self.age(now) <= self.forget_timeout

    def visible(self, now):
        return self.initialized and self.age(now) <= self.visible_timeout

    @property
    def position(self):
        return float(self.x[0]), float(self.x[1])

    @property
    def velocity(self):
        return float(self.x[2]), float(self.x[3])

    @property
    def std(self):
        return float(math.sqrt(max(0.0, 0.5 * (self.P[0, 0] + self.P[1, 1]))))
