"""Monte Carlo localization on the soccer field. numpy only, no ROS imports."""

import itertools
import math

import numpy as np

from .field_model import FieldModel


TRACKING, UNCERTAIN, LOST = 0, 1, 2   # same values as aroc_interfaces/WorldModel


class MonteCarloLocalization:
    """Particle filter over (x, y, theta) in the field frame.

    Motion comes from walking odometry (heading change preferably from the
    IMU), corrections from range/bearing observations of field landmarks.
    Landmarks of one type are indistinguishable. Each particle scores the
    observations of one type against the best one-to-one assignment to the
    map landmarks of that type (two posts seen in one frame cannot both be the
    same post), with an outlier floor per observation.

    While tracking, observations far outside what the current estimate
    predicts (chi-square gate on range/bearing) are dropped as false
    detections. If every observation keeps failing the gate, the estimate is
    the thing that is wrong: the filter declares itself LOST, stops gating and
    injects random particles until observations fit again.
    """

    def __init__(self, field: FieldModel, n_particles=300, seed=None,
                 odom_noise_xy=0.25, odom_noise_xy_min=0.004,
                 odom_noise_theta=0.10, odom_noise_theta_min=0.004,
                 range_sigma_base=0.10, range_sigma_rel=0.15,
                 bearing_sigma=0.10, outlier_weight=0.05,
                 max_random_fraction=0.10, alpha_slow=0.02, alpha_fast=0.3,
                 injection_threshold=0.4, gate_chi2=13.8, tracking_std_xy=0.45,
                 lost_after=5):
        self.field = field
        self.n = int(n_particles)
        self.rng = np.random.default_rng(seed)
        self.odom_noise_xy = odom_noise_xy
        self.odom_noise_xy_min = odom_noise_xy_min
        self.odom_noise_theta = odom_noise_theta
        self.odom_noise_theta_min = odom_noise_theta_min
        self.range_sigma_base = range_sigma_base
        self.range_sigma_rel = range_sigma_rel
        self.bearing_sigma = bearing_sigma
        self.outlier_weight = outlier_weight
        self.max_random_fraction = max_random_fraction
        self.alpha_slow = alpha_slow
        self.alpha_fast = alpha_fast
        self.injection_threshold = injection_threshold
        self.gate_chi2 = gate_chi2            # 99.9 % of chi-square with 2 dof
        self.tracking_std_xy = tracking_std_xy
        self.lost_after = lost_after
        self.state = UNCERTAIN
        self._gated_frames = 0
        self.rejected = 0
        self._w_slow = 0.0
        self._w_fast = 0.0
        self.particles = np.zeros((self.n, 3))
        self.weights = np.full(self.n, 1.0 / self.n)
        self.init_uniform()

    # ---------------------------------------------------------------- init

    def init_pose(self, x, y, theta, std_xy=0.3, std_theta=0.2):
        self.particles[:, 0] = self.rng.normal(x, std_xy, self.n)
        self.particles[:, 1] = self.rng.normal(y, std_xy, self.n)
        self.particles[:, 2] = self.rng.normal(theta, std_theta, self.n)
        self.weights.fill(1.0 / self.n)
        self._after_reset()

    def init_hypotheses(self, poses, std_xy=0.3, std_theta=0.2):
        """Spread particles evenly over several candidate poses.

        Used after a penalty, when the robot re-enters from one of the two
        touchlines in its own half but does not know which.
        """
        poses = np.asarray(poses, dtype=float)
        idx = self.rng.integers(0, len(poses), self.n)
        self.particles = poses[idx] + self.rng.normal(
            0.0, [std_xy, std_xy, std_theta], (self.n, 3))
        self.weights.fill(1.0 / self.n)
        self._after_reset()

    def _after_reset(self):
        self.state = UNCERTAIN
        self._gated_frames = 0
        # Keep the long-term fit as the yardstick, so a reset onto wrong
        # hypotheses is noticed instead of becoming the new normal.
        self._w_fast = self._w_slow

    def init_uniform(self, theta=None, std_theta=0.2):
        hl, hw = self.field.length / 2.0, self.field.width / 2.0
        self.particles[:, 0] = self.rng.uniform(-hl, hl, self.n)
        self.particles[:, 1] = self.rng.uniform(-hw, hw, self.n)
        if theta is None:
            self.particles[:, 2] = self.rng.uniform(-math.pi, math.pi, self.n)
        else:
            self.particles[:, 2] = self.rng.normal(theta, std_theta, self.n)
        self.weights.fill(1.0 / self.n)
        self._after_reset()

    # -------------------------------------------------------------- predict

    def predict(self, dx, dy, dtheta):
        """Apply a robot-frame displacement (m, m, rad) with noise."""
        dist = math.hypot(dx, dy)
        sig_xy = self.odom_noise_xy * dist + self.odom_noise_xy_min
        sig_th = self.odom_noise_theta * abs(dtheta) + self.odom_noise_theta_min
        ndx = dx + self.rng.normal(0.0, sig_xy, self.n)
        ndy = dy + self.rng.normal(0.0, sig_xy, self.n)
        th = self.particles[:, 2]
        c, s = np.cos(th), np.sin(th)
        self.particles[:, 0] += c * ndx - s * ndy
        self.particles[:, 1] += s * ndx + c * ndy
        self.particles[:, 2] = th + dtheta + self.rng.normal(0.0, sig_th, self.n)

    # --------------------------------------------------------------- update

    def _gate(self, observations):
        """Drop observations the current estimate cannot explain (tracking only)."""
        if self.state != TRACKING:
            return list(observations)
        x, y, theta, std_xy, std_theta = self.estimate()
        kept = []
        for type_id, rng_m, bearing in observations:
            candidates = self.field.landmarks[int(type_id)]
            exp_r = np.hypot(candidates[:, 0] - x, candidates[:, 1] - y)
            exp_b = np.arctan2(candidates[:, 1] - y, candidates[:, 0] - x) - theta
            err_b = (exp_b - bearing + np.pi) % (2.0 * np.pi) - np.pi
            var_r = (self.range_sigma_base + self.range_sigma_rel * rng_m) ** 2 + std_xy ** 2
            var_b = (self.bearing_sigma ** 2 + std_theta ** 2
                     + (std_xy / np.maximum(exp_r, 0.3)) ** 2)
            d2 = (exp_r - rng_m) ** 2 / var_r + err_b ** 2 / var_b
            if d2.min() <= self.gate_chi2:
                kept.append((type_id, rng_m, bearing))
            else:
                self.rejected += 1
        if kept:
            self._gated_frames = 0
        else:
            self._gated_frames += 1
            if self._gated_frames >= self.lost_after:
                self.state = LOST
                self._w_fast = 0.0    # start injecting at once instead of after a slow decay
        return kept

    def _type_likelihood(self, candidates, obs):
        """Per-particle likelihood of all observations of one landmark type."""
        px, py, pth = self.particles.T
        dx = candidates[:, 0][None, :] - px[:, None]
        dy = candidates[:, 1][None, :] - py[:, None]
        exp_range = np.hypot(dx, dy)
        exp_bearing = np.arctan2(dy, dx) - pth[:, None]
        scores = []                       # k arrays of shape (n, m)
        for rng_m, bearing in obs:
            err_b = (exp_bearing - bearing + np.pi) % (2.0 * np.pi) - np.pi
            sig_r = self.range_sigma_base + self.range_sigma_rel * rng_m
            scores.append(self.outlier_weight + np.exp(
                -0.5 * ((exp_range - rng_m) / sig_r) ** 2 - 0.5 * (err_b / self.bearing_sigma) ** 2))
        k, m = len(obs), len(candidates)
        if k == 1 or k > m:
            # More sightings than landmarks: some are false, score each alone.
            return np.prod([sc.max(axis=1) for sc in scores], axis=0)
        best = np.zeros(self.n)
        for perm in itertools.permutations(range(m), k):   # at most 4*3*2*1 = 24
            prod = np.ones(self.n)
            for i, j in enumerate(perm):
                prod *= scores[i][:, j]
            np.maximum(best, prod, out=best)
        return best

    def update_landmarks(self, observations):
        """observations: iterable of (type_id, range_m, bearing_rad)."""
        valid = [(int(t), float(r), float(b)) for t, r, b in observations
                 if int(t) in self.field.landmarks and r > 0.0]
        if not valid:
            return False
        observations = self._gate(valid)
        if not observations:
            return False
        px, py = self.particles[:, 0], self.particles[:, 1]
        likelihood = np.ones(self.n)
        by_type = {}
        for type_id, rng_m, bearing in observations:
            by_type.setdefault(type_id, []).append((rng_m, bearing))
        for type_id, obs in by_type.items():
            likelihood *= self._type_likelihood(self.field.landmarks[type_id], obs)
        n_obs = len(observations)

        # Short- and long-term averages of how well observations fit. When the
        # short-term one collapses the robot has probably been moved.
        fit = float(np.sum(self.weights * likelihood ** (1.0 / n_obs)))
        if self._w_slow == 0.0:
            self._w_slow = self._w_fast = fit
        else:
            self._w_slow += self.alpha_slow * (fit - self._w_slow)
            self._w_fast += self.alpha_fast * (fit - self._w_fast)
        self.weights *= likelihood

        # Particles that wandered off the carpet are almost certainly wrong.
        hl, hw = self.field.length / 2.0 + 0.7, self.field.width / 2.0 + 0.7
        outside = (np.abs(px) > hl) | (np.abs(py) > hw)
        self.weights[outside] *= 0.01

        total = self.weights.sum()
        if total < 1e-300 or not np.isfinite(total):
            self.weights.fill(1.0 / self.n)
        else:
            self.weights /= total
        self._resample_if_needed()
        self._update_state()
        return True

    def _update_state(self):
        std_xy = self.estimate()[3]
        if self.state == LOST and self._fit_drop() >= self.injection_threshold:
            return     # observations still fit worse than usual: keep searching
        if std_xy < self.tracking_std_xy:
            self.state = TRACKING
        else:
            self.state = UNCERTAIN

    def _fit_drop(self):
        """How much worse recent observations fit than the long-term average (0..1)."""
        if self._w_slow <= 0.0:
            return 0.0
        return 1.0 - self._w_fast / self._w_slow

    def _random_fraction(self):
        """Share of particles to re-draw at random; 0 while observations fit."""
        drop = self._fit_drop()
        if self.state == LOST:
            return self.max_random_fraction if drop >= self.injection_threshold else 0.0
        if drop < self.injection_threshold:
            return 0.0
        return min(self.max_random_fraction, drop)

    def _resample_if_needed(self):
        n_eff = 1.0 / np.sum(self.weights ** 2)
        fraction = self._random_fraction()
        if n_eff > self.n / 2.0 and fraction == 0.0:
            return
        # Low-variance (systematic) resampling.
        positions = (self.rng.random() + np.arange(self.n)) / self.n
        cumulative = np.cumsum(self.weights)
        cumulative[-1] = 1.0
        self.particles = self.particles[np.searchsorted(cumulative, positions)]
        self.weights.fill(1.0 / self.n)

        # Random particles let the filter recover after a referee moves the
        # robot. They are only added while observations fit worse than usual,
        # so a well-localized filter is not polluted by them. Their heading
        # stays near the current estimate because the IMU already pins it down.
        n_random = int(fraction * self.n)
        if n_random > 0:
            hl, hw = self.field.length / 2.0, self.field.width / 2.0
            theta = math.atan2(np.sin(self.particles[:, 2]).mean(),
                               np.cos(self.particles[:, 2]).mean())
            idx = self.rng.choice(self.n, n_random, replace=False)
            self.particles[idx, 0] = self.rng.uniform(-hl, hl, n_random)
            self.particles[idx, 1] = self.rng.uniform(-hw, hw, n_random)
            self.particles[idx, 2] = self.rng.normal(theta, 0.15, n_random)

    # ------------------------------------------------------------- estimate

    def estimate(self):
        """Weighted mean pose and spread: (x, y, theta, std_xy, std_theta)."""
        w = self.weights
        x = float(np.sum(w * self.particles[:, 0]))
        y = float(np.sum(w * self.particles[:, 1]))
        s = float(np.sum(w * np.sin(self.particles[:, 2])))
        c = float(np.sum(w * np.cos(self.particles[:, 2])))
        theta = math.atan2(s, c)
        var_xy = float(np.sum(w * ((self.particles[:, 0] - x) ** 2
                                   + (self.particles[:, 1] - y) ** 2)))
        resultant = min(1.0, math.hypot(s, c))
        std_theta = math.sqrt(max(0.0, -2.0 * math.log(max(resultant, 1e-9))))
        return x, y, theta, math.sqrt(var_xy), std_theta
