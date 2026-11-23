"""Unit tests for the ROS-free core modules. Run: python3 -m pytest test/test_core.py"""

import math

import numpy as np

from GoNasional.core.ball_filter import BallFilter
from GoNasional.core.fall_detector import FallDetector, FallState
from GoNasional.core.field_model import FieldModel
from GoNasional.core.head_controller import HeadController
from GoNasional.core.mcl import MonteCarloLocalization
from GoNasional.core.sequencer import Sequencer, Step

G = 9.81


def feed(detector, acc, start, duration, rate=100.0):
    t = start
    while t < start + duration:
        detector.update(*acc, t)
        t += 1.0 / rate
    return t


def test_fall_detector_directions_and_debounce():
    d = FallDetector()
    t = feed(d, (0.0, 0.0, G), 0.0, 1.0)
    assert d.state is FallState.UPRIGHT

    # A short knock must not count as a fall.
    t = feed(d, (-G, 0.0, 0.0), t, 0.03)
    t = feed(d, (0.0, 0.0, G), t, 0.5)
    assert d.state is FallState.UPRIGHT

    t = feed(d, (-G, 0.0, 0.5), t, 1.0)     # legacy: acc_x < -9 is "front"
    assert d.state is FallState.FALLEN_FRONT
    t = feed(d, (0.0, 0.0, G), t, 1.0)
    assert d.state is FallState.UPRIGHT
    t = feed(d, (G, 0.0, 0.5), t, 1.0)
    assert d.state is FallState.FALLEN_BACK


def test_fall_detector_catches_side_fall():
    d = FallDetector()
    t = feed(d, (0.0, 0.0, G), 0.0, 0.5)
    feed(d, (0.3, G, 0.2), t, 1.0)          # legacy acc_x check never fires here
    assert d.fallen


def test_sequencer_runs_steps_without_blocking():
    log = []
    done = {'page': False}
    timeouts = []
    seq = Sequencer(on_timeout=lambda s, n: timeouts.append(n))
    seq.start('kick', [
        Step('a', lambda: log.append('a'), min_time=0.5),
        Step('page', lambda: log.append('page'), done=lambda: done['page'],
             min_time=0.2, timeout=3.0),
        Step('c', lambda: log.append('c'), min_time=0.1),
    ], on_finish=lambda: log.append('finished'))

    t = 0.0
    while t < 1.0:
        seq.tick(t)
        t += 0.02
    assert log == ['a', 'page'] and seq.step_name == 'page'
    done['page'] = True
    while t < 2.0:
        seq.tick(t)
        t += 0.02
    assert log == ['a', 'page', 'c', 'finished'] and not seq.active and not timeouts


def test_sequencer_timeout_and_restart_from_on_enter():
    timeouts = []
    seq = Sequencer(on_timeout=lambda s, n: timeouts.append(n))
    attempts = []

    def start():
        attempts.append(1)
        seq.start('getup', [
            Step('page', done=lambda: False, timeout=0.3),
            Step('verify', verify, min_time=0.1),
        ])

    def verify():
        if len(attempts) < 3:
            start()

    start()
    t = 0.0
    while t < 5.0 and seq.active:
        seq.tick(t)
        t += 0.02
    assert len(attempts) == 3 and timeouts == ['page'] * 3 and not seq.active


def test_head_tracks_incrementally_then_scans_when_lost():
    h = HeadController()
    h.reset(0.0, -0.3)
    true_pan = 0.6
    t = 0.0
    for _ in range(40):
        t += 0.066
        h.observe(true_pan - h.pan, 0.0, t)
        h.update(t)
    assert abs(h.pan - true_pan) < 0.02 and not h.scanning

    pans = []
    for _ in range(300):
        t += 0.04
        pans.append(h.update(t)[0])
    assert h.scanning
    assert max(pans) > 1.1 and min(pans) < -1.1     # sweeps the whole range


def test_mcl_converges_from_wrong_start_and_tracks():
    field = FieldModel()
    mcl = MonteCarloLocalization(field, n_particles=400, seed=3)
    rng = np.random.default_rng(5)
    truth = np.array([0.5, 1.0, 0.0])
    # Heading is known from the IMU, position is off by more than a metre.
    mcl.init_pose(-0.5, 0.0, 0.0, std_xy=1.0, std_theta=0.1)

    def observe():
        obs = []
        for type_id, points in field.landmarks.items():
            for lx, ly in points:
                r = math.hypot(lx - truth[0], ly - truth[1])
                b = math.atan2(ly - truth[1], lx - truth[0]) - truth[2]
                b = (b + math.pi) % (2 * math.pi) - math.pi
                if r < 4.5 and abs(b) < 1.2:
                    obs.append((type_id, r * (1 + rng.normal(0, 0.08)), b + rng.normal(0, 0.03)))
        return obs

    for step in range(300):
        dx, dth = 0.006, 0.001
        truth[0] += dx * math.cos(truth[2])
        truth[1] += dx * math.sin(truth[2])
        truth[2] += dth
        mcl.predict(dx * 1.15, 0.0, dth)       # odometry over-reads by 15 %
        if step % 2 == 0:
            mcl.update_landmarks(observe())

    x, y, theta, std_xy, _ = mcl.estimate()
    assert math.hypot(x - truth[0], y - truth[1]) < 0.35
    assert abs(theta - truth[2]) < 0.15
    assert std_xy < 0.45


def test_ball_filter_velocity_outlier_and_teleport():
    f = BallFilter()
    t = 0.0
    for i in range(30):
        t = i * 0.066
        f.predict(0.066)
        f.update(1.0 + 0.5 * t, 0.0, 1.0, t)
    assert abs(f.velocity[0] - 0.5) < 0.2 and f.visible(t)

    before = f.position
    f.update(4.0, 3.0, 1.0, t)                 # single false detection
    assert math.hypot(f.position[0] - before[0], f.position[1] - before[1]) < 0.05

    for _ in range(3):                         # ball really moved
        f.update(-2.0, -1.0, 1.0, t)
    assert math.hypot(f.position[0] + 2.0, f.position[1] + 1.0) < 0.05

    assert f.known(t + 5.0) and not f.visible(t + 5.0) and not f.known(t + 7.0)


def test_mcl_recovers_after_robot_is_moved():
    field = FieldModel()
    mcl = MonteCarloLocalization(field, n_particles=400, seed=2)
    rng = np.random.default_rng(4)
    mcl.init_pose(-2.0, 1.0, 0.0, std_xy=0.2, std_theta=0.05)
    truth = (-2.0, 1.0, 0.0)

    for step in range(350):
        if step == 80:
            truth = (1.5, -1.5, 0.0)            # picked up and put down elsewhere
        obs = []
        for type_id, points in field.landmarks.items():
            for lx, ly in points:
                r = math.hypot(lx - truth[0], ly - truth[1])
                b = math.atan2(ly - truth[1], lx - truth[0]) - truth[2]
                if r < 4.5:
                    obs.append((type_id, r * (1 + rng.normal(0, 0.05)), b + rng.normal(0, 0.03)))
        mcl.predict(0.0, 0.0, 0.0)
        mcl.update_landmarks(obs)

    x, y, _, std_xy, _ = mcl.estimate()
    assert math.hypot(x - truth[0], y - truth[1]) < 0.4 and std_xy < 0.5


def test_mcl_reentry_hypotheses_pick_the_right_touchline():
    field = FieldModel()
    mcl = MonteCarloLocalization(field, n_particles=400, seed=6)
    rng = np.random.default_rng(7)
    truth = np.array([-2.0, 2.9, -math.pi / 2])      # placed on the left touchline
    hw = field.width / 2.0
    poses = [(x, side * hw, truth[2]) for x in (-0.9, -1.8, -2.7, -3.6) for side in (1.0, -1.0)]
    mcl.init_hypotheses(poses, std_xy=0.3, std_theta=0.15)

    for _ in range(150):
        truth[1] -= 0.004                            # walks into the field
        mcl.predict(0.004, 0.0, 0.0)
        obs = []
        for type_id, points in field.landmarks.items():
            for lx, ly in points:
                r = math.hypot(lx - truth[0], ly - truth[1])
                b = math.atan2(ly - truth[1], lx - truth[0]) - truth[2]
                b = (b + math.pi) % (2 * math.pi) - math.pi
                if r < 4.5 and abs(b) < 1.3 and rng.random() > 0.3:
                    obs.append((type_id, r * (1 + rng.normal(0, 0.08)), b + rng.normal(0, 0.03)))
        if obs:
            mcl.update_landmarks(obs)

    x, y, theta, std_xy, _ = mcl.estimate()
    assert math.hypot(x - truth[0], y - truth[1]) < 0.4 and std_xy < 0.45
    assert abs(theta - truth[2]) < 0.2


def _observe(field, truth, rng, max_range=4.5):
    obs = []
    for type_id, points in field.landmarks.items():
        for lx, ly in points:
            r = math.hypot(lx - truth[0], ly - truth[1])
            b = math.atan2(ly - truth[1], lx - truth[0]) - truth[2]
            if r < max_range:
                obs.append((type_id, r * (1 + rng.normal(0, 0.05)), b + rng.normal(0, 0.03)))
    return obs


def test_mcl_gate_drops_false_detections_while_tracking():
    from GoNasional.core.mcl import TRACKING
    field = FieldModel()
    mcl = MonteCarloLocalization(field, n_particles=300, seed=3)
    rng = np.random.default_rng(5)
    truth = (-1.0, 0.5, 0.2)
    mcl.init_pose(*truth, std_xy=0.2, std_theta=0.05)
    for _ in range(30):
        mcl.predict(0.0, 0.0, 0.0)
        mcl.update_landmarks(_observe(field, truth, rng))
    assert mcl.state == TRACKING
    for _ in range(30):
        # A "goalpost" right in front of the robot where no post exists.
        mcl.predict(0.0, 0.0, 0.0)
        mcl.update_landmarks(_observe(field, truth, rng) + [(1, 0.8, 0.0)])
    x, y, _, std_xy, _ = mcl.estimate()
    assert mcl.rejected >= 30
    assert mcl.state == TRACKING
    assert math.hypot(x - truth[0], y - truth[1]) < 0.2


def test_mcl_declares_lost_and_recovers_when_moved():
    from GoNasional.core.mcl import LOST, TRACKING
    field = FieldModel()
    mcl = MonteCarloLocalization(field, n_particles=400, seed=8)
    rng = np.random.default_rng(9)
    truth = (-2.0, 1.0, 0.0)
    mcl.init_pose(*truth, std_xy=0.2, std_theta=0.05)
    states = []
    for step in range(300):
        if step == 40:
            truth = (1.5, -1.0, 0.0)
        mcl.predict(0.0, 0.0, 0.0)
        mcl.update_landmarks(_observe(field, truth, rng))
        states.append(mcl.state)
    assert LOST in states[40:]
    assert states[-1] == TRACKING
    x, y, _, _, _ = mcl.estimate()
    assert math.hypot(x - truth[0], y - truth[1]) < 0.3


def test_mcl_two_posts_need_two_different_posts():
    # Two posts seen 1.6 m apart: one particle where both match the same post
    # must score lower than the one at the true pose.
    field = FieldModel(goal_width=1.6)
    mcl = MonteCarloLocalization(field, n_particles=2, seed=1)
    truth = np.array([3.0, 0.0, 0.0])
    obs = []
    for lx, ly in field.landmarks[1][:2]:
        obs.append((math.hypot(lx - truth[0], ly - truth[1]),
                    math.atan2(ly - truth[1], lx - truth[0])))
    mcl.particles = np.array([truth, [3.0, 0.8, 0.0]])
    lik = mcl._type_likelihood(field.landmarks[1], obs)
    assert lik[0] > 10 * lik[1]


def test_head_scan_holds_still_at_stops():
    h = HeadController(scan_step=0.6, scan_dwell=0.5)
    h.reset(0.0, -0.25)
    h.set_mode('scan')
    t, prev, still = 0.0, None, 0
    pans = []
    for _ in range(500):
        t += 0.02
        pan, tilt = h.update(t)
        pans.append(pan)
        if prev is not None and abs(pan - prev[0]) < 1e-9 and abs(tilt - prev[1]) < 1e-9:
            still += 1
        prev = (pan, tilt)
    assert still > 250                           # most of the time the camera is steady
    assert max(pans) > 1.1 and min(pans) < -1.1  # and the whole width is still covered
