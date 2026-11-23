"""Whole decision chain on a kinematic robot, no ROS and no Webots.

A point robot with noisy odometry walks on the field, sees landmarks and the
ball through a limited field of view, and runs the same MCL, ball filter and
striker tree as the real nodes. The ball is pushed along the robot's heading
when a kick fires. Passing means: found the ball, got behind it, and kicked it
toward the opponent goal.
"""

import math

import numpy as np
import pytest

from GoNasional.core.ball_filter import BallFilter
from GoNasional.core.field_model import FieldModel
from GoNasional.core.geometry import to_field, to_robot, wrap
from GoNasional.core.mcl import MonteCarloLocalization
from GoNasional.core.striker import Striker

DT = 0.1
FOV = math.radians(65.0)


def run(robot_start, ball_start, seed=0, max_time=400.0):
    field = FieldModel()
    rng = np.random.default_rng(seed)
    striker = Striker(field)
    mcl = MonteCarloLocalization(field, n_particles=300, seed=seed)
    mcl.init_pose(*robot_start, std_xy=0.3, std_theta=0.1)
    ball_filter = BallFilter()

    robot = np.array(robot_start, dtype=float)
    ball = np.array(ball_start, dtype=float)
    head_pan = 0.0
    kicks = []
    t = 0.0
    cmd = (0.0, 0.0, 0.0)

    while t < max_time:
        t += DT
        # --- truth: the robot walks what was commanded, a bit short and noisy
        vx, vy, wz = cmd
        robot[0] += (vx * math.cos(robot[2]) - vy * math.sin(robot[2])) * DT * 0.9
        robot[1] += (vx * math.sin(robot[2]) + vy * math.cos(robot[2])) * DT * 0.9
        robot[2] = wrap(robot[2] + wz * DT + rng.normal(0, 0.002))

        # --- estimate
        mcl.predict(vx * DT, vy * DT, wz * DT)
        ball_filter.predict(DT)
        est = mcl.estimate()

        # --- perception: head follows the ball estimate, else sweeps
        rel = to_robot(tuple(robot), tuple(ball))
        dist, bearing = math.hypot(*rel), math.atan2(rel[1], rel[0])
        if ball_filter.visible(t):
            head_pan = float(np.clip(bearing, -1.2, 1.2))
        else:
            head_pan = 1.2 * math.sin(t * 1.0)
        if 0.05 < dist < 5.0 and abs(wrap(bearing - head_pan)) < FOV / 2:
            d = dist * (1 + rng.normal(0, 0.03))
            b = bearing + rng.normal(0, 0.01)
            fx, fy = to_field(est[:3], (d * math.cos(b), d * math.sin(b)))
            ball_filter.update(fx, fy, d, t)

        observations = []
        for type_id, points in field.landmarks.items():
            for lx, ly in points:
                r = math.hypot(lx - robot[0], ly - robot[1])
                b = wrap(math.atan2(ly - robot[1], lx - robot[0]) - robot[2])
                if 0.3 < r < 4.5 and abs(wrap(b - head_pan)) < FOV / 2 and rng.random() > 0.3:
                    observations.append((type_id, r * (1 + rng.normal(0, 0.08)),
                                         b + rng.normal(0, 0.03)))
        if observations:
            mcl.update_landmarks(observations)
        est = mcl.estimate()

        # --- decide
        bb = striker.tick(
            now=t, motion_state='WALKING' if any(cmd) else 'READY', playing=True,
            pose=est[:3], localized=est[3] < 0.45,
            ball_known=ball_filter.known(t), ball_visible=ball_filter.visible(t),
            ball_robot=to_robot(est[:3], ball_filter.position),
            ball_field=ball_filter.position)
        cmd = bb.cmd

        if bb.kick is not None:
            true_rel = to_robot(tuple(robot), tuple(ball))
            kicks.append({'t': t, 'rel': true_rel, 'heading': robot[2],
                          'ball': tuple(ball), 'foot': bb.kick})
            # Only a ball actually in front of the foot gets moved.
            if 0.10 < true_rel[0] < 0.30 and abs(true_rel[1]) < 0.12:
                ball += 2.5 * np.array([math.cos(robot[2]), math.sin(robot[2])])
                return {'kicks': kicks, 'ball': ball, 'robot': robot, 'time': t,
                        'pose_error': math.hypot(est[0] - robot[0], est[1] - robot[1])}
            cmd = (0.0, 0.0, 0.0)
    return {'kicks': kicks, 'ball': ball, 'robot': robot, 'time': t, 'pose_error': None}


@pytest.mark.parametrize('robot_start,ball_start', [
    ((-0.5, 0.0, 0.0), (0.6, 0.0)),            # straight ahead (default sim scene)
    ((-1.0, -1.0, 0.0), (0.5, 0.8)),           # ball off to the left
    ((1.0, 0.5, math.pi), (0.0, -0.5)),        # facing own goal: must go round
    ((2.0, 0.0, 0.0), (0.5, 1.0)),             # ball behind the robot
])
def test_striker_reaches_ball_and_kicks_toward_goal(robot_start, ball_start):
    result = run(robot_start, ball_start, seed=1)
    assert result['kicks'], 'never kicked'
    kick = result['kicks'][-1]
    assert 0.10 < kick['rel'][0] < 0.30 and abs(kick['rel'][1]) < 0.12, kick
    # The shot line from the ball must pass between the posts.
    bx, by = kick['ball']
    y_at_goal = by + math.tan(kick['heading']) * (4.5 - bx)
    assert abs(kick['heading']) < math.pi / 2 and abs(y_at_goal) < 1.3, (kick, y_at_goal)
    assert result['pose_error'] < 0.5
    assert (kick['foot'] == 'left') == (kick['rel'][1] > 0)
