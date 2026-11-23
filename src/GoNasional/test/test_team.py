"""Team play: role assignment, team ball, packets, and a 3-robot kinematic game.

Run: python3 -m pytest test/test_team.py
"""

import math
from types import SimpleNamespace

import numpy as np

from GoNasional.core.field_model import FieldModel
from GoNasional.core.geometry import to_robot, wrap
from GoNasional.core.striker import Striker
from GoNasional.core.team import (DEFENDER, GOALIE, STRIKER, SUPPORTER, PlayerState, TeamConfig,
                                  assign_roles, role_position, team_ball)

CFG = TeamConfig()
FIELD = FieldModel()


def player(n, x, y, th=0.0, ball=(0.0, 0.0), age=0.2, role='', localized=True, stamp=0.0):
    return PlayerState(player=n, pose=(x, y, th), localized=localized, ball=ball,
                       ball_age=age, role=role, stamp=stamp)


def test_every_robot_computes_the_same_roles():
    ps = [player(1, -4.0, 0.0), player(2, -1.0, 0.5), player(3, 1.0, -1.0)]
    ball = (0.5, -0.5)
    views = [assign_roles(me, [p for p in ps if p is not me], ball, 0.0, CFG, goalie=1)
             for me in ps]
    assert views[0] == views[1] == views[2]
    assert views[0][1] == GOALIE and views[0][3] == STRIKER and views[0][2] == DEFENDER


def test_striker_hysteresis_prevents_flapping():
    ball = (0.0, 0.0)
    a = player(2, -1.0, 0.0, role=STRIKER)
    b = player(3, -0.9, 0.0)                    # slightly closer, but not by much
    roles = assign_roles(a, [b], ball, 0.0, CFG)
    assert roles[2] == STRIKER
    b_far_ahead = player(3, -0.3, 0.0)          # clearly better placed: takes over
    assert assign_roles(a, [b_far_ahead], ball, 0.0, CFG)[3] == STRIKER


def test_stale_and_inactive_mates_are_ignored():
    me = player(2, -3.0, 0.0, stamp=10.0)
    stale = player(3, 0.1, 0.0, stamp=5.0)
    fallen = player(4, 0.2, 0.0, stamp=10.0)
    fallen.active = False
    roles = assign_roles(me, [stale, fallen], (0.0, 0.0), 10.0, CFG)
    assert roles == {2: STRIKER}


def test_team_ball_uses_teammate_when_own_is_old():
    me = player(2, 0.0, 0.0, ball=None, age=math.inf, stamp=1.0)
    mate = player(3, 1.0, 1.0, ball=(2.0, 1.0), age=0.3, stamp=0.9)
    assert team_ball(me, [mate], 1.0, CFG) == (2.0, 1.0)
    mate.localized = False                      # unlocalized teammate: field ball unusable
    assert team_ball(me, [mate], 1.0, CFG) is None


def test_positions_stay_on_field_and_off_the_ball():
    for ball in [(0.0, 0.0), (4.2, 2.8), (-4.0, -2.5), (-3.0, 0.0)]:
        for role in (GOALIE, DEFENDER, SUPPORTER):
            x, y, th = role_position(role, ball, FIELD, CFG)
            assert abs(x) <= 4.5 and abs(y) <= 3.0
            assert math.hypot(x - ball[0], y - ball[1]) >= CFG.keep_off_ball - 1e-9
    gx, gy, _ = role_position(GOALIE, (-2.0, 2.5), FIELD, CFG)
    assert gx < -4.0 and abs(gy) < FIELD.goal_width / 2.0


def test_team_packet_roundtrip_and_foreign_packets():
    from GoNasional.core.team_packet import decode, encode
    wm = SimpleNamespace(
        pose=SimpleNamespace(x=1.0, y=-2.0, theta=0.5), localized=True, ball_known=True,
        ball_field=SimpleNamespace(x=0.3, y=0.4), ball_age=0.25)
    data = encode(7, 2, wm, STRIKER, True, 5)
    msg = decode(data, 7)
    assert msg['p'] == 2 and msg['ball'] == [0.3, 0.4] and msg['role'] == STRIKER
    assert decode(data, 8) is None              # other team
    assert decode(b'garbage', 7) is None
    assert len(data) < 200


def test_three_robots_share_the_work_and_score():
    """Kinematic game: perfect localization, limited ball view, shared team ball."""
    rng = np.random.default_rng(1)
    dt, fov = 0.1, math.radians(70.0)
    starts = [(-3.5, 2.0, 0.0), (-2.0, -1.5, 0.5), (-0.5, 2.5, -0.8)]
    robots = [np.array(s, dtype=float) for s in starts]
    strikers = [Striker(FIELD) for _ in starts]
    roles = [''] * 3
    seen = [None] * 3                           # (ball field xy, time) per robot
    ball = np.array([1.0, -1.5])
    ball_vel = np.zeros(2)
    t, scored = 0.0, False
    two_strikers = crowded = ticks = 0
    while t < 300.0 and not scored:
        t += dt
        ball += ball_vel * dt
        ball_vel *= math.exp(-0.6 * dt)
        if ball[0] > 4.5 and abs(ball[1]) < FIELD.goal_width / 2:
            scored = True
        ball[0] = min(ball[0], 4.6)
        ball[1] = float(np.clip(ball[1], -3.0, 3.0))
        states = []
        for i, r in enumerate(robots):
            rel = to_robot(tuple(r), tuple(ball))
            if math.hypot(*rel) < 5.0 and abs(math.atan2(rel[1], rel[0])) < fov / 2 + 1.0:
                seen[i] = (tuple(ball + rng.normal(0, 0.03, 2)), t)
            b, bt = seen[i] if seen[i] else (None, -math.inf)
            states.append(PlayerState(player=i + 1, pose=tuple(r), localized=True, ball=b,
                                      ball_age=t - bt, role=roles[i], stamp=t))
        for i, r in enumerate(robots):
            me, mates = states[i], [s for j, s in enumerate(states) if j != i]
            tball = team_ball(me, mates, t, CFG)
            view = assign_roles(me, mates, tball, t, CFG)
            roles[i] = view[i + 1]
            position = None if roles[i] == STRIKER else role_position(roles[i], tball, FIELD, CFG)
            visible = me.ball_age < 0.3
            bb = strikers[i].tick(
                now=t, motion_state='WALKING', game='playing', pose=tuple(r), localized=True,
                ball_known=tball is not None, ball_visible=visible,
                ball_robot=to_robot(tuple(r), tball) if tball else (0.0, 0.0),
                ball_field=tball or (0.0, 0.0), role=roles[i] if position else STRIKER,
                position=position)
            vx, vy, wz = bb.cmd
            r[0] += (vx * math.cos(r[2]) - vy * math.sin(r[2])) * dt
            r[1] += (vx * math.sin(r[2]) + vy * math.cos(r[2])) * dt
            r[2] = wrap(r[2] + wz * dt)
            if bb.kick is not None:
                ball_vel = 1.2 * np.array([math.cos(r[2]), math.sin(r[2])])
        ticks += 1
        two_strikers += roles.count(STRIKER) > 1
        others = [i for i in range(3) if roles[i] != STRIKER]
        crowded += any(math.hypot(*(robots[i][:2] - ball)) < 0.3 for i in others)
    assert scored, f'no goal in {t:.0f} s, ball at {ball}'
    assert two_strikers / ticks < 0.05
    assert crowded / ticks < 0.05
