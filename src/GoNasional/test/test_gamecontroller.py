"""GameController codec and referee-aware behavior. No ROS needed."""

import math
import struct

from GoNasional.core import gamecontroller as gc
from GoNasional.core.field_model import FieldModel
from GoNasional.core.striker import Striker


def test_sizes_match_the_c_structs():
    # sizeof(RoboCupGameControlData) and sizeof(RoboCupGameControlReturnData), v20.
    assert gc.CONTROL_SIZE == 158
    assert gc.RETURN_SIZE == 32


def test_control_round_trip_and_field_offsets():
    control = gc.ControlData(packet_number=7, state=gc.STATE_PLAYING, stopped=True,
                             set_play=4, first_half=False, kicking_team=12,
                             secs_remaining=-5, secondary_time=9)
    control.teams[0].number, control.teams[1].number = 12, 30
    control.teams[1].score = 3
    control.teams[1].goalkeeper = 1
    control.teams[0].players[2] = gc.PlayerInfo(penalty=6, secs_till_unpenalized=40, cautions=1)
    data = gc.pack_control(control)

    # Hand-checked offsets from the header file.
    assert data[:4] == b'RGme' and data[4] == 20 and data[5] == 7
    assert data[8] == 1 and data[10] == gc.STATE_PLAYING and data[13] == 12
    assert struct.unpack_from('<hh', data, 14) == (-5, 9)
    assert data[18] == 12 and data[18 + 70] == 30
    assert data[18 + 10 + 2 * 3] == 6          # team 0, player 3, penalty

    parsed = gc.parse_control(data)
    assert parsed == control
    assert parsed.team(30).score == 3 and parsed.opponent(30).number == 12
    assert parsed.team(99) is None and parsed.opponent(99) is None


def test_bad_packets_are_rejected():
    good = gc.pack_control(gc.ControlData())
    assert gc.parse_control(good) is not None
    assert gc.parse_control(good[:-1]) is None
    assert gc.parse_control(b'XXXX' + good[4:]) is None
    assert gc.parse_control(good[:4] + bytes([12]) + good[5:]) is None     # old protocol
    assert gc.parse_control(good[:10] + bytes([9]) + good[11:]) is None    # invalid state
    assert gc.parse_control(b'') is None


def test_status_packet_layout():
    data = gc.pack_status(12, 3, fallen=True, pose=(1.5, -0.25, 0.5),
                          ball_age=2.0, ball=(0.4, 0.1))
    assert len(data) == 32
    header, version, player, team, fallen = struct.unpack_from('<4s4B', data, 0)
    assert (header, version, player, team, fallen) == (b'RGrt', 4, 3, 12, 1)
    x, y, theta, ball_age, bx, by = struct.unpack_from('<6f', data, 8)
    assert (x, y, bx, by) == (1500.0, -250.0, 400.0, 100.0)
    assert abs(theta - 0.5) < 1e-6 and ball_age == 2.0


def tick(striker, game, pose=(-3.0, 2.0, 0.0), kickoff_ours=False, ball=(0.5, 0.0)):
    return striker.tick(now=1.0, motion_state='READY', game=game, kickoff_ours=kickoff_ours,
                        pose=pose, localized=True, ball_known=True, ball_visible=True,
                        ball_robot=ball, ball_field=(0.0, 0.0))


def test_robot_does_not_move_when_the_referee_says_so():
    striker = Striker(FieldModel())
    for game in ('initial', 'set', 'hold', 'penalized', 'finished'):
        bb = tick(striker, game)
        assert bb.cmd == (0.0, 0.0, 0.0) and bb.kick is None, game
    assert tick(striker, 'penalized').head_mode == 'center'
    assert tick(striker, 'playing').cmd != (0.0, 0.0, 0.0)


def test_ready_walks_to_kickoff_position_and_stops_there():
    striker = Striker(FieldModel())
    cfg = striker.cfg
    for ours, target in ((True, cfg.ready_pose_kickoff), (False, cfg.ready_pose_defend)):
        pose = [-3.0, 2.5, -math.pi / 2]        # just walked in from the touchline
        for _ in range(3000):
            bb = tick(striker, 'ready', pose=tuple(pose), kickoff_ours=ours)
            vx, vy, wz = bb.cmd
            pose[0] += (vx * math.cos(pose[2]) - vy * math.sin(pose[2])) * 0.1
            pose[1] += (vx * math.sin(pose[2]) + vy * math.cos(pose[2])) * 0.1
            pose[2] += wz * 0.1
        assert math.hypot(pose[0] - target[0], pose[1] - target[1]) < 0.3, (ours, pose)
        assert abs(pose[2] - target[2]) < 0.3
        assert bb.cmd == (0.0, 0.0, 0.0)
        assert pose[0] < 0.0                    # never left our own half
