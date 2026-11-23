"""RoboCup Humanoid Soccer League GameController wire format (struct version 20).

Layout taken from game_controller_msgs/headers/RoboCupGameControlData.h of
https://github.com/RoboCup-HumanoidSoccerLeague/GameController. All integers
are little-endian and the structs have no padding. No ROS imports.
"""

import struct
from dataclasses import dataclass, field
from typing import List, Optional

DATA_PORT = 3838
RETURN_PORT = 3939

HEADER = b'RGme'
VERSION = 20
MAX_NUM_PLAYERS = 20
RETURN_HEADER = b'RGrt'
RETURN_VERSION = 4

STATE_INITIAL, STATE_READY, STATE_SET, STATE_PLAYING, STATE_FINISHED = range(5)
KICKING_TEAM_NONE = 255
PENALTY_NONE = 0
PENALTY_SUBSTITUTE = 13

_HEAD = struct.Struct('<4s10B2h')          # header .. kickingTeam, secsRemaining, secondaryTime
_TEAM = struct.Struct('<6B2H')             # teamNumber .. penaltyShot, singleShots, messageBudget
_PLAYER = struct.Struct('<3B')             # penalty, secsTillUnpenalised, cautions
_RETURN = struct.Struct('<4s4B3ff2f')      # header, version, player, team, fallen, pose, ballAge, ball

TEAM_SIZE = _TEAM.size + MAX_NUM_PLAYERS * _PLAYER.size
CONTROL_SIZE = _HEAD.size + 2 * TEAM_SIZE
RETURN_SIZE = _RETURN.size


@dataclass
class PlayerInfo:
    penalty: int = PENALTY_NONE
    secs_till_unpenalized: int = 0
    cautions: int = 0


@dataclass
class TeamInfo:
    number: int = 0
    field_player_color: int = 0
    goalkeeper_color: int = 0
    goalkeeper: int = 0
    score: int = 0
    penalty_shot: int = 0
    single_shots: int = 0
    message_budget: int = 0
    players: List[PlayerInfo] = field(
        default_factory=lambda: [PlayerInfo() for _ in range(MAX_NUM_PLAYERS)])


@dataclass
class ControlData:
    packet_number: int = 0
    players_per_team: int = 4
    competition_type: int = 0
    stopped: bool = False
    game_phase: int = 0
    state: int = STATE_INITIAL
    set_play: int = 0
    first_half: bool = True
    kicking_team: int = KICKING_TEAM_NONE
    secs_remaining: int = 0
    secondary_time: int = 0
    teams: List[TeamInfo] = field(default_factory=lambda: [TeamInfo(), TeamInfo()])

    def team(self, number) -> Optional[TeamInfo]:
        for team in self.teams:
            if team.number == number:
                return team
        return None

    def opponent(self, number) -> Optional[TeamInfo]:
        if self.team(number) is None:
            return None
        return next((t for t in self.teams if t.number != number), None)


def parse_control(data: bytes) -> Optional[ControlData]:
    """Decode a control packet; None if it is not a valid v20 packet."""
    if len(data) != CONTROL_SIZE or data[:4] != HEADER or data[4] != VERSION:
        return None
    (_, _, packet_number, players_per_team, competition_type, stopped, game_phase,
     state, set_play, first_half, kicking_team, secs_remaining,
     secondary_time) = _HEAD.unpack_from(data, 0)
    if state > STATE_FINISHED:
        return None
    out = ControlData(
        packet_number=packet_number, players_per_team=players_per_team,
        competition_type=competition_type, stopped=bool(stopped), game_phase=game_phase,
        state=state, set_play=set_play, first_half=bool(first_half),
        kicking_team=kicking_team, secs_remaining=secs_remaining,
        secondary_time=secondary_time, teams=[])
    offset = _HEAD.size
    for _ in range(2):
        (number, field_color, keeper_color, goalkeeper, score, penalty_shot,
         single_shots, message_budget) = _TEAM.unpack_from(data, offset)
        offset += _TEAM.size
        players = []
        for _ in range(MAX_NUM_PLAYERS):
            players.append(PlayerInfo(*_PLAYER.unpack_from(data, offset)))
            offset += _PLAYER.size
        out.teams.append(TeamInfo(number, field_color, keeper_color, goalkeeper, score,
                                  penalty_shot, single_shots, message_budget, players))
    return out


def pack_control(control: ControlData) -> bytes:
    """Encode a control packet (used by tests and the fake referee tool)."""
    data = bytearray(_HEAD.pack(
        HEADER, VERSION, control.packet_number & 0xFF, control.players_per_team,
        control.competition_type, int(control.stopped), control.game_phase, control.state,
        control.set_play, int(control.first_half), control.kicking_team,
        control.secs_remaining, control.secondary_time))
    for team in control.teams:
        data += _TEAM.pack(team.number, team.field_player_color, team.goalkeeper_color,
                           team.goalkeeper, team.score, team.penalty_shot,
                           team.single_shots, team.message_budget)
        for player in team.players:
            data += _PLAYER.pack(player.penalty, player.secs_till_unpenalized, player.cautions)
    return bytes(data)


def pack_status(team_number, player_number, fallen, pose=(0.0, 0.0, 0.0),
                ball_age=-1.0, ball=(0.0, 0.0)) -> bytes:
    """Encode the robot's reply.

    pose: x, y in metres and theta in rad, field frame with +x toward the goal
    we attack. ball: metres in the robot frame. The wire format wants
    millimetres, converted here.
    """
    return _RETURN.pack(
        RETURN_HEADER, RETURN_VERSION, player_number, team_number, 1 if fallen else 0,
        pose[0] * 1000.0, pose[1] * 1000.0, pose[2],
        ball_age, ball[0] * 1000.0, ball[1] * 1000.0)
