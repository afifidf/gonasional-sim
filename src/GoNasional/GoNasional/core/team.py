"""Team play: shared team ball, role assignment and positions for non-strikers.

Pure Python, no ROS. Every robot runs the same deterministic assignment on the
same shared information (its own state plus the latest message of each
teammate), so the roles agree without a leader. Disagreements during the
second or so after something changes are tolerated: two robots may both go to
the ball briefly, and the hysteresis bonus settles it.

Roles:
  goalie     the goalkeeper (GameController flag or `goalie_player`)
  striker    lowest estimated time to reach the ball and get behind it
  defender   of the rest, the one nearest to our own goal
  supporter  everybody else
"""

import math
from dataclasses import dataclass, field as dc_field

from .geometry import clamp, wrap

GOALIE, STRIKER, DEFENDER, SUPPORTER = 'goalie', 'striker', 'defender', 'supporter'
ROLES = (GOALIE, STRIKER, DEFENDER, SUPPORTER)


@dataclass
class PlayerState:
    """What one robot knows about itself; also what it broadcasts."""
    player: int
    pose: tuple = (0.0, 0.0, 0.0)          # field frame
    localized: bool = False
    ball: tuple = None                      # field frame, or None when unknown
    ball_age: float = math.inf              # s since that robot last saw the ball
    role: str = ''
    active: bool = True                     # standing, not penalized, playing
    stamp: float = 0.0                      # receive (or own) time, s


@dataclass
class TeamConfig:
    walk_speed: float = 0.06               # m/s, for time estimates
    turn_speed: float = 0.4                # rad/s
    not_localized_penalty: float = 6.0     # s added when the robot does not know where it is
    no_ball_penalty: float = 8.0           # s added when the robot does not see the ball itself
    striker_bonus: float = 4.0             # s subtracted for the current striker (hysteresis)
    mate_timeout: float = 3.0              # s; older teammate messages are ignored
    team_ball_max_age: float = 3.0         # s; teammate ball observations older than this are ignored
    goalie_player: int = 0                 # goalkeeper when no GameController says (0 = none)
    # Positions (field frame, +x toward the goal we attack).
    defender_ratio: float = 0.35           # fraction of the way from our goal to the ball
    defender_max_x: float = -0.8
    supporter_back: float = 1.3            # m behind the ball
    supporter_side: float = 1.0            # m to the side, toward the field centre
    goalie_line: float = 0.35              # m in front of our goal line
    goalie_clear_radius: float = 0.8       # goalie attacks a ball this close in its area
    keep_off_ball: float = 0.6             # non-strikers stay this far from the ball


def time_to_ball(player, ball, cfg, field_aim=(4.5, 0.0)):
    """Estimated seconds for `player` to stand behind `ball` facing the goal."""
    x, y, theta = player.pose
    dist = math.hypot(ball[0] - x, ball[1] - y)
    bearing = math.atan2(ball[1] - y, ball[0] - x)
    turn = abs(wrap(bearing - theta))
    # Coming from the goal side means walking round the ball.
    aim = math.atan2(field_aim[1] - ball[1], field_aim[0] - ball[0])
    behind = abs(wrap(aim - bearing))
    t = dist / cfg.walk_speed + turn / cfg.turn_speed + behind / cfg.turn_speed
    if not player.localized:
        t += cfg.not_localized_penalty
    if player.ball_age > 1.0:
        t += cfg.no_ball_penalty
    if player.role == STRIKER:
        t -= cfg.striker_bonus
    return t


def team_ball(me, mates, now, cfg):
    """Best ball estimate in the field frame: own if fresh, else the freshest teammate's."""
    if me.ball is not None and me.ball_age <= 1.0:
        return me.ball
    best = None
    for mate in mates:
        if mate.ball is None or not mate.localized:
            continue
        age = mate.ball_age + (now - mate.stamp)
        if age <= cfg.team_ball_max_age and (best is None or age < best[0]):
            best = (age, mate.ball)
    if best is not None:
        return best[1]
    return me.ball


def assign_roles(me, mates, ball, now, cfg, goalie=None):
    """{player number: role} for every active robot, the same on every robot.

    goalie: player number of the goalkeeper, or None for `cfg.goalie_player`.
    """
    goalie = cfg.goalie_player if goalie is None else goalie
    players = [me] + [m for m in mates
                      if m.player != me.player and now - m.stamp <= cfg.mate_timeout]
    active = sorted((p for p in players if p.active), key=lambda p: p.player)
    roles = {}
    field_players = []
    for p in active:
        if p.player == goalie:
            roles[p.player] = GOALIE
        else:
            field_players.append(p)
    if not field_players:
        return roles
    if ball is None:
        # Nobody knows the ball: the lowest number searches as striker.
        striker = field_players[0]
    else:
        striker = min(field_players, key=lambda p: (time_to_ball(p, ball, cfg), p.player))
    roles[striker.player] = STRIKER
    rest = [p for p in field_players if p is not striker]
    if rest:
        defender = min(rest, key=lambda p: (p.pose[0] + 4.5, p.player))
        roles[defender.player] = DEFENDER
        for p in rest:
            if p is not defender:
                roles[p.player] = SUPPORTER
    return roles


def role_position(role, ball, field, cfg, me_pose=None):
    """(x, y, theta) target for a non-striker role, facing the ball."""
    hl, hw = field.length / 2.0, field.width / 2.0
    own_goal = (-hl, 0.0)
    if ball is None:
        ball = (0.0, 0.0)
    if role == GOALIE:
        half = max(0.0, field.goal_width / 2.0 - 0.2)
        x = -hl + cfg.goalie_line
        y = clamp(ball[1] * 0.5, -half, half)
    elif role == DEFENDER:
        x = own_goal[0] + cfg.defender_ratio * (ball[0] - own_goal[0])
        y = own_goal[1] + cfg.defender_ratio * (ball[1] - own_goal[1])
        x = min(x, cfg.defender_max_x)
    else:  # supporter: behind the ball, offset toward the middle of the field
        side = -1.0 if ball[1] > 0.0 else 1.0
        x = ball[0] - cfg.supporter_back
        y = ball[1] + side * cfg.supporter_side
    x = clamp(x, -hl + 0.2, hl - 0.3)
    y = clamp(y, -hw + 0.3, hw - 0.3)
    # Never park on top of the ball (the striker needs the space).
    dx, dy = x - ball[0], y - ball[1]
    d = math.hypot(dx, dy)
    if d < cfg.keep_off_ball:
        if d < 1e-6:
            dx, dy, d = -1.0, 0.0, 1.0
        x, y = ball[0] + dx / d * cfg.keep_off_ball, ball[1] + dy / d * cfg.keep_off_ball
    theta = math.atan2(ball[1] - y, ball[0] - x)
    return (x, y, theta)


def goalie_should_clear(me_pose, ball, field, cfg):
    """True when the ball is inside our goal area and close: the goalie plays it."""
    if ball is None:
        return False
    hl = field.length / 2.0
    in_area = ball[0] < -hl + 1.5 and abs(ball[1]) < field.goal_width / 2.0 + 0.6
    near = math.hypot(ball[0] - me_pose[0], ball[1] - me_pose[1]) < cfg.goalie_clear_radius
    return in_area and near


@dataclass
class TeamView:
    """Result of one team update for this robot."""
    role: str = STRIKER
    roles: dict = dc_field(default_factory=dict)
    ball: tuple = None
    target: tuple = None
