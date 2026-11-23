"""Striker behavior: find the ball, get behind it, aim at the goal, kick.

Pure decision logic on a blackboard; the ROS node only copies topics in and
commands out. That keeps the same tree usable in Webots, on the robot, and in
the kinematic test in test/test_closed_loop.py.
"""

import math
from dataclasses import dataclass, field as dc_field

from .bt import Action, Condition, Selector, Sequence, Status
from .field_model import FieldModel
from .geometry import clamp, wrap


@dataclass
class StrikerConfig:
    # Walking limits (m/s, rad/s) the behavior may request.
    max_vx: float = 0.06
    max_vy: float = 0.03
    max_wz: float = 0.5
    back_vx: float = 0.03

    # Approach.
    near_radius: float = 0.60          # switch from approach to align
    near_hysteresis: float = 0.15
    standoff: float = 0.40             # approach point this far behind the ball
    wrong_side_offset: float = 0.50    # detour radius when the ball is between us and our aim
    turn_in_place_angle: float = 0.6   # rad; beyond this, turn before walking

    # Align / kick geometry in the robot frame (m).
    orbit_distance: float = 0.30
    kick_x: float = 0.19               # ball this far ahead of base_footprint
    kick_y: float = 0.055              # lateral offset of the kicking foot
    kick_tol_x: float = 0.03
    kick_tol_y: float = 0.03
    aim_tol: float = math.radians(10.0)
    aim_exit_tol: float = math.radians(20.0)
    kick_settle_s: float = 0.4
    kick_cooldown_s: float = 2.0
    foot_switch_margin: float = 0.03

    # Gains.
    k_turn: float = 1.5
    k_lin: float = 0.8

    # Search.
    search_wait_s: float = 4.0         # let the head scan before the body turns
    search_wz: float = 0.35

    # Glancing away from the ball to pick up landmarks while not localized.
    glance_min_ball_distance: float = 1.2
    glance_period_s: float = 8.0
    glance_duration_s: float = 3.0

    post_margin: float = 0.35

    # Positioning in the READY state: x, y (m), theta (rad) in the field frame.
    ready_pose_kickoff: tuple = (-0.75, 0.0, 0.0)     # we kick off: just behind the ball
    ready_pose_defend: tuple = (-2.0, 0.0, 0.0)       # they kick off: outside the centre circle
    position_tol: float = 0.25
    heading_tol: float = 0.25


@dataclass
class Blackboard:
    # Inputs, refreshed by the caller before every tick.
    now: float = 0.0
    motion_state: str = 'IDLE'
    playing: bool = False
    # Referee phase: initial | ready | set | playing | hold | penalized | finished.
    # None means "derive from `playing`" (manual start button, no GameController).
    game: str = None
    kickoff_ours: bool = False
    pose: tuple = (0.0, 0.0, 0.0)
    localized: bool = False
    ball_known: bool = False
    ball_visible: bool = False
    ball_robot: tuple = (0.0, 0.0)
    ball_field: tuple = (0.0, 0.0)
    # Team play (core/team.py). Anything but 'striker' walks to `position`
    # (field frame x, y, theta) and watches the ball instead of playing it.
    role: str = 'striker'
    position: tuple = None

    # Outputs, reset at the start of every tick.
    cmd: tuple = (0.0, 0.0, 0.0)
    head_mode: str = 'track'
    kick: str = None
    active_leaf: str = ''
    debug: dict = dc_field(default_factory=dict)


class Striker:

    def __init__(self, field: FieldModel, config: StrikerConfig = None):
        self.field = field
        self.cfg = config or StrikerConfig()
        self.bb = Blackboard()
        self._near = False
        self._aligned = False
        self._foot = 'right'
        self._zone_since = None
        self._lost_since = None
        self._last_ball_side = 1.0
        self._last_kick = -math.inf
        self._glance_start = -math.inf
        self._in_position = False
        self.tree = self._build()

    def _build(self):
        return Selector('root', [
            Sequence('not_ready', [
                Condition('motion_busy', lambda bb: bb.motion_state not in ('READY', 'WALKING')),
                Action('wait_for_motion', self._wait),
            ]),
            Sequence('penalized', [
                Condition('is_penalized', lambda bb: bb.game in ('penalized', 'finished')),
                Action('stand_out', self._stand_out),
            ]),
            Sequence('ready', [
                Condition('game_ready', lambda bb: bb.game == 'ready'),
                Action('go_to_position', self._go_to_position),
            ]),
            Sequence('not_playing', [
                Condition('game_stopped', lambda bb: bb.game != 'playing'),
                Action('stand_and_localize', self._stand),
            ]),
            Sequence('support', [
                Condition('not_striker', lambda bb: bb.role != 'striker'
                          and bb.position is not None),
                Action('hold_position', self._hold_position),
            ]),
            Sequence('no_ball', [
                Condition('ball_unknown', lambda bb: not bb.ball_known),
                Action('search_ball', self._search),
            ]),
            Sequence('kick', [
                Condition('in_kick_zone', self._in_kick_zone),
                Action('kick_ball', self._kick),
            ]),
            Sequence('align', [
                Condition('ball_near', self._ball_near),
                Action('align_to_goal', self._align),
            ]),
            Action('approach_ball', self._approach),
        ])

    # ------------------------------------------------------------------ tick

    def tick(self, **inputs):
        bb = self.bb
        for key, value in inputs.items():
            setattr(bb, key, value)
        bb.cmd = (0.0, 0.0, 0.0)
        bb.head_mode = 'track'
        bb.kick = None
        bb.active_leaf = ''
        bb.debug = {}
        if 'game' not in inputs or bb.game is None:
            bb.game = 'playing' if bb.playing else 'initial'

        if bb.ball_known:
            self._lost_since = None
            if abs(bb.ball_robot[1]) > 0.05:
                self._last_ball_side = math.copysign(1.0, bb.ball_robot[1])
        elif self._lost_since is None:
            self._lost_since = bb.now

        self.tree.tick(bb)
        if bb.active_leaf not in ('kick_ball', 'align_to_goal'):
            self._zone_since = None
        if bb.active_leaf not in ('go_to_position', 'hold_position'):
            self._in_position = False
        if bb.active_leaf in ('wait_for_motion', 'stand_and_localize', 'search_ball',
                              'stand_out', 'go_to_position', 'hold_position'):
            self._aligned = False
            self._near = False
        return bb

    # --------------------------------------------------------------- helpers

    def _aim_error(self, bb):
        """Angle the robot must turn to face along the shot direction."""
        if bb.localized:
            target = self.field.aim_point(bb.ball_field[1], self.cfg.post_margin)
            aim = math.atan2(target[1] - bb.ball_field[1], target[0] - bb.ball_field[0])
        else:
            # Without a position fix the heading (IMU-anchored) is still good:
            # shoot along the field's +x axis, as the legacy yaw logic did.
            aim = 0.0
        return wrap(aim - bb.pose[2])

    def _limit(self, vx, vy, wz):
        c = self.cfg
        return (clamp(vx, -c.back_vx, c.max_vx),
                clamp(vy, -c.max_vy, c.max_vy),
                clamp(wz, -c.max_wz, c.max_wz))

    def _ball_near(self, bb):
        dist = math.hypot(*bb.ball_robot)
        limit = self.cfg.near_radius + (self.cfg.near_hysteresis if self._near else 0.0)
        self._near = dist < limit
        return self._near

    def _zone_errors(self, bb):
        c = self.cfg
        by = bb.ball_robot[1]
        if self._foot == 'left' and by < -c.foot_switch_margin:
            self._foot = 'right'
        elif self._foot == 'right' and by > c.foot_switch_margin:
            self._foot = 'left'
        foot_y = c.kick_y if self._foot == 'left' else -c.kick_y
        return bb.ball_robot[0] - c.kick_x, by - foot_y

    def _in_kick_zone(self, bb):
        c = self.cfg
        if not (self._aligned and bb.ball_visible):
            return False
        if bb.now - self._last_kick < c.kick_cooldown_s:
            return False
        ex, ey = self._zone_errors(bb)
        return (abs(ex) < c.kick_tol_x and abs(ey) < c.kick_tol_y
                and abs(self._aim_error(bb)) < c.aim_exit_tol)

    # --------------------------------------------------------------- actions

    def _wait(self, bb):
        bb.cmd = (0.0, 0.0, 0.0)

    def _stand(self, bb):
        # Not allowed to move (initial, set, play stopped, opponent's kick).
        # Keep looking: at landmarks until localized, then at the ball.
        bb.head_mode = 'track' if bb.localized else 'scan'

    def _stand_out(self, bb):
        bb.head_mode = 'center'

    def _go_to_position(self, bb):
        """Walk to the kick-off position; there is no ball to care about yet."""
        c = self.cfg
        target = c.ready_pose_kickoff if bb.kickoff_ours else c.ready_pose_defend
        bb.head_mode = 'scan'
        self._walk_to(bb, target)

    def _hold_position(self, bb):
        """Non-striker: stand at the role position, watching the ball."""
        if not bb.localized:
            # Position targets mean nothing without a fix: look for landmarks.
            bb.head_mode = 'scan'
            return None
        bb.head_mode = 'track' if bb.ball_known else 'scan'
        self._walk_to(bb, bb.position)
        if bb.ball_known:
            # Do not walk into the ball (and the striker playing it): step aside.
            bx, by = bb.ball_robot
            if math.hypot(bx, by) < 0.6 and bx > -0.1 and bb.cmd[0] > 0.0:
                side = -1.0 if by > 0.0 else 1.0
                bb.cmd = self._limit(0.0, side * self.cfg.max_vy, bb.cmd[2])
        return None

    def _walk_to(self, bb, target):
        c = self.cfg
        tx, ty, ttheta = target
        x, y, theta = bb.pose
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)
        bb.debug = {'dist': dist}
        # Once there, stay unless clearly pushed off; otherwise estimate
        # jitter makes the robot shuffle all through READY.
        tolerance = c.position_tol * (2.0 if self._in_position else 1.0)
        self._in_position = dist <= tolerance
        if not self._in_position:
            heading = wrap(math.atan2(dy, dx) - theta)
            speed = 0.0 if abs(heading) > c.turn_in_place_angle else c.max_vx * math.cos(heading)
            bb.cmd = self._limit(speed, 0.0, c.k_turn * heading)
            return None
        heading = wrap(ttheta - theta)
        if abs(heading) > c.heading_tol:
            bb.cmd = self._limit(0.0, 0.0, c.k_turn * heading)
        return None

    def _search(self, bb):
        lost_for = 0.0 if self._lost_since is None else bb.now - self._lost_since
        if lost_for > self.cfg.search_wait_s:
            bb.cmd = (0.0, 0.0, self._last_ball_side * self.cfg.search_wz)

    def _kick(self, bb):
        if self._zone_since is None:
            self._zone_since = bb.now
        if bb.now - self._zone_since < self.cfg.kick_settle_s:
            return None
        bb.kick = self._foot
        self._last_kick = bb.now
        self._zone_since = None
        self._aligned = False
        return Status.SUCCESS

    def _align(self, bb):
        c = self.cfg
        bx, by = bb.ball_robot
        dist = math.hypot(bx, by)
        bearing = math.atan2(by, bx)
        aim_err = self._aim_error(bb)
        bb.debug = {'aim_err': aim_err, 'dist': dist}

        if self._aligned and abs(aim_err) > c.aim_exit_tol:
            self._aligned = False
        elif not self._aligned and abs(aim_err) < c.aim_tol:
            self._aligned = True

        if not self._aligned:
            # Circle the ball while facing it. To rotate the shot direction
            # counter-clockwise the robot has to step to its right, and back.
            vy = -math.copysign(c.max_vy, aim_err) * min(1.0, abs(aim_err) / 0.5)
            bb.cmd = self._limit(c.k_lin * (dist - c.orbit_distance), vy,
                                 c.k_turn * bearing)
            return None

        ex, ey = self._zone_errors(bb)
        bb.cmd = self._limit(c.k_lin * ex, c.k_lin * ey, 0.5 * c.k_turn * aim_err)
        return None

    def _approach(self, bb):
        c = self.cfg
        bx, by = bb.ball_robot
        dist = math.hypot(bx, by)
        bearing = math.atan2(by, bx)
        aim_err = self._aim_error(bb)

        # Aim direction expressed in the robot frame is simply aim_err.
        ax, ay = math.cos(aim_err), math.sin(aim_err)
        if abs(wrap(aim_err - bearing)) > math.radians(100.0):
            # We are on the goal side of the ball: go round it, not through it.
            side = 1.0 if (ax * by - ay * bx) < 0.0 else -1.0
            px = bx - side * ay * c.wrong_side_offset
            py = by + side * ax * c.wrong_side_offset
        else:
            px, py = bx - c.standoff * ax, by - c.standoff * ay

        heading = math.atan2(py, px) if math.hypot(px, py) > 0.15 else bearing
        speed = 0.0 if abs(heading) > c.turn_in_place_angle else c.max_vx * math.cos(heading)
        bb.cmd = self._limit(speed, 0.0, c.k_turn * heading)
        bb.debug = {'aim_err': aim_err, 'dist': dist}

        if not bb.localized and dist > c.glance_min_ball_distance:
            since = bb.now - self._glance_start
            if since > c.glance_period_s:
                self._glance_start = bb.now
                since = 0.0
            if since < c.glance_duration_s:
                bb.head_mode = 'scan'
        return None
