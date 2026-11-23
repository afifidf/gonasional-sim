"""Behavior node: runs the striker tree on the world model.

It only talks to the abstract interfaces (/world_model, /motion/*, /cmd_vel,
/head/mode, /game_state), never to /robotis/* or to Webots, so the same node
runs in simulation and on the robot.

Buttons (OpenCR): `user` toggles stand/sit, `start` toggles play/pause when no
GameController is feeding /game_state.

Team play: with teammates on /team/state (team_comm), every robot runs the same
role assignment (core/team.py). Only the striker plays the ball; the goalie,
defender and supporter walk to their positions and watch it. A robot that does
not see the ball itself uses the freshest teammate observation. Alone, the
robot is always the striker (or the goalie, if it is the goalkeeper).
"""

import math

import rclpy
from aroc_interfaces.msg import GameState, TeamState, WorldModel
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from ..core.field_model import FieldModel
from ..core.geometry import to_robot
from ..core.striker import Striker, StrikerConfig
from ..core.team import (GOALIE, STRIKER, PlayerState, TeamConfig, assign_roles,
                         goalie_should_clear, role_position, team_ball)


class BehaviorNode(Node):

    def __init__(self):
        super().__init__('behavior')
        p = self.declare_parameter
        field = FieldModel(
            length=p('field.length', 9.0).value,
            width=p('field.width', 6.0).value,
            goal_width=p('field.goal_width', 2.6).value,
            penalty_mark_distance=p('field.penalty_mark_distance', 1.5).value)
        config = StrikerConfig()
        for name, default in vars(config).items():
            value = p(f'striker.{name}', list(default) if isinstance(default, tuple)
                      else default).value
            setattr(config, name, tuple(value) if isinstance(default, tuple) else value)
        self.striker = Striker(field, config)
        self.field = field

        self.player = int(p('player_number', 1).value)
        self.team_cfg = TeamConfig()
        for name, default in vars(self.team_cfg).items():
            setattr(self.team_cfg, name, p(f'team.{name}', default).value)
        self.team = None
        self.role = STRIKER

        self.autostart = p('autostart', False).value
        self.world_model_timeout = p('world_model_timeout', 0.5).value
        self.game_state_timeout = p('game_state_timeout', 5.0).value
        # Where the robot is when the referee calls READY / lifts a penalty:
        # reentry (a touchline of our half), start (start_pose) or none.
        self.reset_on_ready = p('reset_on_ready', 'reentry').value
        self.reset_on_unpenalize = p('reset_on_unpenalize', 'reentry').value
        self._last_game = None
        self._referee_seen = False

        self.motion_state = 'IDLE'
        self.wm = None
        self.wm_time = None
        self.manual_playing = self.autostart
        self.game_state = None
        self.game_state_time = None
        self._stand_requested_at = -math.inf
        self._last_leaf = ''
        self._last_head_mode = None

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.kick_pub = self.create_publisher(String, '/motion/kick', 10)
        self.head_pub = self.create_publisher(String, '/head/mode', 10)
        self.motion_cmd_pub = self.create_publisher(String, '/motion/command', 10)
        self.state_pub = self.create_publisher(String, '/behavior/state', 10)
        self.reset_pub = self.create_publisher(String, '/localization/reset', 1)
        self.role_pub = self.create_publisher(String, '/behavior/role', 10)
        self.create_subscription(TeamState, '/team/state', self._on_team, 10)

        self.create_subscription(WorldModel, '/world_model', self._on_world_model, 10)
        self.create_subscription(String, '/motion/status', self._on_motion_status, 10)
        self.create_subscription(GameState, '/game_state', self._on_game_state, 10)
        self.create_subscription(String, '/robotis/open_cr/button', self._on_button, 10)

        self.create_timer(1.0 / p('rate_hz', 10.0).value, self._tick)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_world_model(self, msg):
        self.wm = msg
        self.wm_time = self._now()

    def _on_team(self, msg):
        self.team = msg

    def _team_view(self, now, wm, game):
        """(role, ball field xy or None, position or None) for this tick."""
        ball_age = wm.ball_age if wm.ball_age >= 0.0 else math.inf
        me = PlayerState(
            player=self.player, pose=(wm.pose.x, wm.pose.y, wm.pose.theta),
            localized=wm.localized,
            ball=(wm.ball_field.x, wm.ball_field.y) if wm.ball_known else None,
            ball_age=ball_age, role=self.role,
            active=self.motion_state in ('READY', 'WALKING', 'KICKING')
            and game not in ('penalized', 'finished'),
            stamp=now)
        mates = []
        goalie = None
        if self.game_state is not None and self.game_state.is_goalkeeper:
            goalie = self.player
        for m in (self.team.mates if self.team is not None else []):
            mates.append(PlayerState(
                player=m.player, pose=(m.pose.x, m.pose.y, m.pose.theta),
                localized=m.localized,
                ball=(m.ball.x, m.ball.y) if m.ball_known else None,
                ball_age=m.ball_age if m.ball_known else math.inf,
                role=m.role, active=m.active, stamp=now - m.age))
            if m.role == GOALIE and goalie is None:
                goalie = m.player
        ball = team_ball(me, mates, now, self.team_cfg)
        roles = assign_roles(me, mates, ball, now, self.team_cfg, goalie=goalie)
        role = roles.get(self.player, STRIKER)
        if role == GOALIE and goalie_should_clear(me.pose, ball, self.field, self.team_cfg):
            return GOALIE, ball, None        # goalie plays the ball itself (striker tree)
        position = None if role == STRIKER else role_position(
            role, ball, self.field, self.team_cfg, me.pose)
        return role, ball, position

    def _on_motion_status(self, msg):
        self.motion_state = msg.data

    def _on_game_state(self, msg):
        self.game_state = msg
        self.game_state_time = self._now()

    def _on_button(self, msg):
        button = msg.data.strip().lower()
        if button == 'user':
            command = 'stand' if self.motion_state == 'IDLE' else 'sit'
            if command == 'sit':
                self.manual_playing = False
            self.motion_cmd_pub.publish(String(data=command))
        elif button == 'start':
            self.manual_playing = not self.manual_playing
            self.get_logger().info(f'Manual referee: {"PLAY" if self.manual_playing else "STOP"}')

    def _game(self, now):
        """Referee phase for the behavior tree, and whether the kick-off is ours."""
        referee_alive = (self.game_state_time is not None
                         and now - self.game_state_time <= self.game_state_timeout)
        if not referee_alive:
            if self._referee_seen:
                # Lost the GameController mid-game (WiFi drop): stand still
                # rather than play on unsupervised. `start` takes over by hand.
                self._referee_seen = False
                self.manual_playing = False
                self.get_logger().warn('GameController lost: holding, press start to play')
            return ('playing' if self.manual_playing else 'initial'), False
        self._referee_seen = True
        gs = self.game_state
        ours = gs.kickoff_ours
        if gs.penalized:
            return 'penalized', ours
        if gs.state == GameState.FINISHED:
            return 'finished', ours
        if gs.stopped:
            return 'hold', ours
        if gs.state == GameState.PLAYING:
            # The opponent's kick-off or set play: the ball is not ours to
            # touch until the referee's countdown has run out.
            if gs.kicking_team_valid and not ours and gs.secondary_time > 0:
                return 'hold', ours
            return 'playing', ours
        return ('initial', 'ready', 'set')[gs.state], ours

    def _on_game_change(self, previous, game):
        self.get_logger().info(f'game: {previous} -> {game}')
        if previous == 'initial' and game == 'ready' and self.reset_on_ready != 'none':
            self.reset_pub.publish(String(data=self.reset_on_ready))
        elif previous == 'penalized' and game != 'penalized' \
                and self.reset_on_unpenalize != 'none':
            self.reset_pub.publish(String(data=self.reset_on_unpenalize))

    def _tick(self):
        now = self._now()

        if self.autostart and self.motion_state == 'IDLE' and now - self._stand_requested_at > 5.0:
            self._stand_requested_at = now
            self.motion_cmd_pub.publish(String(data='stand'))

        wm_fresh = self.wm_time is not None and now - self.wm_time <= self.world_model_timeout
        if not wm_fresh:
            # No state estimate, no motion: stop and let the watchdogs hold.
            self.cmd_pub.publish(Twist())
            self._report('no_world_model')
            return

        game, kickoff_ours = self._game(now)
        if game != self._last_game:
            if self._last_game is not None:
                self._on_game_change(self._last_game, game)
            self._last_game = game

        wm = self.wm
        pose = (wm.pose.x, wm.pose.y, wm.pose.theta)
        role, ball, position = self._team_view(now, wm, game)
        if role != self.role:
            self.get_logger().info(f'role: {self.role} -> {role}')
            self.role = role
        self.role_pub.publish(String(data=role))

        ball_known, ball_robot = wm.ball_known, (wm.ball_robot.x, wm.ball_robot.y)
        ball_field = (wm.ball_field.x, wm.ball_field.y)
        if not ball_known and ball is not None and wm.localized:
            # A teammate sees the ball: use it (not "visible", so no kicking on it).
            ball_known, ball_field = True, ball
            ball_robot = to_robot(pose, ball)
        bb = self.striker.tick(
            now=now,
            motion_state=self.motion_state,
            game=game,
            kickoff_ours=kickoff_ours,
            pose=pose,
            localized=wm.localized,
            ball_known=ball_known,
            ball_visible=wm.ball_visible,
            ball_robot=ball_robot,
            ball_field=ball_field,
            role=STRIKER if position is None else role,
            position=position)

        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.angular.z = bb.cmd
        self.cmd_pub.publish(cmd)
        if bb.head_mode != self._last_head_mode or int(now * 10) % 10 == 0:
            self.head_pub.publish(String(data=bb.head_mode))
            self._last_head_mode = bb.head_mode
        if bb.kick is not None:
            self.get_logger().info(
                f'Kick {bb.kick}: ball at ({wm.ball_robot.x:.2f}, {wm.ball_robot.y:.2f}) m, '
                f'pose ({wm.pose.x:.2f}, {wm.pose.y:.2f}, {math.degrees(wm.pose.theta):.0f} deg)')
            self.kick_pub.publish(String(data=bb.kick))
        self._report(bb.active_leaf)

    def _report(self, leaf):
        if leaf != self._last_leaf:
            self.get_logger().info(f'behavior: {leaf}')
            self._last_leaf = leaf
        self.state_pub.publish(String(data=leaf))


def main():
    rclpy.init()
    node = BehaviorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
