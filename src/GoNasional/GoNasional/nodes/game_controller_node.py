"""GameController bridge (RoboCup HSL protocol v20).

Listens for the referee's UDP broadcast on port 3838, publishes this robot's
view of it on /game_state, and answers the GameController on port 3939 twice a
second with pose, ball and fallen state so the referee sees the robot alive.

The socket is non-blocking and polled from a timer; no thread, no blocking
call. If the GameController goes silent the node stops publishing and the
behavior falls back to the manual start button.
"""

import socket

import rclpy
from aroc_interfaces.msg import GameState, WorldModel
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from ..core import gamecontroller as gc


class GameControllerNode(Node):

    def __init__(self):
        super().__init__('game_controller')
        p = self.declare_parameter
        self.team_number = p('team_number', 1).value
        self.player_number = p('player_number', 1).value
        self.listen_port = p('listen_port', gc.DATA_PORT).value
        self.return_port = p('return_port', gc.RETURN_PORT).value
        self.silence_timeout = p('silence_timeout', 5.0).value

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind(('', self.listen_port))
        self.sock.setblocking(False)

        self.referee_address = None
        self.last_packet_time = None
        self.wm = None
        self.wm_time = None
        self.motion_state = 'IDLE'
        self._last_summary = None

        self.pub = self.create_publisher(GameState, '/game_state', 10)
        self.create_subscription(WorldModel, '/world_model', self._on_world_model, 10)
        self.create_subscription(String, '/motion/status', self._on_motion_status, 10)
        self.create_timer(0.02, self._poll)
        self.create_timer(0.5, self._send_status)
        self.get_logger().info(
            f'Listening for GameController on UDP {self.listen_port} as team '
            f'{self.team_number}, player {self.player_number}')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_world_model(self, msg):
        self.wm = msg
        self.wm_time = self._now()

    def _on_motion_status(self, msg):
        self.motion_state = msg.data

    def _poll(self):
        latest = None
        # Drain the socket and keep only the newest valid packet.
        for _ in range(20):
            try:
                data, address = self.sock.recvfrom(1024)
            except BlockingIOError:
                break
            except OSError as error:
                self.get_logger().warn(f'UDP receive failed: {error}',
                                       throttle_duration_sec=5.0)
                break
            control = gc.parse_control(data)
            if control is None:
                self.get_logger().warn(
                    f'Ignoring {len(data)}-byte packet from {address[0]}: not a '
                    f'GameController v{gc.VERSION} control message',
                    throttle_duration_sec=5.0)
                continue
            if control.team(self.team_number) is None:
                continue        # a game on another field
            latest = (control, address)
        if latest is None:
            if (self.last_packet_time is not None
                    and self._now() - self.last_packet_time > self.silence_timeout):
                self.get_logger().warn('GameController went silent')
                self.last_packet_time = None
                self.referee_address = None
            return
        control, address = latest
        self.referee_address = address[0]
        self.last_packet_time = self._now()
        self._publish(control)

    def _publish(self, control):
        own = control.team(self.team_number)
        opponent = control.opponent(self.team_number)
        index = self.player_number - 1
        player = own.players[index] if 0 <= index < len(own.players) else gc.PlayerInfo()

        msg = GameState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = control.state
        msg.game_phase = control.game_phase
        msg.set_play = control.set_play
        msg.stopped = control.stopped
        msg.first_half = control.first_half
        msg.secs_remaining = control.secs_remaining
        msg.secondary_time = control.secondary_time
        msg.kicking_team_valid = control.kicking_team != gc.KICKING_TEAM_NONE
        msg.kickoff_ours = control.kicking_team == self.team_number
        msg.penalized = player.penalty != gc.PENALTY_NONE
        msg.penalty = player.penalty
        msg.secs_till_unpenalized = player.secs_till_unpenalized
        msg.is_goalkeeper = own.goalkeeper == self.player_number
        msg.own_score = own.score
        msg.opponent_score = opponent.score if opponent is not None else 0
        msg.from_game_controller = True
        self.pub.publish(msg)

        summary = (control.state, control.stopped, msg.penalized, control.set_play,
                   msg.kickoff_ours, msg.own_score, msg.opponent_score)
        if summary != self._last_summary:
            self._last_summary = summary
            names = ('INITIAL', 'READY', 'SET', 'PLAYING', 'FINISHED')
            self.get_logger().info(
                f'Referee: {names[control.state]}'
                f'{" STOPPED" if control.stopped else ""}'
                f'{" PENALIZED(" + str(player.penalty) + ")" if msg.penalized else ""}'
                f' set_play={control.set_play} kick_ours={msg.kickoff_ours}'
                f' score {msg.own_score}:{msg.opponent_score}')

    def _send_status(self):
        if self.referee_address is None:
            return
        fresh = self.wm_time is not None and self._now() - self.wm_time < 1.0
        pose, ball_age, ball = (0.0, 0.0, 0.0), -1.0, (0.0, 0.0)
        if fresh:
            wm = self.wm
            pose = (wm.pose.x, wm.pose.y, wm.pose.theta)
            if wm.ball_known:
                ball_age = max(0.0, wm.ball_age)
                ball = (wm.ball_robot.x, wm.ball_robot.y)
        packet = gc.pack_status(
            self.team_number, self.player_number,
            fallen=self.motion_state == 'GETTING_UP',
            pose=pose, ball_age=ball_age, ball=ball)
        try:
            self.sock.sendto(packet, (self.referee_address, self.return_port))
        except OSError as error:
            self.get_logger().warn(f'UDP send failed: {error}', throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = GameControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
