"""Team communication over UDP broadcast (WiFi between the robots).

Every robot broadcasts a small JSON packet a few times per second and listens
on the same port. Packets of other teams (different team number or magic) and
our own echoes are dropped.

Inputs:
  /world_model      aroc_interfaces/WorldModel   own pose and ball
  /behavior/role    std_msgs/String              own role (from the behavior node)
  /motion/status    std_msgs/String              READY/WALKING/KICKING = active
  /game_state       aroc_interfaces/GameState    penalized = inactive
Output:
  /team/state       aroc_interfaces/TeamState    latest message of each teammate

Parameters: team_number, player_number (shared with game_controller, set in
soccer.yaml), team_comm.port (0 = 10000 + team_number), team_comm.address
(broadcast address of the robots' network), team_comm.rate_hz.
"""

import random
import socket

import rclpy
from aroc_interfaces.msg import GameState, TeamMate, TeamState, WorldModel
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from ..core.team_packet import ACTIVE_MOTION, decode, encode

class TeamComm(Node):

    def __init__(self):
        super().__init__('team_comm')
        p = self.declare_parameter
        self.team = int(p('team_number', 1).value)
        self.player = int(p('player_number', 1).value)
        port = int(p('team_comm.port', 0).value) or 10000 + self.team
        self.address = (p('team_comm.address', '255.255.255.255').value, port)
        self.timeout = p('team_comm.mate_timeout', 3.0).value

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if hasattr(socket, 'SO_REUSEPORT'):
            # Several simulated robots on one machine share the port.
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind(('', port))
        self.sock.setblocking(False)

        self.wm = None
        self.role = ''
        self.motion = 'IDLE'
        self.penalized = False
        self.seq = 0
        self.sid = random.getrandbits(31)
        self.mates = {}                 # player -> (packet, receive time)
        self._send_errors = 0

        self.pub = self.create_publisher(TeamState, '/team/state', 10)
        self.create_subscription(WorldModel, '/world_model', self._on_wm, 10)
        self.create_subscription(String, '/behavior/role', self._on_role, 10)
        self.create_subscription(String, '/motion/status', self._on_motion, 10)
        self.create_subscription(GameState, '/game_state', self._on_game, 10)
        self.create_timer(1.0 / p('team_comm.rate_hz', 3.0).value, self._send)
        self.create_timer(0.05, self._receive)
        self.create_timer(0.2, self._publish)
        self.get_logger().info(
            f'team {self.team} player {self.player}: UDP {self.address[0]}:{port}')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_wm(self, msg):
        self.wm = msg

    def _on_role(self, msg):
        self.role = msg.data

    def _on_motion(self, msg):
        self.motion = msg.data

    def _on_game(self, msg):
        self.penalized = msg.penalized

    def _send(self):
        if self.wm is None:
            return
        self.seq += 1
        active = self.motion in ACTIVE_MOTION and not self.penalized
        data = encode(self.team, self.player, self.wm, self.role, active, self.seq, self.sid)
        try:
            self.sock.sendto(data, self.address)
        except OSError as err:
            self._send_errors += 1
            if self._send_errors in (1, 10, 100):
                self.get_logger().warn(f'team_comm send failed ({err}); no network?')

    def _receive(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(2048)
            except (BlockingIOError, InterruptedError):
                return
            except OSError:
                return
            msg = decode(data, self.team)
            if msg is None:
                continue
            if msg['p'] == self.player:
                if msg.get('id', self.sid) != self.sid:
                    self.get_logger().error(
                        f'another robot also uses player_number {self.player}: '
                        'set a different player_number on each robot',
                        throttle_duration_sec=5.0)
                continue
            self.mates[msg['p']] = (msg, self._now())

    def _publish(self):
        now = self._now()
        out = TeamState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'field'
        for player, (msg, t) in sorted(self.mates.items()):
            age = now - t
            if age > self.timeout:
                continue
            mate = TeamMate()
            mate.player = player
            mate.pose.x, mate.pose.y, mate.pose.theta = msg['pose']
            mate.localized = bool(msg.get('loc', False))
            if 'ball' in msg:
                mate.ball_known = True
                mate.ball.x, mate.ball.y = msg['ball']
                mate.ball_age = float(msg.get('bage', 0.0))
            mate.role = str(msg.get('role', ''))
            mate.active = bool(msg.get('act', False))
            mate.age = float(age)
            out.mates.append(mate)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = TeamComm()
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
