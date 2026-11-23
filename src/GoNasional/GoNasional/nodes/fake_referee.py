"""Minimal stand-in for the GameController, for tests without the GUI.

    ros2 run GoNasional fake_referee --team 1 --state ready --kickoff ours
    ros2 run GoNasional fake_referee --team 1 --state playing --penalize 1 --duration 10

Broadcasts v20 control packets at 2 Hz on UDP 3838 and prints the status
packets robots send back on 3939. Not a referee: it sends one fixed state.
"""

import argparse
import socket
import struct
import time

from ..core import gamecontroller as gc

STATES = {'initial': 0, 'ready': 1, 'set': 2, 'playing': 3, 'finished': 4}


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--team', type=int, default=1)
    parser.add_argument('--opponent', type=int, default=2)
    parser.add_argument('--state', choices=STATES, default='initial')
    parser.add_argument('--kickoff', choices=('ours', 'theirs', 'none'), default='none')
    parser.add_argument('--secondary-time', type=int, default=0)
    parser.add_argument('--stopped', action='store_true')
    parser.add_argument('--penalize', type=int, default=0, help='player number to penalize')
    parser.add_argument('--duration', type=float, default=0.0, help='seconds, 0 = forever')
    parser.add_argument('--address', default='127.0.0.1')
    args = parser.parse_args()

    control = gc.ControlData(state=STATES[args.state], stopped=args.stopped,
                             secondary_time=args.secondary_time)
    control.teams[0].number, control.teams[1].number = args.team, args.opponent
    control.kicking_team = {'ours': args.team, 'theirs': args.opponent,
                            'none': gc.KICKING_TEAM_NONE}[args.kickoff]
    if args.penalize:
        control.teams[0].players[args.penalize - 1].penalty = 6     # picked up
        control.teams[0].players[args.penalize - 1].secs_till_unpenalized = 45

    send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    send.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    receive = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receive.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    receive.bind(('', gc.RETURN_PORT))
    receive.setblocking(False)

    start = time.monotonic()
    try:
        while not args.duration or time.monotonic() - start < args.duration:
            control.packet_number = (control.packet_number + 1) & 0xFF
            send.sendto(gc.pack_control(control), (args.address, gc.DATA_PORT))
            try:
                data, address = receive.recvfrom(256)
                if len(data) == gc.RETURN_SIZE and data[:4] == gc.RETURN_HEADER:
                    (_, version, player, team, fallen, x, y, theta, ball_age,
                     bx, by) = struct.unpack('<4s4B3ff2f', data)
                    print(f'status from {address[0]}: team {team} player {player} '
                          f'fallen={fallen} pose=({x:.0f} mm, {y:.0f} mm, {theta:.2f} rad) '
                          f'ball_age={ball_age:.1f}s ball=({bx:.0f}, {by:.0f}) mm', flush=True)
            except BlockingIOError:
                pass
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
