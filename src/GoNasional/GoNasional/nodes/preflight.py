"""Pre-match check: is everything this robot needs running and sane?

    ros2 run GoNasional preflight            # on the robot, after all launches
    ros2 run GoNasional preflight --sim      # in Webots (no USB camera topic)

Listens for a few seconds, then prints PASS / WARN / FAIL per item and exits
with status 1 if anything failed. It only reads topics and transforms; it
never moves the robot.
"""

import argparse
import math
import sys
import time

import rclpy
from aroc_interfaces.msg import DetectionArray, GameState, TeamState, WorldModel
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, Imu, JointState
from std_msgs.msg import Float64, String
from tf2_ros import Buffer, TransformException, TransformListener

OP3_JOINTS = [
    'r_sho_pitch', 'l_sho_pitch', 'r_sho_roll', 'l_sho_roll', 'r_el', 'l_el',
    'r_hip_yaw', 'l_hip_yaw', 'r_hip_roll', 'l_hip_roll', 'r_hip_pitch', 'l_hip_pitch',
    'r_knee', 'l_knee', 'r_ank_pitch', 'l_ank_pitch', 'r_ank_roll', 'l_ank_roll',
    'head_pan', 'head_tilt']
STATE_NAMES = ('TRACKING', 'UNCERTAIN', 'LOST')


class Watch:
    def __init__(self):
        self.count = 0
        self.last = None
        self.acc_sum = [0.0, 0.0, 0.0]

    def __call__(self, msg):
        self.count += 1
        self.last = msg
        acc = getattr(msg, 'linear_acceleration', None)
        if acc is not None:
            # Averaged: single samples swing with every step when walking.
            self.acc_sum = [self.acc_sum[0] + acc.x, self.acc_sum[1] + acc.y,
                            self.acc_sum[2] + acc.z]


def main():
    parser = argparse.ArgumentParser(description='Cek pra-pertandingan robot OP3.')
    parser.add_argument('--sim', action='store_true', help='simulasi Webots')
    parser.add_argument('--duration', type=float, default=5.0)
    parser.add_argument('--image-topic', default=None)
    args, ros_args = parser.parse_known_args()
    image_topic = args.image_topic or (
        '/robotis_op3/camera/image_raw' if args.sim else '/usb_cam_node/image_raw')

    rclpy.init(args=ros_args)
    node = rclpy.create_node('aroc_preflight')
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)
    topics = {
        'joints': ('/robotis/present_joint_states', JointState, 50.0, True),
        'imu': ('/robotis/open_cr/imu', Imu, 40.0, True),
        'yaw': ('/robotis/open_cr/yaw', Float64, 10.0, True),
        'camera': (image_topic, Image, 15.0, True),
        # In Webots with vision:=gt there is no detector; only YOLO mode has it.
        'detections': ('/vision/detections', DetectionArray, 5.0, not args.sim),
        'world_model': ('/world_model', WorldModel, 10.0, True),
        'motion': ('/motion/status', String, 5.0, True),
        'game_state': ('/game_state', GameState, 1.0, False),
        'team': ('/team/state', TeamState, 2.0, False),
    }
    watches = {}
    for key, (topic, msg_type, _, _) in topics.items():
        watches[key] = Watch()
        qos = qos_profile_sensor_data if key in ('imu', 'yaw', 'camera') else 10
        node.create_subscription(msg_type, topic, watches[key], qos)

    end = time.monotonic() + args.duration
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    results = []

    def report(level, item, detail):
        results.append((level, item, detail))

    for key, (topic, _, min_rate, required) in topics.items():
        rate = watches[key].count / args.duration
        if rate >= min_rate:
            report('PASS', topic, f'{rate:.1f} Hz')
        elif rate > 0.0:
            report('WARN', topic, f'{rate:.1f} Hz (< {min_rate:.0f} Hz)')
        else:
            report('FAIL' if required else 'WARN', topic,
                   'tidak ada pesan' + ('' if required else ' (opsional)'))

    js = watches['joints'].last
    if js is not None:
        pos = dict(zip(js.name, js.position))
        missing = [j for j in OP3_JOINTS if j not in pos]
        bad = [j for j in OP3_JOINTS if j in pos and not math.isfinite(pos[j])]
        if missing or bad:
            report('FAIL', 'joint', f'hilang {missing} / NaN {bad}')
        else:
            report('PASS', 'joint', '20 sendi lengkap, tanpa NaN')

    if watches['imu'].count:
        ax, ay, az = (v / watches['imu'].count for v in watches['imu'].acc_sum)
        norm = math.sqrt(ax ** 2 + ay ** 2 + az ** 2)
        if norm < 1e-6:
            report('FAIL', 'imu', 'akselerasi nol (IMU tidak terbaca / modul sim)')
        else:
            tilt = math.degrees(math.acos(min(1.0, abs(az) / norm)))
            report('PASS' if tilt < 20.0 else 'WARN', 'imu',
                   f'kemiringan {tilt:.0f} deg dari tegak (robot harus berdiri tegak saat cek)')

    try:
        tf = tf_buffer.lookup_transform('body_link', 'cam_link', Time())
        age = node.get_clock().now().nanoseconds * 1e-9 - \
            Time.from_msg(tf.header.stamp).nanoseconds * 1e-9
        report('PASS' if age < 0.5 else 'WARN', 'tf body_link->cam_link',
               f'umur {age:.2f} s' + ('' if age < 0.5 else ' (joint_states tersendat?)'))
    except TransformException as err:
        report('FAIL', 'tf body_link->cam_link',
               f'{err.__class__.__name__}: robot_state_publisher jalan? (urdf:=true)')

    det = watches['detections'].last
    if det is not None:
        labels = sorted({d.label for d in det.detections})
        report('PASS', 'model YOLO', f'frame {det.width}x{det.height}, terlihat: {labels or "-"}')

    wm = watches['world_model'].last
    if wm is not None:
        state = STATE_NAMES[wm.localization_state] if wm.localization_state < 3 else '?'
        detail = (f'{state}, pose ({wm.pose.x:.2f}, {wm.pose.y:.2f}, '
                  f'{math.degrees(wm.pose.theta):.0f} deg), std {wm.pose_std_xy:.2f} m')
        report('PASS' if wm.localized else 'WARN', 'lokalisasi', detail)
        report('PASS' if wm.ball_known else 'WARN', 'bola',
               'terlihat' if wm.ball_visible else ('diketahui' if wm.ball_known else
                                                   'belum terlihat (taruh bola di depan robot)'))

    motion = watches['motion'].last
    if motion is not None:
        report('PASS' if motion.data in ('READY', 'WALKING') else 'WARN', 'motion',
               motion.data)

    gs = watches['game_state'].last
    if gs is not None:
        report('PASS' if gs.from_game_controller else 'WARN', 'GameController',
               'paket diterima' if gs.from_game_controller else 'mode manual (tanpa GC)')

    team = watches['team'].last
    if team is not None:
        mates = ', '.join(f'#{m.player} {m.role or "-"}' for m in team.mates) or 'tidak ada'
        report('PASS' if team.mates else 'WARN', 'tim', f'teman terdengar: {mates}')

    width = max(len(item) for _, item, _ in results)
    for level, item, detail in results:
        print(f'[{level}] {item:<{width}}  {detail}')
    failed = sum(1 for level, _, _ in results if level == 'FAIL')
    warned = sum(1 for level, _, _ in results if level == 'WARN')
    print(f'\n{failed} FAIL, {warned} WARN. '
          + ('Siap main.' if not failed else 'Perbaiki FAIL sebelum bertanding.'))
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
