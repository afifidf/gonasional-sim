"""Team packet format (JSON over UDP). No ROS imports, so it can be unit tested."""

import json
import math

MAGIC = 'AROC1'
ACTIVE_MOTION = ('READY', 'WALKING', 'KICKING')


def encode(team, player, wm, role, active, seq, sid=0):
    """sid: random id of the sending process; tells two robots with the same number apart."""
    msg = {'m': MAGIC, 'team': team, 'p': player, 's': seq, 'id': sid,
           'pose': [round(wm.pose.x, 3), round(wm.pose.y, 3), round(wm.pose.theta, 3)],
           'loc': bool(wm.localized), 'role': role, 'act': bool(active)}
    if wm.ball_known:
        msg['ball'] = [round(wm.ball_field.x, 3), round(wm.ball_field.y, 3)]
        msg['bage'] = round(max(0.0, wm.ball_age), 2)
    return json.dumps(msg, separators=(',', ':')).encode()


def decode(data, team):
    """Parsed packet dict, or None if it is not ours / malformed."""
    try:
        msg = json.loads(data.decode())
        if msg.get('m') != MAGIC or msg.get('team') != team:
            return None
        msg['p'] = int(msg['p'])
        msg['pose'] = [float(v) for v in msg['pose']][:3]
        if len(msg['pose']) != 3 or not all(math.isfinite(v) for v in msg['pose']):
            return None
        if 'ball' in msg:
            msg['ball'] = [float(v) for v in msg['ball']][:2]
        return msg
    except (ValueError, KeyError, TypeError, UnicodeDecodeError, AttributeError):
        return None
