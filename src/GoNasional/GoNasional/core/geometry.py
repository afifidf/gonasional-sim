"""Small 2D helpers shared by the core modules."""

import math


def wrap(angle):
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value, low, high):
    return max(low, min(high, value))


def to_field(pose, point):
    """Robot-frame point (x fwd, y left) -> field frame, given pose (x, y, theta)."""
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    return (x + c * point[0] - s * point[1], y + s * point[0] + c * point[1])


def to_robot(pose, point):
    """Field-frame point -> robot frame, given pose (x, y, theta)."""
    x, y, th = pose
    dx, dy = point[0] - x, point[1] - y
    c, s = math.cos(th), math.sin(th)
    return (c * dx + s * dy, -s * dx + c * dy)
