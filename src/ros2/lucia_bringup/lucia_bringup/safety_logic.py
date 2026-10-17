"""
safety_logic.py

The safety gate's rules, without ROS, so they can be tested anywhere. See
safety_gate.py for the node.
"""

import math


def tof_blocked(near, age_s, stale_s):
    """
    True if forward driving must stop: the ToF sensors see something close
    (lucia_tof's tof/near), or their signal is missing or older than stale_s.
    No news counts as danger, the same rule lucia_tof itself uses.
    """
    if near is None or age_s is None or age_s > stale_s:
        return True
    return bool(near)


def gate(linear, angular, blocked):
    """
    The drive command to pass on. While blocked, forward speed becomes 0;
    turning in place and backing up still go through, so the robot (or Nav2's
    recoveries) can turn away from what's in front of it.
    """
    if blocked and linear > 0.0:
        return 0.0, angular
    return linear, angular


# ----------------------------------------------------------------------------
# Tight spaces: full speed in the open, slower close to things (from the lidar)
# ----------------------------------------------------------------------------

def scan_points(ranges, angle_min, angle_increment, laser_pose, min_range=0.0, max_range=3.0):
    """Lidar ranges -> (x, y) points in the robot's frame (x forward, y left). laser_pose: (x, y, yaw) of the lidar on the robot."""
    lx, ly, lyaw = laser_pose
    points = []
    for i, r in enumerate(ranges):
        if math.isfinite(r) and min_range <= r <= max_range:
            a = lyaw + angle_min + i * angle_increment
            points.append((lx + r * math.cos(a), ly + r * math.sin(a)))
    return points


def clearances(points, robot_radius, lane_margin=0.05):
    """
    How much room the robot has, measured from its edge (meters):
    - near: the closest thing beside or in front of it (behind doesn't matter
      driving forward). Small in a doorway, a narrow gap, or hugging a wall.
    - ahead: free run straight ahead, in a lane as wide as the robot plus
      lane_margin each side.
    Points inside the robot's own outline (its arm) are ignored. inf = nothing seen.
    """
    near = ahead = math.inf
    for x, y in points:
        d = math.hypot(x, y)
        if d <= robot_radius:
            continue
        if x > -robot_radius:
            near = min(near, d - robot_radius)
        if x > 0.0 and abs(y) <= robot_radius + lane_margin:
            ahead = min(ahead, x - robot_radius)
    return near, ahead


def _ramp(value, slow_at, fast_at):
    """0 at or below slow_at, 1 at or above fast_at, straight line between."""
    if value <= slow_at:
        return 0.0
    if value >= fast_at:
        return 1.0
    return (value - slow_at) / (fast_at - slow_at)


def tight_space_cap(near, ahead, min_speed, max_speed,
                    near_slow=0.05, near_fast=0.35, ahead_slow=0.2, ahead_fast=0.8):
    """
    Fastest forward speed allowed with this much room (m/s): max_speed with
    near_fast of room around the front half and ahead_fast of free run ahead,
    down to min_speed at near_slow / ahead_slow, in a straight line between.
    Never 0: stopping is the ToF stop's and Nav2's job, not this one's.
    """
    room = min(_ramp(near, near_slow, near_fast), _ramp(ahead, ahead_slow, ahead_fast))
    return min_speed + (max_speed - min_speed) * room


def limit_forward(linear, angular, cap):
    """
    Slow forward driving down to cap, turning in proportion so the robot
    follows the same curve. Backing up and turning in place pass unchanged.
    """
    if linear <= cap:
        return linear, angular
    scale = cap / linear
    return cap, angular * scale
