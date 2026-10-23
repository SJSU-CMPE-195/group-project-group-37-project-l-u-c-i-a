"""
motion.py

Pure math for turning ROS-style drive commands into Roomba wheel speeds, and
for ramping speed changes so the robot never jolts.
Kept free of ROS and serial imports so it can be tested on any machine.
"""

import math


def twist_to_wheel_speeds(linear_mps, angular_rad_s, wheel_base_mm, max_speed_mm_s):
    """
    Convert a forward speed + turn rate into left/right wheel speeds.

    Args:
        linear_mps:     forward speed in m/s (negative = backward)
        angular_rad_s:  turn rate in rad/s (positive = counter-clockwise, i.e. left)
        wheel_base_mm:  distance between the left and right wheels in mm
        max_speed_mm_s: fastest either wheel is allowed to spin, in mm/s

    Returns:
        (left, right) wheel speeds in mm/s as ints. If either wheel would go
        faster than max_speed_mm_s, both are scaled down together so the robot
        follows the same curve, just slower. NaN/inf input returns (0, 0).
    """
    if not (math.isfinite(linear_mps) and math.isfinite(angular_rad_s)):
        return 0, 0

    forward = linear_mps * 1000.0                # m/s -> mm/s
    turn = angular_rad_s * wheel_base_mm / 2.0   # extra speed on one side, less on the other
    left = forward - turn
    right = forward + turn

    fastest = max(abs(left), abs(right))
    if fastest > max_speed_mm_s:
        scale = max_speed_mm_s / fastest
        left *= scale
        right *= scale

    return int(round(left)), int(round(right))


def ramp_twist(current, target, limits, dt_s):
    """
    One step of a gradual change of several speeds at once (forward speed and
    turn rate), so the robot never jolts. Each speed may change by at most its
    accel * dt_s when speeding up, or decel * dt_s when slowing down, and all of
    them move the same share of the way to their targets. That keeps the robot
    on the curve it was told to follow while it speeds up or slows down: eased
    separately, the turn rate (which changes faster) would stop before the
    forward speed does, and the robot would run straight past the curve.

    current, target: tuples of speeds (m/s, rad/s); limits: one (accel, decel)
    per speed. Speeding up = moving away from zero; anything toward zero is
    slowing down. A speed that reverses stops at exactly 0 first (one step
    can't cross zero), then speeds up the other way. Returns a tuple.
    """
    target = tuple(t if math.isfinite(t) else 0.0 for t in target)
    if dt_s <= 0:
        return tuple(current)
    share = 1.0
    for c, t, (accel, decel) in zip(current, target, limits):
        change = abs(t - c)
        if change == 0:
            continue
        speeding_up = c * t >= 0 and abs(t) > abs(c)
        share = min(share, (accel if speeding_up else decel) * dt_s / change)
        if c * t < 0:                       # reversing: reach 0 this step at the latest
            share = min(share, abs(c) / change)
    if share >= 1.0 - 1e-9:
        return target
    new = []
    for c, t in zip(current, target):
        n = c + share * (t - c)
        if c * n < 0 or abs(n) < 1e-12:     # rounding just past zero
            n = 0.0
        new.append(n)
    return tuple(new)


def ramp_toward(current, target, accel, decel, dt_s):
    """ramp_twist for a single speed: one step from current toward target."""
    return ramp_twist((current,), (target,), ((accel, decel),), dt_s)[0]
