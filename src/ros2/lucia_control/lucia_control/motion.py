"""
motion.py

Pure math for turning ROS-style drive commands into Roomba wheel speeds.
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
