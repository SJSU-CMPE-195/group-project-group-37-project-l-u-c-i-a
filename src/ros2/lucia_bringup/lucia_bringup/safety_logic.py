"""
safety_logic.py

The safety gate's rules, without ROS, so they can be tested anywhere. See
safety_gate.py for the node.
"""


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
