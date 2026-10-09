"""
odometry.py

Turns left/right wheel speeds into an estimated pose (x, y, heading) using
standard differential-drive dead reckoning. Pure math, no ROS, so it can be
tested without a robot.

This works the same whether the wheel speeds it's given are real encoder
measurements or just commanded speeds (an "open-loop" estimate, assuming the
wheels did what they were told) - the caller decides which by what it passes
in. Right now the Roomba's sensor replies don't reach the Pi (see
docs/changelogs/2026-10-09.md), so roomba_bridge.py currently feeds this
COMMANDED speeds, not measured ones. When that's fixed, swap in real encoder
speeds with no change needed here - the math is identical either way.

Known limitation of the open-loop case: it has no way to notice wheel slip,
stalling, or a collision, so it will drift more than real encoder feedback
would. It's still a better motion estimate than assuming the robot never
moves, which is the only other option available right now.
"""

import math


class Odometry:
    def __init__(self, wheel_base_mm):
        if wheel_base_mm <= 0:
            raise ValueError('wheel_base_mm must be greater than 0')
        self.wheel_base_mm = wheel_base_mm
        self.x_mm = 0.0
        self.y_mm = 0.0
        self.heading_rad = 0.0

    def update(self, left_mm_s, right_mm_s, dt_s):
        """
        Advance the pose estimate, given wheel speeds held for dt_s seconds.

        Integrates along the arc the robot actually follows (a circle, or a
        straight line for zero turn rate), rather than a straight-line step,
        so a steady turn traces a curve instead of a jagged polygon.
        """
        if dt_s <= 0:
            return

        forward_mm_s = (left_mm_s + right_mm_s) / 2.0
        turn_rad_s = (right_mm_s - left_mm_s) / self.wheel_base_mm
        dtheta = turn_rad_s * dt_s

        if abs(turn_rad_s) < 1e-9:  # straight line - avoid dividing by ~0 below
            dx = forward_mm_s * dt_s * math.cos(self.heading_rad)
            dy = forward_mm_s * dt_s * math.sin(self.heading_rad)
        else:
            radius_mm = forward_mm_s / turn_rad_s
            dx = radius_mm * (math.sin(self.heading_rad + dtheta) - math.sin(self.heading_rad))
            dy = -radius_mm * (math.cos(self.heading_rad + dtheta) - math.cos(self.heading_rad))

        self.x_mm += dx
        self.y_mm += dy
        self.heading_rad = _wrap_angle(self.heading_rad + dtheta)

    def pose(self):
        """Return (x_mm, y_mm, heading_rad)."""
        return self.x_mm, self.y_mm, self.heading_rad


def _wrap_angle(angle_rad):
    """Keep an angle in (-pi, pi] so it doesn't grow without bound over time."""
    return (angle_rad + math.pi) % (2 * math.pi) - math.pi
