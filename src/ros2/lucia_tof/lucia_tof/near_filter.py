"""Is something close to one ToF sensor? Decided from its frames, with two
guards against glitches. No ROS imports, so it can be tested on its own.

Seen on the robot (2026-11-10): the middle sensor reported 0 mm, flagged near,
with nothing in front of it, in 2 of 20 looks over 10 s. With the ToF safety
stop on, each would stop the robot for nothing. So:

- Readings closer than the sensor can measure (min_mm, about 2 cm) are dropped,
  unless many zones read that close at once (COVERED_ZONES of 16): then
  something really is pressed against the sensor, and that's near.
- A sensor's answer changes only after `frames` frames in a row agree, so one
  odd frame can't stop the robot, or let it go. At 60 frames/s, 3 frames add
  50 ms to a stop (1.5 cm at 30 cm/s).

What counts as near for one frame is up to the caller: tof_node asks
zones.in_the_way (something in the robot's path, within the stop distance);
without that, a reading under near_mm. The Pico's own near flag (FLAG_NEAR,
NEAR_MM in src/pico2/main.py) isn't used.
"""

INVALID = -1

# This many zones (of 16) under min_mm in one frame = something against the sensor.
COVERED_ZONES = 8


def too_close(distances, min_mm):
    """How many zones read closer than the sensor can measure (0 <= d < min_mm)."""
    return sum(1 for d in distances if 0 <= d < min_mm)


def nearest(distances, min_mm):
    """Smallest believable distance in mm, or INVALID (-1) if no zone has one.
    Zones the Pico marked invalid are -1."""
    good = [d for d in distances if d >= min_mm]
    return min(good) if good else INVALID


class NearFilter:
    """One sensor's near / clear answer. Starts as near: nothing heard yet
    counts as danger, like a silent sensor."""

    def __init__(self, near_mm=300, frames=3, min_mm=20):
        self.near_mm, self.frames, self.min_mm = near_mm, frames, min_mm
        self.reset()

    def reset(self):
        self.near = True
        self._against = 0       # frames in a row that disagree with self.near

    def update(self, distances, frame_near=None):
        """Feed one frame's 16 distances (mm, -1 = invalid), and optionally
        whether this frame has something in the way (decided by the caller).
        Returns (nearest_mm, near): nearest_mm is 0 when the sensor is covered,
        INVALID when nothing is in range."""
        covered = too_close(distances, self.min_mm) >= COVERED_ZONES
        nearest_mm = 0 if covered else nearest(distances, self.min_mm)
        if covered:
            frame_near = True       # can't see: count it as danger
        elif frame_near is None:
            frame_near = 0 <= nearest_mm < self.near_mm
        if frame_near == self.near:
            self._against = 0
        else:
            self._against += 1
            if self._against >= self.frames:
                self.near = frame_near
                self._against = 0
        return nearest_mm, self.near
