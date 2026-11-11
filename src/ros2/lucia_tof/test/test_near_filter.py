from lucia_tof.near_filter import COVERED_ZONES, INVALID, NearFilter, nearest, too_close

CLEAR = [600] * 16


def with_zone(value, zone=5, base=CLEAR):
    d = list(base)
    d[zone] = value
    return d


def run(f, frames):
    return [f.update(d)[1] for d in frames]


def clear_filter():
    f = NearFilter(near_mm=300, frames=3, min_mm=20)
    run(f, [CLEAR] * 3)
    assert not f.near
    return f


def test_starts_near_until_three_clear_frames():
    f = NearFilter(near_mm=300, frames=3, min_mm=20)
    assert run(f, [CLEAR] * 3) == [True, True, False]


def test_zero_mm_glitch_is_ignored():
    # What the middle sensor did on the robot: one zone at 0 mm, nothing in front of it
    f = clear_filter()
    assert run(f, [with_zone(0)] * 10) == [False] * 10
    assert f.update(with_zone(0))[0] == 600


def test_one_odd_frame_cannot_stop_or_release_it():
    f = clear_filter()
    assert run(f, [with_zone(150), CLEAR, with_zone(150), with_zone(150), CLEAR]) == [False] * 5
    f = clear_filter()
    run(f, [with_zone(150)] * 3)
    assert f.near
    assert run(f, [CLEAR, with_zone(150), CLEAR, CLEAR]) == [True] * 4


def test_driving_toward_a_wall_stops_three_frames_after_300_mm():
    f = clear_filter()
    approach = [[d] * 16 for d in (420, 380, 340, 299, 280, 260, 240)]
    assert run(f, approach) == [False, False, False, False, False, True, True]


def test_covered_sensor_is_near():
    f = clear_filter()
    covered = [5] * COVERED_ZONES + [-1] * (16 - COVERED_ZONES)
    assert f.update(covered) == (0, False)
    assert run(f, [covered] * 2) == [False, True]


def test_nothing_in_range_is_clear():
    f = clear_filter()
    assert f.update([INVALID] * 16) == (INVALID, False)


def test_helpers():
    assert nearest([-1, 0, 19, 20, 500], 20) == 20
    assert nearest([-1, 0, 10], 20) == INVALID
    assert too_close([-1, 0, 19, 20], 20) == 2


def test_reset_goes_back_to_near():
    f = clear_filter()
    f.reset()
    assert f.near
