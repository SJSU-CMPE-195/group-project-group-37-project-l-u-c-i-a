"""Tests for song.py. Run from src/ros2/lucia_control:  python3 -m pytest test"""

import pytest

from lucia_control.song import (MAX_NOTES, REST, clean_tune, seconds,
                                slot_start_times, slots)


def test_clean_tune_keeps_playable_notes():
    assert clean_tune([(60, 16), [72, 32]]) == [(60, 16), (72, 32)]


def test_unplayable_notes_become_rests_and_durations_are_clamped():
    assert clean_tune([(20, 16), (200, 8), (60, 0), (60, 999)]) == [
        (REST, 16), (REST, 8), (60, 1), (60, 255)]


@pytest.mark.parametrize('bad', [[], None, [(60,)], [('C4', 16)], 'C4 D4'])
def test_clean_tune_rejects(bad):
    with pytest.raises(ValueError):
        clean_tune(bad)


def test_too_long_is_rejected():
    with pytest.raises(ValueError):
        clean_tune([(60, 8)] * (MAX_NOTES + 1))


def test_slots_split_into_16s():
    tune = [(60 + i % 12, 8) for i in range(40)]
    out = slots(tune)
    assert [s for s, _ in out] == [0, 1, 2]
    assert [len(p) for _, p in out] == [16, 16, 8]
    assert [n for _, p in out for n in p] == clean_tune(tune)


def test_slot_start_times_follow_each_other():
    tune = [(60, 32)] * 20                  # 16 notes of 0.5 s, then 4
    (s0, t0), (s1, t1) = slot_start_times(tune, gap_s=0.05)
    assert (s0, t0) == (0, 0.0)
    assert s1 == 1 and t1 == pytest.approx(8.0 + 0.05)


def test_seconds():
    assert seconds([(60, 64), (0, 32)]) == 1.5
