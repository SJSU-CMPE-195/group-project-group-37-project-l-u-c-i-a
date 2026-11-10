"""Tests for roomba_oi.py's byte encoding, with a fake serial port. Run from src/ros2/lucia_control:  python3 -m pytest test"""

import pytest

serial = pytest.importorskip('serial')      # roomba_oi imports pyserial

from lucia_control.roomba_oi import RoombaOI  # noqa: E402


class FakeSerial:
    def __init__(self):
        self.written = []

    def write(self, data):
        self.written.append(list(data))


def make_roomba():
    roomba = RoombaOI.__new__(RoombaOI)     # skip __init__: no real port
    roomba.ser = FakeSerial()
    return roomba


def test_beep_defines_and_plays_a_song():
    roomba = make_roomba()
    roomba.beep()
    assert roomba.ser.written == [[140, 3, 2, 72, 8, 79, 12], [141, 3]]


def test_start_and_safe_opcodes(monkeypatch):
    monkeypatch.setattr('time.sleep', lambda s: None)
    roomba = make_roomba()
    roomba.start()
    roomba.safe_mode()
    assert roomba.ser.written == [[128], [131]]


def test_define_and_play_song():
    roomba = make_roomba()
    roomba.define_song(1, [(60, 16), (0, 8)])
    roomba.play_song(1)
    assert roomba.ser.written == [[140, 1, 2, 60, 16, 0, 8], [141, 1]]


@pytest.mark.parametrize('slot, notes', [(4, [(60, 8)]), (0, []), (0, [(60, 8)] * 17)])
def test_define_song_rejects(slot, notes):
    with pytest.raises(ValueError):
        make_roomba().define_song(slot, notes)


class LineRecorder:
    """Fake serial port that records RTS/DTR changes, to check the wake pulse."""

    def __init__(self):
        self.changes = []

    def __setattr__(self, name, value):
        if name in ('rts', 'dtr'):
            self.changes.append((name, value))
        object.__setattr__(self, name, value)


def test_wake_pulses_rts_and_dtr_together(monkeypatch):
    # RTS reaches BRC on the new cable, DTR on the old one: one low pulse on both, then released
    sleeps = []
    monkeypatch.setattr('time.sleep', sleeps.append)
    roomba = RoombaOI.__new__(RoombaOI)
    roomba.ser = LineRecorder()
    roomba.wake()
    states = roomba.ser.changes
    assert [v for n, v in states if n == 'rts'] == [False, True, False]
    assert [v for n, v in states if n == 'dtr'] == [False, True, False]
    assert roomba.ser.rts is False and roomba.ser.dtr is False
    assert sleeps[:2] == [0.5, 0.5]
