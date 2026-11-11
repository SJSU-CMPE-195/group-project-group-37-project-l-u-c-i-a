"""Tests for battery.py, and RoombaOI.query with a fake serial port."""

import struct

import pytest

from lucia_control import battery


def reply(state=0, mv=16224, ma=-350, temp=27, charge=2522, capacity=2697):
    return struct.pack('>BHhbHH', state, mv, ma, temp, charge, capacity)


def test_query_asks_for_the_six_battery_packets():
    assert battery.QUERY == bytes([149, 6, 21, 22, 23, 24, 25, 26])
    assert battery.REPLY_LEN == len(reply()) == 10


def test_what_the_robot_answered():
    # 2026-11-10, Noel's new cable: 16224 mV, 2522 of 2697 mAh
    s = battery.parse(reply())
    assert s['voltage_mv'] == 16224 and s['charge_mah'] == 2522 and s['capacity_mah'] == 2697
    assert s['percent'] == pytest.approx(93.5, abs=0.1)
    assert s['current_ma'] == -350 and s['temperature_c'] == 27
    assert s['charging_text'] == 'not charging' and not battery.is_charging(s)


def test_charging_states():
    assert battery.is_charging(battery.parse(reply(state=2, ma=1200)))
    assert battery.parse(reply(state=3))['charging_text'] == 'trickle charging'
    assert battery.parse(reply(state=9))['charging_text'] == 'state 9'


def test_signed_values():
    s = battery.parse(reply(ma=-1, temp=-5))
    assert s['current_ma'] == -1 and s['temperature_c'] == -5


def test_no_or_short_reply():
    assert battery.parse(b'') is None
    assert battery.parse(reply()[:7]) is None
    assert battery.parse(None) is None


def test_capacity_zero_has_no_percent():
    assert battery.parse(reply(charge=10, capacity=0))['percent'] is None


def test_percent_is_capped():
    assert battery.parse(reply(charge=3000, capacity=2697))['percent'] == 100.0


serial = pytest.importorskip('serial')      # roomba_oi imports pyserial
from lucia_control.roomba_oi import RoombaOI  # noqa: E402


class QuerySerial:
    def __init__(self, answer):
        self.answer, self.written, self.timeout, self.flushed, self.timeouts_seen = answer, [], 2, 0, []

    def reset_input_buffer(self):
        self.flushed += 1

    def write(self, data):
        self.written.append(bytes(data))

    def read(self, n):
        self.timeouts_seen.append(self.timeout)
        return self.answer[:n]


def test_query_reads_with_a_short_timeout_and_restores_it():
    roomba = RoombaOI.__new__(RoombaOI)
    roomba.ser = QuerySerial(reply())
    assert battery.parse(roomba.query(battery.QUERY, battery.REPLY_LEN))['percent'] > 93
    assert roomba.ser.written == [battery.QUERY] and roomba.ser.flushed == 1
    assert roomba.ser.timeouts_seen == [0.1] and roomba.ser.timeout == 2
