"""
battery.py

The Roomba's battery, read in one Query List request (Open Interface opcode
149) instead of six single-packet requests. Pure Python, no ROS, so it can be
tested anywhere.

The Roomba's replies reach the Pi since Noel's new cable (BG046PKO, checked
2026-11-10: 16.2 V, 2522 of 2697 mAh); before that, nothing came back.

Packets, in the order asked:
    21 charging state   1 byte  (0-5, CHARGING_TEXT)
    22 voltage          2 bytes unsigned, mV
    23 current          2 bytes signed, mA (negative = discharging)
    24 temperature      1 byte  signed, degrees C
    25 battery charge   2 bytes unsigned, mAh
    26 battery capacity 2 bytes unsigned, mAh
All big-endian.
"""

import struct

PACKETS = (21, 22, 23, 24, 25, 26)
REPLY_LEN = 10
QUERY = bytes([149, len(PACKETS)] + list(PACKETS))

CHARGING_TEXT = {
    0: 'not charging',
    1: 'reconditioning',
    2: 'charging',
    3: 'trickle charging',
    4: 'waiting',
    5: 'charging fault',
}


def parse(reply):
    """
    The Roomba's 10-byte reply -> dict, or None if it's short (no reply: the
    Roomba is off or asleep, or the cable's reply wire isn't working).
    """
    if reply is None or len(reply) < REPLY_LEN:
        return None
    state, voltage, current, temp, charge, capacity = struct.unpack('>BHhbHH', bytes(reply[:REPLY_LEN]))
    percent = None
    if capacity > 0:
        percent = max(0.0, min(100.0, 100.0 * charge / capacity))
    return {
        'charging_state': state,
        'charging_text': CHARGING_TEXT.get(state, f'state {state}'),
        'voltage_mv': voltage,
        'current_ma': current,
        'temperature_c': temp,
        'charge_mah': charge,
        'capacity_mah': capacity,
        'percent': percent,
    }


def is_charging(state):
    """True while the Roomba is charging (any of its charging kinds)."""
    return state['charging_state'] in (1, 2, 3)
