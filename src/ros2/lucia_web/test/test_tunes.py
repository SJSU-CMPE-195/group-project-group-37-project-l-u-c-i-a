"""Tests for tunes.py. Run from src/ros2/lucia_web:  python3 -m pytest test"""

import json
import struct

import pytest

from lucia_web.tunes import (BUILTIN, MAX_NOTES, REST, clean_tune,
                             guess_melody_part, load_library, melody,
                             note_name, note_number, parse_midi,
                             parse_tune_text, save_library, tune_seconds,
                             tune_text, valid_tune_name)


# ---- a tiny MIDI file writer for the tests ---------------------------------

def vlq(n):
    out = [n & 0x7F]
    n >>= 7
    while n:
        out.append(0x80 | (n & 0x7F))
        n >>= 7
    return bytes(reversed(out))


def track(events, name=None):
    """events: (tick, bytes) in any order; returns an MTrk chunk."""
    body = b''
    if name:
        body += b'\x00\xff\x03' + vlq(len(name)) + name.encode()
    now = 0
    for tick, data in sorted(events, key=lambda e: e[0]):
        body += vlq(tick - now) + data
        now = tick
    body += b'\x00\xff\x2f\x00'
    return b'MTrk' + struct.pack('>I', len(body)) + body


def notes_on(channel, specs):
    """specs: (start_tick, end_tick, pitch) -> note on/off events."""
    ev = []
    for s, e, p in specs:
        ev.append((s, bytes([0x90 | channel, p, 100])))
        ev.append((e, bytes([0x80 | channel, p, 0])))
    return ev


def midi_file(tracks, division=480, fmt=1):
    return b'MThd' + struct.pack('>IHHH', 6, fmt, len(tracks), division) + b''.join(tracks)


TEMPO_120 = (0, b'\xff\x51\x03' + (500000).to_bytes(3, 'big'))     # 0.5 s per quarter note


# ---- text format -------------------------------------------------------------

@pytest.mark.parametrize('token, number', [('C4', 60), ('c4', 60), ('A4', 69), ('C#4', 61), ('Db4', 61),
                                           ('B3', 59), ('G1', 31), ('60', 60), ('R', REST), ('-', REST)])
def test_note_number(token, number):
    assert note_number(token) == number


def test_note_name():
    assert note_name(60) == 'C4' and note_name(61) == 'C#4' and note_name(0) == 'R'


def test_parse_and_format_round_trip():
    tune = parse_tune_text('B4:16 G#4:16, R:8 60 Bb3:24')
    assert tune == [(71, 16), (68, 16), (REST, 8), (60, 16), (58, 24)]
    assert parse_tune_text(tune_text(tune)) == tune


@pytest.mark.parametrize('text, says', [('', 'no notes'), ('H4:16', 'note 1'), ('C4:16 C4:x', 'note 2'),
                                        ('C4:999', 'duration 999'), (None, 'no notes'),
                                        (' '.join(['C4'] * 65), '65 notes')])
def test_parse_errors_say_where(text, says):
    with pytest.raises(ValueError) as e:
        parse_tune_text(text)
    assert says in str(e.value)


def test_builtins_are_valid_and_short():
    for name, notes in BUILTIN.items():
        assert clean_tune(notes) == notes, name
        assert valid_tune_name(name)
    assert len(BUILTIN['La Cucaracha']) == 24
    assert tune_seconds(BUILTIN['Horn']) < 1


# ---- saved tunes -------------------------------------------------------------

def test_library_round_trip(tmp_path):
    path = str(tmp_path / 'tunes.json')
    assert load_library(path) == {}
    save_library(path, {'Pokémon Center': [(72, 16), (76, 16)]})
    assert load_library(path) == {'Pokémon Center': [(72, 16), (76, 16)]}


def test_library_skips_bad_entries(tmp_path):
    path = tmp_path / 'tunes.json'
    path.write_text(json.dumps({'ok': [[60, 16]], 'bad': 'nope', '../evil': [[60, 16]], 'empty': []}))
    assert load_library(str(path)) == {'ok': [(60, 16)]}
    path.write_text('not json')
    assert load_library(str(path)) == {}


@pytest.mark.parametrize('name, ok', [('Pokémon Center', True), ("Persona 5 - Life Will Change", True),
                                      ('a' * 41, False), ('', False), ('../x', False), (' x', False), (3, False)])
def test_tune_names(name, ok):
    assert valid_tune_name(name) is ok


# ---- MIDI --------------------------------------------------------------------

def test_parse_midi_times_and_parts():
    data = midi_file([
        track([TEMPO_120]),
        track(notes_on(0, [(0, 480, 72), (480, 960, 74)]), name='Melody'),
        track(notes_on(9, [(0, 120, 36)]), name='Drums'),
    ])
    parts = parse_midi(data)
    assert [p['name'] for p in parts] == ['Melody (ch 1)', 'Drums (ch 10, drums)']
    assert parts[0]['notes'] == [(0.0, 0.5, 72), (0.5, 1.0, 74)]


def test_tempo_change_mid_song():
    tempo_fast = (480, b'\xff\x51\x03' + (250000).to_bytes(3, 'big'))   # twice as fast from beat 2
    data = midi_file([track([TEMPO_120, tempo_fast] + notes_on(0, [(0, 480, 60), (480, 960, 62)]))], fmt=0)
    (part,) = parse_midi(data)
    assert part['notes'] == [(0.0, 0.5, 60), (0.5, pytest.approx(0.75), 62)]


def test_format0_splits_by_channel_and_running_status():
    # running status: second note-on omits the status byte
    events = [(0, b'\x90\x48\x64'), (0, b'\x30\x64'), (480, b'\x80\x48\x00'), (480, b'\x80\x30\x00'),
              (0, b'\x91\x40\x64'), (960, b'\x81\x40\x00')]
    parts = parse_midi(midi_file([track(events)], fmt=0))
    assert [len(p['notes']) for p in parts] == [2, 1]
    assert sorted(p for _, _, p in parts[0]['notes']) == [0x30, 0x48]


@pytest.mark.parametrize('data, says', [(b'RIFF1234', 'not a MIDI file'),
                                        (midi_file([track([TEMPO_120])]), 'no notes'),
                                        (midi_file([track(notes_on(0, [(0, 480, 60)]))])[:-6], 'cut short')])
def test_parse_midi_errors(data, says):
    with pytest.raises(ValueError) as e:
        parse_midi(data)
    assert says in str(e.value)


def test_guess_melody_part_prefers_high_non_drum():
    bass = {'channel': 1, 'notes': [(i, i + 1, 40) for i in range(20)]}
    tune = {'channel': 0, 'notes': [(i, i + 1, 76) for i in range(12)]}
    drums = {'channel': 9, 'notes': [(i, i + 1, 90) for i in range(30)]}
    assert guess_melody_part([bass, drums, tune]) == 2


def test_melody_keeps_top_note_rests_and_units():
    notes = [(0.0, 0.25, 60), (0.0, 0.25, 64),     # chord: top note E4
             (0.5, 0.75, 67)]                     # after a quarter-second gap
    tune, info = melody(notes)
    assert tune == [(64, 16), (REST, 16), (67, 16)]
    assert info == {'available': 3, 'truncated': False, 'out_of_range': 0}


def test_melody_start_transpose_and_range():
    notes = [(0.0, 0.5, 60), (1.0, 1.5, 10), (1.5, 2.0, 62)]     # 10 + 12 = 22: still too low
    tune, info = melody(notes, start_s=1.0, transpose=12)
    assert tune == [(REST, 32), (74, 32)]
    assert info['out_of_range'] == 1


def test_melody_cuts_to_64_and_splits_long_notes():
    many = [(i * 0.25, i * 0.25 + 0.25, 72) for i in range(100)]
    tune, info = melody(many)
    assert len(tune) == MAX_NOTES and info['truncated'] and info['available'] == 100
    long_note, _ = melody([(0.0, 6.0, 72)])
    assert long_note == [(72, 255), (72, 129)]          # 6 s = 384/64, in pieces of at most 255


def test_melody_from_a_real_file():
    data = midi_file([track([TEMPO_120]),
                      track(notes_on(0, [(0, 240, 76), (240, 480, 79), (960, 1440, 84)]), name='Lead')])
    parts = parse_midi(data)
    tune, _ = melody(parts[guess_melody_part(parts)]['notes'])
    assert tune == [(76, 16), (79, 16), (REST, 32), (84, 32)]
