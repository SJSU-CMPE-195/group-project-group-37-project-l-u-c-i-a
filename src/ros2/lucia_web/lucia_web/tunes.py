"""
tunes.py

Tunes for the Roomba's speaker, for the web page: the built-in ones, saved
ones, a text format for writing and editing them, and turning a MIDI file into
one. Pure Python (no ROS), so it can be tested anywhere.

A tune is a list of (note, duration) pairs: MIDI note number (31-127 sounds,
anything else is a rest) and duration in 1/64 s (1-255). At most 64 notes: the
Roomba holds 4 songs of 16 notes (lucia_control/song.py plays them).

Text format, one token per note, separated by spaces or commas:
    B4:16 G#4:16 Bb3:24 R:8 60:16
note name with octave (C4 is middle C, MIDI 60), a MIDI number, or R for a
rest; then a colon and the duration in 1/64 s (16 = a quarter second). Without
a duration a note lasts 16.
"""

import bisect
import json
import os
import re
import struct

NOTE_MIN, NOTE_MAX = 31, 127
REST = 0
MAX_NOTES = 64
MAX_DURATION = 255
DEFAULT_DURATION = 16

NAMES = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']
SEMITONE = {'C': 0, 'D': 2, 'E': 4, 'F': 5, 'G': 7, 'A': 9, 'B': 11}
NOTE_RE = re.compile(r'^([A-Ga-g])([#b]?)(-?\d)$')
TUNE_NAME_RE = re.compile(r"^[\w ()'!&.,+-]{1,40}$")

# ----------------------------------------------------------------------
# Built-in tunes
# ----------------------------------------------------------------------

BUILTIN = {
    'Horn': [(67, 10), (REST, 5), (67, 26)],
    'Charge!': [(67, 10), (72, 10), (76, 10), (79, 20), (76, 10), (79, 40)],
}
# La Cucaracha and Mass Destruction (Persona 3) were built in until 2026-10-23;
# more tunes come from MIDI files (saved in tunes.json on the Pi).

# ----------------------------------------------------------------------
# Checking and text format
# ----------------------------------------------------------------------


def clean_tune(notes):
    """
    The tune as (note, duration) ints within the Roomba's limits; notes it
    can't play become rests. Raises ValueError if empty, too long or malformed.
    """
    try:
        pairs = [(int(n), int(d)) for n, d in notes]
    except (TypeError, ValueError):
        raise ValueError('a tune is a list of [note, duration] pairs')
    if not pairs:
        raise ValueError('the tune has no notes')
    if len(pairs) > MAX_NOTES:
        raise ValueError(f'{len(pairs)} notes; the Roomba can play {MAX_NOTES}')
    return [(n if NOTE_MIN <= n <= NOTE_MAX else REST, max(1, min(MAX_DURATION, d)))
            for n, d in pairs]


def tune_seconds(notes):
    return sum(d for _, d in notes) / 64.0


def note_number(token):
    """'C#4' / 'Bb3' / '60' / 'R' -> MIDI number (REST for a rest). ValueError if not a note."""
    if token.upper() in ('R', '-'):
        return REST
    if re.fullmatch(r'\d{1,3}', token):
        return int(token)
    m = NOTE_RE.match(token)
    if not m:
        raise ValueError(f'"{token}" is not a note (try C4, F#5, Bb3, 60 or R)')
    letter, accidental, octave = m.groups()
    semi = SEMITONE[letter.upper()] + (1 if accidental == '#' else -1 if accidental == 'b' else 0)
    return (int(octave) + 1) * 12 + semi


def note_name(number):
    if not NOTE_MIN <= number <= NOTE_MAX:
        return 'R'
    return f'{NAMES[number % 12]}{number // 12 - 1}'


def parse_tune_text(text):
    """Text format (see module docstring) -> cleaned tune. ValueError says which token is wrong."""
    if not isinstance(text, str):
        raise ValueError('no notes')
    tokens = [t for t in re.split(r'[\s,]+', text.strip()) if t]
    notes = []
    for i, token in enumerate(tokens, 1):
        name, _, dur = token.partition(':')
        try:
            number = note_number(name)
            duration = int(dur) if dur else DEFAULT_DURATION
        except ValueError as exc:
            raise ValueError(f'note {i}: {exc}' if 'note' in str(exc) else f'note {i}: "{token}" has a bad duration')
        if not 1 <= duration <= MAX_DURATION:
            raise ValueError(f'note {i}: duration {duration} is outside 1-{MAX_DURATION} (1/64 s)')
        notes.append((number, duration))
    return clean_tune(notes)


def tune_text(notes):
    """Cleaned tune -> text format."""
    return ' '.join(f'{note_name(n)}:{d}' for n, d in notes)


# ----------------------------------------------------------------------
# Saved tunes (a JSON file next to the saved maps)
# ----------------------------------------------------------------------


def valid_tune_name(name):
    return isinstance(name, str) and bool(TUNE_NAME_RE.match(name)) and name.strip() == name


def load_library(path):
    """Saved tunes, {name: tune}. Unreadable files or entries are skipped."""
    try:
        with open(path) as f:
            raw = json.load(f)
    except (OSError, ValueError):
        return {}
    out = {}
    if isinstance(raw, dict):
        for name, notes in raw.items():
            try:
                if valid_tune_name(name):
                    out[name] = clean_tune(notes)
            except ValueError:
                continue
    return out


def save_library(path, library):
    """Write the saved tunes (to a temporary file first, so a power cut can't leave half a file)."""
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump({name: [list(p) for p in notes] for name, notes in sorted(library.items())}, f, indent=1)
    os.replace(tmp, path)


# ----------------------------------------------------------------------
# MIDI files -> a tune
# ----------------------------------------------------------------------


def _vlq(data, i):
    value = 0
    while True:
        b = data[i]
        i += 1
        value = (value << 7) | (b & 0x7F)
        if not b & 0x80:
            return value, i


def parse_midi(data):
    """
    A Standard MIDI File's notes, as parts: one per track and channel, so a
    one-track file with several instruments still splits up. Returns a list of
    {'name', 'channel', 'notes': [(start_s, end_s, pitch), ...]} with at least
    one note, in file order. ValueError if it isn't a MIDI file it can read.
    """
    try:
        return _parse_midi(bytes(data))
    except (IndexError, struct.error):
        raise ValueError('the MIDI file is cut short or damaged')


def _parse_midi(data):
    if data[:4] != b'MThd':
        raise ValueError('not a MIDI file')
    header_len = int.from_bytes(data[4:8], 'big')
    _fmt, ntracks, division = struct.unpack('>HHH', data[8:14])
    if division & 0x8000:
        raise ValueError('this MIDI file uses SMPTE timing, which isn\'t supported')
    i = 8 + header_len
    tempos = [(0, 500000)]                   # (tick, microseconds per quarter note)
    raw_parts = {}                           # (track, channel) -> [(start_tick, end_tick, pitch)]
    names = {}
    for track in range(ntracks):
        if data[i:i + 4] != b'MTrk':
            raise ValueError('the MIDI file is damaged (track header missing)')
        length = int.from_bytes(data[i + 4:i + 8], 'big')
        j, end = i + 8, i + 8 + length
        i = end
        tick, status, sounding = 0, None, {}
        while j < end:
            delta, j = _vlq(data, j)
            tick += delta
            b = data[j]
            if b == 0xFF:                            # meta event
                kind = data[j + 1]
                mlen, j = _vlq(data, j + 2)
                body = data[j:j + mlen]
                j += mlen
                if kind == 0x51 and mlen == 3:
                    tempos.append((tick, int.from_bytes(body, 'big')))
                elif kind == 0x03 and track not in names:
                    names[track] = body.decode('latin-1', 'replace').strip()
                elif kind == 0x2F:
                    break
                continue
            if b in (0xF0, 0xF7):                    # system exclusive
                slen, j = _vlq(data, j + 1)
                j += slen
                continue
            if b & 0x80:
                status = b
                j += 1
            elif status is None:
                raise ValueError('the MIDI file is damaged (no status byte)')
            kind, channel = status & 0xF0, status & 0x0F
            size = 1 if kind in (0xC0, 0xD0) else 2
            args = data[j:j + size]
            j += size
            if kind == 0x90 and args[1] > 0:
                sounding.setdefault((channel, args[0]), []).append(tick)
            elif kind == 0x80 or (kind == 0x90 and args[1] == 0):
                starts = sounding.get((channel, args[0]))
                if starts:
                    raw_parts.setdefault((track, channel), []).append((starts.pop(0), tick, args[0]))

    tempos.sort()
    # seconds at each tempo change, so any tick converts with one lookup
    ticks, secs, per_q = [], [], []
    t_sec = 0.0
    for k, (tick, us) in enumerate(tempos):
        if k:
            prev_tick, prev_us = tempos[k - 1]
            t_sec += (tick - prev_tick) * prev_us / 1e6 / division
        ticks.append(tick)
        secs.append(t_sec)
        per_q.append(us)

    def seconds(tick):
        k = bisect.bisect_right(ticks, tick) - 1
        return secs[k] + (tick - ticks[k]) * per_q[k] / 1e6 / division

    parts = []
    for (track, channel), notes in sorted(raw_parts.items()):
        name = names.get(track) or f'Track {track + 1}'
        parts.append({
            'name': f'{name} (ch {channel + 1}{", drums" if channel == 9 else ""})',
            'channel': channel,
            'notes': sorted((seconds(s), seconds(e), p) for s, e, p in notes),
        })
    if not parts:
        raise ValueError('the MIDI file has no notes')
    return parts


def guess_melody_part(parts):
    """Index of the part most likely to be the tune: not drums, at least 8 notes, highest on average."""
    best, best_key = 0, None
    for k, part in enumerate(parts):
        notes = part['notes']
        key = (part['channel'] != 9, len(notes) >= 8, sum(p for _, _, p in notes) / len(notes))
        if best_key is None or key > best_key:
            best, best_key = k, key
    return best


def melody(notes, start_s=0.0, transpose=0, max_notes=MAX_NOTES):
    """
    One part's notes -> a Roomba tune: one note at a time (the highest of any
    that start together), rests for gaps, times rounded to 1/64 s, starting at
    the first note at or after start_s. Returns (tune, info) where info says
    how many notes the whole melody had, whether it was cut to max_notes, and
    how many notes fell outside the Roomba's range (they become rests; try
    transposing by an octave).
    """
    picked = []                                     # (start, end, pitch) per onset
    for s, e, p in sorted(notes):
        if s < start_s - 1e-6:
            continue
        if picked and s - picked[-1][0] < 0.01:     # starts together: keep the highest
            if p > picked[-1][2]:
                picked[-1] = (picked[-1][0], e, p)
            continue
        picked.append((s, e, p))
    if not picked:
        raise ValueError('no notes after that start time')
    t0 = picked[0][0]
    unit = lambda t: int(round((t - t0) * 64))      # noqa: E731

    out, out_of_range = [], 0
    for k, (s, e, p) in enumerate(picked):
        start = unit(s)
        nxt = unit(picked[k + 1][0]) if k + 1 < len(picked) else unit(e)
        stop = max(start, min(unit(e), nxt))
        pitch = p + transpose
        if not NOTE_MIN <= pitch <= NOTE_MAX:
            out_of_range += 1
            pitch = REST
        for n, length in ((pitch, stop - start), (REST, nxt - stop)):
            while length > 0:
                piece = min(length, MAX_DURATION)
                if out and n == REST and out[-1][0] == REST and out[-1][1] + piece <= MAX_DURATION:
                    out[-1] = (REST, out[-1][1] + piece)
                else:
                    out.append((n, piece))
                length -= piece
    while out and out[-1][0] == REST:
        out.pop()
    info = {'available': len(out), 'truncated': len(out) > max_notes, 'out_of_range': out_of_range}
    return out[:max_notes], info
