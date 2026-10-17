"""
song.py

Tunes for the Roomba's speaker, without ROS or serial, so they can be tested
anywhere. roomba_bridge uses this to play a tune sent from the web page.

A tune is a list of (note, duration) pairs:
- note: MIDI note number. The Roomba plays 31..127 (G1 up); anything outside
  that range is a rest (silence).
- duration: in 1/64 s, 1..255 (255 is about 4 s).

The Roomba stores up to 4 songs ("slots") of up to 16 notes each and plays one
slot at a time, so a tune is at most 64 notes, played as up to 4 slots back to
back.
"""

NOTE_MIN = 31
NOTE_MAX = 127
REST = 0
SLOT_NOTES = 16
SLOTS = 4
MAX_NOTES = SLOT_NOTES * SLOTS
MAX_DURATION = 255


def clean_tune(notes):
    """
    The tune as a list of (note, duration) ints the Roomba accepts: notes it
    can't play become rests, durations are kept within 1..255. Raises
    ValueError if it's empty, too long, or not a list of pairs.
    """
    try:
        pairs = [(int(n), int(d)) for n, d in notes]
    except (TypeError, ValueError):
        raise ValueError('a tune is a list of [note, duration] pairs')
    if not pairs:
        raise ValueError('the tune has no notes')
    if len(pairs) > MAX_NOTES:
        raise ValueError(f'the tune has {len(pairs)} notes; the Roomba can play {MAX_NOTES}')
    return [(n if NOTE_MIN <= n <= NOTE_MAX else REST, max(1, min(MAX_DURATION, d)))
            for n, d in pairs]


def slots(notes):
    """Split a tune into the Roomba's song slots: [(slot number, pairs), ...], at most 4 of 16."""
    tune = clean_tune(notes)
    return [(i, tune[i * SLOT_NOTES:(i + 1) * SLOT_NOTES])
            for i in range((len(tune) + SLOT_NOTES - 1) // SLOT_NOTES)]


def seconds(pairs):
    """How long a list of (note, duration) pairs plays, in seconds."""
    return sum(d for _, d in pairs) / 64.0


def slot_start_times(notes, gap_s=0.05):
    """
    When to start each slot, in seconds after the first: each one starts when
    the previous one has finished, plus a small gap (the Roomba ignores Play
    while a song is still playing).
    """
    out, t = [], 0.0
    for slot, pairs in slots(notes):
        out.append((slot, t))
        t += seconds(pairs) + gap_s
    return out
