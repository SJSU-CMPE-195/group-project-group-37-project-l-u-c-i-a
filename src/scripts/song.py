"""
song.py

Plays songs on the Roomba's built-in speaker using the iRobot OI.

The Roomba supports up to 4 stored song slots (0-3), each up to 16 notes.
Notes are MIDI note numbers (31-127).
Duration is in units of 1/64th of a second (e.g. 32 = 0.5s).

Usage:
    python song.py --port COM5 --song mass_destruction
    python song.py --port COM5 --song la_cucaracha
"""

import argparse
import time

from roomba_oi import RoombaOI

# ------------------------------------------------------------------
# MIDI note numbers
# ------------------------------------------------------------------
C4  = 60
CS4 = 61  # C#4
D4  = 62
DS4 = 63  # D#4
E4  = 64
F4  = 65
FS4 = 66  # F#4
G4  = 67
GS4 = 68  # G#4
A4  = 69
AS4 = 70  # A#4 / Bb4
B4  = 71

# ------------------------------------------------------------------
# Song definitions — list of (midi_note, duration) tuples
# ------------------------------------------------------------------

# Mass Destruction (Persona) — B G# G# F# D# C# C# B C# D#
MASS_DESTRUCTION = [
    (B4,  16),
    (GS4, 16),
    (GS4, 24),
    (FS4, 16),
    (DS4, 16),
    (CS4, 16),
    (CS4, 24),
    (B4,  16),
    (CS4, 16),
    (DS4, 32),
]

# La Cucaracha — split across two slots (16 note limit per slot)
# Phrase 1: E E E | C E G
# Phrase 2: G G F | E D C
# Phrase 3: E E E | C E G  (same as phrase 1)
# Phrase 4: G F# F | E G C (ending)
LA_CUCARACHA_1 = [
    (E4, 16), (E4, 16), (E4, 16),   # E E E
    (C4, 12), (E4, 12), (G4, 24),   # C E G
    (G4, 16), (G4, 16), (F4, 16),   # G G F
    (E4, 16), (D4, 16), (C4, 24),   # E D C
    (E4, 16), (E4, 16), (E4, 16),   # E E E
    (C4, 12),                        # C ...
]

LA_CUCARACHA_2 = [
    (E4, 12), (G4, 24),              # ... E G  (finish phrase 3)
    (G4, 16), (FS4, 16), (F4, 16),  # G F# F
    (E4, 16), (G4, 16), (C4, 32),   # E G C (held ending)
]

# ------------------------------------------------------------------
# Helpers
# ------------------------------------------------------------------

def load_song(roomba, slot, notes):
    """Send a Song definition command (opcode 140) to the Roomba."""
    if len(notes) > 16:
        raise ValueError(f"Song has {len(notes)} notes — Roomba max is 16 per slot.")
    cmd = [140, slot, len(notes)]
    for note, duration in notes:
        cmd.extend([note, duration])
    roomba._send(*cmd)
    time.sleep(0.1)


def play_song(roomba, slot):
    """Send a Play command (opcode 141) for a given slot."""
    roomba._send(141, slot)


def song_duration(notes):
    """Total playback time for a note list in seconds."""
    return sum(d for _, d in notes) / 64.0


# ------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description='Play songs on the Roomba speaker')
    parser.add_argument('--port', default='COM5',
                        help='Serial port (e.g. COM5 or /dev/ttyUSB0)')
    parser.add_argument('--song', choices=['mass_destruction', 'la_cucaracha'],
                        default='mass_destruction',
                        help='Which song to play (default: mass_destruction)')
    args = parser.parse_args()

    print(f"Connecting on {args.port}...")
    with RoombaOI(args.port) as roomba:
        roomba.start()
        roomba.full_mode()
        time.sleep(0.5)

        if args.song == 'mass_destruction':
            print("Loading Mass Destruction...")
            load_song(roomba, 0, MASS_DESTRUCTION)

            print("Playing...")
            play_song(roomba, 0)
            time.sleep(song_duration(MASS_DESTRUCTION) + 0.3)

        elif args.song == 'la_cucaracha':
            print("Loading La Cucaracha...")
            load_song(roomba, 0, LA_CUCARACHA_1)
            load_song(roomba, 1, LA_CUCARACHA_2)

            print("Playing part 1...")
            play_song(roomba, 0)
            time.sleep(song_duration(LA_CUCARACHA_1) + 0.1)

            print("Playing part 2...")
            play_song(roomba, 1)
            time.sleep(song_duration(LA_CUCARACHA_2) + 0.3)

        print("Done.")


if __name__ == '__main__':
    main()
