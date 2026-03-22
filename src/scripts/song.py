"""
song.py

Plays songs on the Roomba's built-in speaker using the iRobot OI.

The Roomba supports up to 4 stored song slots (0-3), each up to 16 notes.
Notes are MIDI note numbers (31-127).
Duration is in units of 1/64th of a second (e.g. 32 = 0.5s).

Usage:
    python song.py --port COM5 --song velvet_room
    python song.py --port COM5 --song la_cucaracha
"""

import argparse
import time

from roomba_oi import RoombaOI

# ------------------------------------------------------------------
# MIDI note numbers
# ------------------------------------------------------------------
C4  = 60
D4  = 62
E4  = 64
F4  = 65
FS4 = 66  # F#4
G4  = 67
GS4 = 68  # G#4
A4  = 69
AS4 = 70  # A#4 / Bb4
B4  = 71
C5  = 72  # C'
CS5 = 73  # C#'
D5  = 74  # D'

# ------------------------------------------------------------------
# Song definitions — list of (midi_note, duration) tuples
# ------------------------------------------------------------------

# Velvet Room theme (Aria of the Soul - Persona)
# Part A: E - A - C' - B - A - G# - A - B - A - E
VELVET_ROOM_A = [
    (E4,  32),
    (A4,  32),
    (C5,  32),
    (B4,  32),
    (A4,  32),
    (GS4, 32),
    (A4,  32),
    (B4,  32),
    (A4,  32),
    (E4,  48),
]

# Part B: E - B - D' - C' - B - A# - B - C#' - B - F#
VELVET_ROOM_B = [
    (E4,  32),
    (B4,  32),
    (D5,  32),
    (C5,  32),
    (B4,  32),
    (AS4, 32),
    (B4,  32),
    (CS5, 32),
    (B4,  32),
    (FS4, 48),
]

# La Cucaracha
LA_CUCARACHA = [
    (E4, 16), (E4, 16), (E4, 16),
    (C4, 12), (E4, 12), (G4, 24),
    (G4, 16), (G4, 16), (F4, 16),
    (E4, 16), (D4, 16), (C4, 24),
    (E4, 16), (E4, 16), (E4, 16),
    (C4, 12),
]

# ------------------------------------------------------------------
# Helpers
# ------------------------------------------------------------------

def load_song(roomba, slot, notes):
    """Send a Song definition command (opcode 140) to the Roomba."""
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
    parser.add_argument('--song', choices=['velvet_room', 'la_cucaracha'],
                        default='velvet_room',
                        help='Which song to play (default: velvet_room)')
    args = parser.parse_args()

    print(f"Connecting on {args.port}...")
    with RoombaOI(args.port) as roomba:
        roomba.start()
        roomba.full_mode()
        time.sleep(0.5)

        if args.song == 'velvet_room':
            print("Loading Velvet Room theme...")
            load_song(roomba, 0, VELVET_ROOM_A)
            load_song(roomba, 1, VELVET_ROOM_B)

            print("Playing Part A...")
            play_song(roomba, 0)
            time.sleep(song_duration(VELVET_ROOM_A) + 0.3)

            print("Playing Part B...")
            play_song(roomba, 1)
            time.sleep(song_duration(VELVET_ROOM_B) + 0.3)

        elif args.song == 'la_cucaracha':
            print("Loading La Cucaracha...")
            load_song(roomba, 0, LA_CUCARACHA)

            print("Playing...")
            play_song(roomba, 0)
            time.sleep(song_duration(LA_CUCARACHA) + 0.3)

        print("Done.")


if __name__ == '__main__':
    main()
