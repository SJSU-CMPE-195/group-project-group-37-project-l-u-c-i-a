"""
roomba_oi.py

Wrapper for the iRobot Roomba 650 Open Interface (OI).
Handles serial connection setup and encodes all commands so other
scripts don't need to manage opcodes or byte packing directly.

Usage:
    from roomba_oi import RoombaOI

    with RoombaOI('COM5') as roomba:       # Windows
    with RoombaOI('/dev/ttyUSB0') as roomba:  # Linux

    roomba.start()
    roomba.full_mode()
    roomba.drive(200, 32767)  # forward at 200 mm/s
"""

import serial
import struct
import time


class RoombaOI:
    # --- Opcodes ---
    OP_START        = 128
    OP_SAFE         = 131
    OP_FULL         = 132
    OP_STOP         = 173
    OP_DRIVE        = 137
    OP_DRIVE_DIRECT = 145
    OP_MOTORS       = 138
    OP_LEDS         = 139
    OP_DIGIT_ASCII  = 164
    OP_SENSORS      = 142
    OP_QUERY_LIST   = 149
    OP_SEEK_DOCK    = 143

    # --- Sensor packet IDs ---
    PKT_BUMPS_DROPS      = 7
    PKT_WALL             = 8
    PKT_CLIFF_LEFT       = 9
    PKT_CLIFF_FRONT_LEFT = 10
    PKT_CLIFF_FRONT_RIGHT= 11
    PKT_CLIFF_RIGHT      = 12
    PKT_DISTANCE         = 19
    PKT_ANGLE            = 20
    PKT_CHARGING_STATE   = 21
    PKT_VOLTAGE          = 22
    PKT_CURRENT          = 23
    PKT_TEMPERATURE      = 24
    PKT_BATTERY_CHARGE   = 25
    PKT_BATTERY_CAPACITY = 26
    PKT_LEFT_ENCODER     = 43
    PKT_RIGHT_ENCODER    = 44

    # Bytes returned per packet ID
    PACKET_SIZES = {
        7: 1, 8: 1, 9: 1, 10: 1, 11: 1, 12: 1,
        19: 2, 20: 2, 21: 1, 22: 2, 23: 2, 24: 1,
        25: 2, 26: 2, 43: 2, 44: 2,
    }

    def __init__(self, port, baud=115200, timeout=1):
        """
        Open a serial connection to the Roomba.

        Args:
            port:    Serial port string, e.g. 'COM5' or '/dev/ttyUSB0'
            baud:    Baud rate (Roomba 650 default: 115200)
            timeout: Read timeout in seconds
        """
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(2)  # allow connection to settle

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _send(self, *bytes_):
        """Write raw bytes to the serial port."""
        self.ser.write(bytes(bytes_))

    # ------------------------------------------------------------------
    # Mode control
    # ------------------------------------------------------------------

    def start(self):
        """Enter OI passive mode. Always call this first."""
        self._send(self.OP_START)
        time.sleep(0.1)

    def safe_mode(self):
        """
        Enter safe mode.
        Drive commands work, but the Roomba will stop automatically
        if a cliff or wheel-drop sensor triggers.
        """
        self._send(self.OP_SAFE)
        time.sleep(0.1)

    def full_mode(self):
        """
        Enter full mode.
        Gives complete control — safety stops are disabled.
        Use with caution.
        """
        self._send(self.OP_FULL)
        time.sleep(0.1)

    def passive_mode(self):
        """Return to passive mode (same opcode as start)."""
        self._send(self.OP_START)
        time.sleep(0.1)

    # ------------------------------------------------------------------
    # Motion
    # ------------------------------------------------------------------

    def drive(self, velocity, radius):
        """
        Control both wheels with a single velocity + turning radius.

        Args:
            velocity: mm/s, range -500 to 500
                      positive = forward, negative = backward
            radius:   mm, range -2000 to 2000
                      32767  = drive straight
                      1      = spin counter-clockwise in place
                      -1     = spin clockwise in place
                      positive = turn left, negative = turn right
        """
        vel = max(-500, min(500, int(velocity)))
        # 32767 is the special straight value per OI spec
        if radius == 32767:
            rad = 32767
        else:
            rad = max(-2000, min(2000, int(radius)))
        self._send(self.OP_DRIVE, *struct.pack('>hh', vel, rad))

    def drive_direct(self, left_velocity, right_velocity):
        """
        Independently control left and right wheel velocities.

        Args:
            left_velocity:  mm/s, -500 to 500
            right_velocity: mm/s, -500 to 500
        """
        lv = max(-500, min(500, int(left_velocity)))
        rv = max(-500, min(500, int(right_velocity)))
        # OI spec sends right wheel first, then left
        self._send(self.OP_DRIVE_DIRECT, *struct.pack('>hh', rv, lv))

    def stop(self):
        """Stop all wheel movement."""
        self.drive(0, 0)

    def seek_dock(self):
        """Command the Roomba to autonomously return to its charging dock."""
        self._send(self.OP_SEEK_DOCK)

    # ------------------------------------------------------------------
    # Display
    # ------------------------------------------------------------------

    def display_text(self, text):
        """
        Display up to 4 ASCII characters on the 7-segment LED display.
        Shorter strings are right-padded with spaces.
        """
        text = text.ljust(4)[:4]
        self._send(self.OP_DIGIT_ASCII, *[ord(c) for c in text])

    # ------------------------------------------------------------------
    # Sensors
    # ------------------------------------------------------------------

    def read_sensor_raw(self, packet_id):
        """
        Request a single sensor packet and return its raw bytes.

        Args:
            packet_id: One of the PKT_* constants

        Returns:
            bytes of length PACKET_SIZES[packet_id]
        """
        self._send(self.OP_SENSORS, packet_id)
        size = self.PACKET_SIZES.get(packet_id, 1)
        return self.ser.read(size)

    def read_sensor_int(self, packet_id, signed=False):
        """
        Request a sensor packet and decode it as an integer.

        Args:
            packet_id: One of the PKT_* constants
            signed:    True for signed values (e.g. current, temperature)

        Returns:
            int
        """
        raw = self.read_sensor_raw(packet_id)
        return int.from_bytes(raw, byteorder='big', signed=signed)

    def read_bumps(self):
        """
        Read bump and wheel-drop sensors.

        Returns:
            dict with keys: bump_right, bump_left,
                            wheeldrop_right, wheeldrop_left
        """
        val = self.read_sensor_int(self.PKT_BUMPS_DROPS)
        return {
            'bump_right':      bool(val & 0x01),
            'bump_left':       bool(val & 0x02),
            'wheeldrop_right': bool(val & 0x04),
            'wheeldrop_left':  bool(val & 0x08),
        }

    def read_cliffs(self):
        """
        Read all four cliff sensors.

        Returns:
            dict with keys: left, front_left, front_right, right
        """
        return {
            'left':        bool(self.read_sensor_int(self.PKT_CLIFF_LEFT)),
            'front_left':  bool(self.read_sensor_int(self.PKT_CLIFF_FRONT_LEFT)),
            'front_right': bool(self.read_sensor_int(self.PKT_CLIFF_FRONT_RIGHT)),
            'right':       bool(self.read_sensor_int(self.PKT_CLIFF_RIGHT)),
        }

    def read_battery(self):
        """
        Read battery state.

        Returns:
            dict with keys:
                voltage_mV, current_mA, temperature_C,
                charge_mAh, capacity_mAh, charge_pct
        """
        voltage  = self.read_sensor_int(self.PKT_VOLTAGE)
        current  = self.read_sensor_int(self.PKT_CURRENT, signed=True)
        temp     = self.read_sensor_int(self.PKT_TEMPERATURE, signed=True)
        charge   = self.read_sensor_int(self.PKT_BATTERY_CHARGE)
        capacity = self.read_sensor_int(self.PKT_BATTERY_CAPACITY)
        pct = (charge / capacity * 100) if capacity > 0 else 0

        return {
            'voltage_mV':   voltage,
            'current_mA':   current,
            'temperature_C': temp,
            'charge_mAh':   charge,
            'capacity_mAh': capacity,
            'charge_pct':   round(pct, 1),
        }

    def read_encoders(self):
        """
        Read raw wheel encoder counts (unsigned 16-bit, wraps at 65535).

        Returns:
            dict with keys: left, right
        """
        return {
            'left':  self.read_sensor_int(self.PKT_LEFT_ENCODER),
            'right': self.read_sensor_int(self.PKT_RIGHT_ENCODER),
        }

    # ------------------------------------------------------------------
    # Context manager support
    # ------------------------------------------------------------------

    def reset(self):
        """
        Soft reset the Roomba (opcode 7).
        Reboots the OI and returns to a clean passive state.
        Equivalent to removing and reinserting the battery.
        """
        self._send(7)
        time.sleep(3)  # allow reboot to complete

    def close(self):
        """Stop, reset, and close the serial port cleanly."""
        self.stop()
        self.reset()
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
