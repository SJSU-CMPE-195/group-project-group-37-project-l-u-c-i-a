"""
lidar_reader.py

Wrapper for the YDLIDAR G4 sensor using the ydlidar SDK.
Mirrors the RoombaOI context-manager pattern so other scripts can
import and use it without touching serial setup or scan framing.

Install:
    pip install ydlidar

Usage:
    from lidar_reader import LidarReader

    with LidarReader('/dev/ttyUSB0') as lidar:
        print(lidar.get_info())
        for scan in lidar.iter_scans():
            # scan is a list of (intensity, angle_deg, distance_mm) tuples
            process(scan)
"""

import math
import ydlidar


class LidarReader:

    def __init__(self, port, baudrate=230400, scan_frequency=9.0, sample_rate=9):
        """
        Configure and initialize the YDLIDAR G4.

        Args:
            port:           Serial port, e.g. '/dev/ttyUSB0'
            baudrate:       Default 230400 for G4
            scan_frequency: Rotations per second (7–12 Hz)
            sample_rate:    Sample rate in kHz (9 for G4)
        """
        ydlidar.os_init()
        self._laser = ydlidar.CYdLidar()
        self._info = {
            'port': port,
            'baudrate': baudrate,
            'scan_frequency': scan_frequency,
            'sample_rate': sample_rate,
            'max_range_m': 16.0,
            'min_range_m': 0.08,
        }

        self._laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
        self._laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, baudrate)
        self._laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
        self._laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
        self._laser.setlidaropt(ydlidar.LidarPropScanFrequency, float(scan_frequency))
        self._laser.setlidaropt(ydlidar.LidarPropSampleRate, sample_rate)
        self._laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
        self._laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
        self._laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
        self._laser.setlidaropt(ydlidar.LidarPropMaxRange, 16.0)
        self._laser.setlidaropt(ydlidar.LidarPropMinRange, 0.08)
        # Note: SDK spells this "Intenstiy" — that is not a typo here
        self._laser.setlidaropt(ydlidar.LidarPropIntenstiy, False)

        if not self._laser.initialize():
            raise RuntimeError(f"Failed to initialize YDLIDAR G4 on {port}")
        if not self._laser.turnOn():
            self._laser.disconnecting()
            raise RuntimeError("YDLIDAR G4 initialized but failed to start scanning")

    # ------------------------------------------------------------------
    # Device info
    # ------------------------------------------------------------------

    def get_info(self):
        """
        Return device configuration as a dict.

        Returns:
            dict with keys: port, baudrate, scan_frequency, sample_rate,
                            max_range_m, min_range_m
        """
        return dict(self._info)

    def get_health(self):
        """
        Return a health tuple compatible with the RPLiDAR convention.

        Returns:
            tuple ('Good', 0) — the G4 SDK does not expose a separate
            health query; a running sensor is considered Good.
        """
        return ('Good', 0)

    # ------------------------------------------------------------------
    # Scanning
    # ------------------------------------------------------------------

    @staticmethod
    def _to_tuple(point):
        angle_deg = math.degrees(point.angle) % 360.0
        distance_mm = point.range * 1000.0
        return (point.intensity, angle_deg, distance_mm)

    def iter_scans(self, min_len=5):
        """
        Yield complete 360-degree scans as they arrive.

        Each scan is a list of (intensity, angle_deg, distance_mm) tuples:
            intensity   — signal strength (float; 0.0 when intensity is disabled)
            angle_deg   — degrees, 0.0–359.99, clockwise from front
            distance_mm — millimetres; points with range == 0 are excluded

        Args:
            min_len: Minimum number of valid points to yield a scan (default 5)

        Yields:
            list of (intensity, angle_deg, distance_mm) tuples
        """
        scan = ydlidar.LaserScan()
        while ydlidar.os_isOk():
            if self._laser.doProcessSimple(scan):
                points = [self._to_tuple(p) for p in scan.points if p.range > 0]
                if len(points) >= min_len:
                    yield points

    def read_scan(self, min_len=5):
        """
        Block until one complete scan arrives and return it.

        Returns:
            list of (intensity, angle_deg, distance_mm) tuples
        """
        for scan in self.iter_scans(min_len=min_len):
            return scan

    # ------------------------------------------------------------------
    # Context manager support
    # ------------------------------------------------------------------

    def close(self):
        """Stop the motor and close the serial connection cleanly."""
        self._laser.turnOff()
        self._laser.disconnecting()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
