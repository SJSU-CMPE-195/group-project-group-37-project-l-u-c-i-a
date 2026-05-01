"""
lidar_reader.py

Wrapper for the RPLiDAR sensor using the rplidar library.
Mirrors the RoombaOI context-manager pattern so other scripts can
import and use it without touching serial setup or scan framing.

Install:
    pip install rplidar

Usage:
    from lidar_reader import LidarReader

    with LidarReader('/dev/ttyUSB1') as lidar:
        print(lidar.get_info())
        for scan in lidar.iter_scans():
            # scan is a list of (quality, angle_deg, distance_mm) tuples
            process(scan)
"""

from rplidar import RPLidar


class LidarReader:

    def __init__(self, port, baudrate=115200, timeout=1):
        """
        Open a serial connection to the RPLiDAR.

        Args:
            port:     Serial port, e.g. '/dev/ttyUSB1'
            baudrate: Default 115200; some models use 256000
            timeout:  Read timeout in seconds
        """
        self._lidar = RPLidar(port, baudrate=baudrate, timeout=timeout)

    # ------------------------------------------------------------------
    # Device info
    # ------------------------------------------------------------------

    def get_info(self):
        """
        Return device information.

        Returns:
            dict with keys: model, firmware, hardware, serialnumber
        """
        return self._lidar.get_info()

    def get_health(self):
        """
        Return device health status.

        Returns:
            tuple (status, error_code)
            status is 'Good', 'Warning', or 'Error'
        """
        return self._lidar.get_health()

    # ------------------------------------------------------------------
    # Scanning
    # ------------------------------------------------------------------

    def iter_scans(self, min_len=5):
        """
        Yield complete 360-degree scans as they arrive.

        Each scan is a list of (quality, angle, distance) tuples:
            quality  — signal strength (0–255; discard if 0)
            angle    — degrees, 0.0–359.99, clockwise from front
            distance — millimetres; 0 means no object detected

        Args:
            min_len: Minimum number of points to accept a scan (default 5)

        Yields:
            list of (quality, angle, distance) tuples
        """
        yield from self._lidar.iter_scans(min_len=min_len)

    def read_scan(self, min_len=5):
        """
        Block until one complete scan arrives and return it.

        Returns:
            list of (quality, angle, distance) tuples
        """
        for scan in self._lidar.iter_scans(min_len=min_len):
            return scan

    # ------------------------------------------------------------------
    # Context manager support
    # ------------------------------------------------------------------

    def close(self):
        """Stop the motor and close the serial port cleanly."""
        self._lidar.stop()
        self._lidar.stop_motor()
        self._lidar.disconnect()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
