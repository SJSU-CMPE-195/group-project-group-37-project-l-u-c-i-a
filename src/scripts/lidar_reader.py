"""
lidar_reader.py

Wrapper for the RPLiDAR A2M8 sensor using the rplidar package.
Mirrors the RoombaOI context-manager pattern so other scripts can
import and use it without touching serial setup or scan framing.

Install:
    pip install rplidar-roboticia

Usage:
    from lidar_reader import LidarReader

    with LidarReader('/dev/ttyUSB0') as lidar:
        print(lidar.get_info())
        for scan in lidar.iter_scans():
            # scan is a list of (quality, angle_deg, distance_mm) tuples
            process(scan)
"""

from rplidar import RPLidar


class LidarReader:

    def __init__(self, port, baudrate=115200):
        """
        Connect to the RPLiDAR A2M8.

        Args:
            port:     Serial port, e.g. '/dev/ttyUSB0'
            baudrate: Default 115200 for A2M8
        """
        self._lidar = RPLidar(port, baudrate=baudrate)
        self._info = {
            'port': port,
            'baudrate': baudrate,
            'scan_frequency': 10.0,
            'sample_rate': 8,
            'max_range_m': 12.0,
            'min_range_m': 0.15,
        }

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
        Return a (status, error_code) health tuple.

        Returns:
            tuple e.g. ('Good', 0)
        """
        return self._lidar.get_health()

    # ------------------------------------------------------------------
    # Scanning
    # ------------------------------------------------------------------

    def iter_scans(self, min_len=5, min_quality=0):
        """
        Yield complete 360-degree scans as they arrive.

        Each scan is a list of (quality, angle_deg, distance_mm) tuples:
            quality     — signal quality (0–15)
            angle_deg   — degrees, 0.0–359.99, clockwise from front
            distance_mm — millimetres

        Args:
            min_len:     Minimum number of points to yield a scan (default 5)
            min_quality: Minimum point quality to include (default 0)

        Yields:
            list of (quality, angle_deg, distance_mm) tuples
        """
        for scan in self._lidar.iter_scans():
            points = [(q, a, d) for q, a, d in scan if q >= min_quality]
            if len(points) >= min_len:
                yield points

    def read_scan(self, min_len=5):
        """
        Block until one complete scan arrives and return it.

        Returns:
            list of (quality, angle_deg, distance_mm) tuples
        """
        for scan in self.iter_scans(min_len=min_len):
            return scan

    # ------------------------------------------------------------------
    # Context manager support
    # ------------------------------------------------------------------

    def close(self):
        """Stop the motor and close the serial connection cleanly."""
        self._lidar.stop()
        self._lidar.stop_motor()
        self._lidar.disconnect()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
