"""
mapping_process.py

Starts and stops `ros2 launch lucia_bringup mapping.launch.py` as a child
process for the web UI, and keeps the last lines of its output so the page can
show them. No ROS imports, so it can be tested with any command.
"""

import collections
import os
import signal
import subprocess
import threading
import time

MAPPING_COMMAND = ['ros2', 'launch', 'lucia_bringup', 'mapping.launch.py']


class MappingProcess:
    def __init__(self, command=None, log_lines=200):
        self._command = list(command or MAPPING_COMMAND)
        self._proc = None
        self._use_imu = None
        self._started_at = None
        self._log = collections.deque(maxlen=log_lines)
        self._lock = threading.Lock()

    @property
    def running(self):
        with self._lock:
            return self._proc is not None and self._proc.poll() is None

    @property
    def use_imu(self):
        return self._use_imu if self.running else None

    @property
    def uptime_s(self):
        """Seconds since mapping started, or None if it isn't running."""
        return time.monotonic() - self._started_at if self.running else None

    def log_tail(self, count=40):
        with self._lock:
            return list(self._log)[-count:]

    def start(self, use_imu=True, settings_file=None):
        """
        Start mapping. settings_file: a ROS 2 parameters file with the page's
        settings (settings.params_yaml), or None for the nodes' own defaults.
        Returns False if it's already running.
        """
        with self._lock:
            if self._proc is not None and self._proc.poll() is None:
                return False
            self._log.clear()
            args = self._command + [f'use_imu:={"true" if use_imu else "false"}']
            if settings_file:
                args.append(f'settings_file:={settings_file}')
            # Own process group, so stop() reaches every node the launch started.
            self._proc = subprocess.Popen(
                args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, start_new_session=True)
            self._use_imu = use_imu
            self._started_at = time.monotonic()
            proc = self._proc
        threading.Thread(target=self._read_output, args=(proc,), daemon=True).start()
        return True

    def stop(self, timeout_s=15.0):
        """Stop mapping like Ctrl+C would, escalating if it doesn't exit. Returns False if not running."""
        with self._lock:
            proc = self._proc
        if proc is None or proc.poll() is not None:
            return False
        for sig, wait_s in ((signal.SIGINT, timeout_s), (signal.SIGTERM, 5.0), (signal.SIGKILL, 5.0)):
            try:
                os.killpg(proc.pid, sig)
            except ProcessLookupError:
                break
            try:
                proc.wait(timeout=wait_s)
                break
            except subprocess.TimeoutExpired:
                continue
        return True

    def _read_output(self, proc):
        for line in proc.stdout:
            with self._lock:
                self._log.append(line.rstrip())
        code = proc.wait()
        with self._lock:
            self._log.append(f'--- mapping stopped (exit code {code}) ---')
