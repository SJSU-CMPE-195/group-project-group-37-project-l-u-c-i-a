"""Tests for mapping_process.py, using a stand-in command instead of ros2 launch."""

import sys
import time

from lucia_web.mapping_process import MappingProcess

# Prints its arguments, then waits until interrupted - like `ros2 launch` does.
FAKE_LAUNCH = [sys.executable, '-c',
               'import sys, time; print("args", sys.argv[1:], flush=True); time.sleep(60)']


def wait_for(condition, timeout_s=5.0):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if condition():
            return True
        time.sleep(0.05)
    return False


def test_start_runs_and_captures_output():
    m = MappingProcess(FAKE_LAUNCH)
    try:
        assert m.start(use_imu=False)
        assert m.running
        assert m.use_imu is False
        assert wait_for(lambda: any("use_imu:=false" in line for line in m.log_tail()))
    finally:
        m.stop()


def test_settings_file_goes_to_the_launch():
    m = MappingProcess(FAKE_LAUNCH)
    try:
        assert m.start(use_imu=True, settings_file='/tmp/lucia_settings.yaml')
        assert wait_for(lambda: any("settings_file:=/tmp/lucia_settings.yaml" in line for line in m.log_tail()))
    finally:
        m.stop()


def test_cannot_start_twice():
    m = MappingProcess(FAKE_LAUNCH)
    try:
        assert m.start()
        assert not m.start()
    finally:
        m.stop()


def test_stop_ends_the_process():
    m = MappingProcess(FAKE_LAUNCH)
    m.start()
    assert m.stop(timeout_s=5.0)
    assert not m.running
    assert m.use_imu is None
    assert wait_for(lambda: any('mapping stopped' in line for line in m.log_tail()))


def test_stop_when_not_running():
    assert not MappingProcess(FAKE_LAUNCH).stop()


def test_uptime_counts_while_running():
    m = MappingProcess(FAKE_LAUNCH)
    assert m.uptime_s is None
    try:
        m.start()
        time.sleep(0.2)
        assert 0.1 < m.uptime_s < 5.0
    finally:
        m.stop()
    assert m.uptime_s is None
