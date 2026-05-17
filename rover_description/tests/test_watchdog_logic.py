"""
Unit tests for WatchdogNode state machine logic.
Tests run without ROS infrastructure — logic is extracted and tested directly.
"""

import time
import pytest


# ---------------------------------------------------------------------------
# Minimal testable state machine extracted from WatchdogNode
# ---------------------------------------------------------------------------
class WatchdogState:
    """Pure-Python watchdog state, no ROS or GPIO dependencies."""

    def __init__(self, timeout_sec: float = 2.0):
        self.timeout_sec = timeout_sec
        self._estop_active = False
        self._last_heartbeat = time.monotonic()
        self.estop_count = 0

    def receive_heartbeat(self):
        if not self._estop_active:
            self._last_heartbeat = time.monotonic()

    def tick(self) -> bool:
        """Returns True if E-STOP should fire this tick."""
        if self._estop_active:
            return True
        elapsed = time.monotonic() - self._last_heartbeat
        if elapsed > self.timeout_sec:
            self._trigger()
            return True
        return False

    def _trigger(self):
        self._estop_active = True
        self.estop_count += 1

    @property
    def is_estop(self) -> bool:
        return self._estop_active


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
class TestWatchdogLogic:
    def test_timeout_triggers_estop(self):
        """No heartbeat → E-STOP fires after timeout."""
        wd = WatchdogState(timeout_sec=0.05)
        time.sleep(0.1)
        assert wd.tick() is True
        assert wd.is_estop is True

    def test_no_double_trigger(self):
        """E-STOP fires exactly once, even across multiple ticks."""
        wd = WatchdogState(timeout_sec=0.05)
        time.sleep(0.1)
        wd.tick()
        wd.tick()
        wd.tick()
        assert wd.estop_count == 1

    def test_heartbeat_resets_timer(self):
        """Continuous heartbeats keep watchdog from firing."""
        wd = WatchdogState(timeout_sec=0.1)
        for _ in range(20):
            wd.receive_heartbeat()
            time.sleep(0.01)
            assert wd.tick() is False
        assert wd.is_estop is False

    def test_estop_no_recovery(self):
        """Once E-STOP fires, heartbeat does NOT clear it."""
        wd = WatchdogState(timeout_sec=0.05)
        time.sleep(0.1)
        wd.tick()
        assert wd.is_estop is True
        wd.receive_heartbeat()  # simulate heartbeat arriving after E-STOP
        assert wd.is_estop is True, "E-STOP must not self-clear — operator restart required"
