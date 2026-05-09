"""
Unit tests for PickPlaceStateMachine — no ROS, no hardware.
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from pick_place_server import PickPlaceStateMachine


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------
TARGET = object()   # dummy pose sentinel
DROP   = object()


class TestPickPlaceHappyPath:
    """Full sequence with arm enabled."""

    def test_happy_path_arm_enabled(self):
        sm = PickPlaceStateMachine(arm_enabled=True)
        assert sm.state == 'IDLE'

        sm.start(TARGET, DROP)
        assert sm.state == 'NAV_TO_PREGRASP'

        sm.nav_success()
        assert sm.state == 'PICK'

        sm.arm_success()   # deploy_ready + gripper close → STOW
        assert sm.state == 'STOW'

        sm.arm_success()   # stow → NAV_TO_DROP
        assert sm.state == 'NAV_TO_DROP'

        sm.nav_success()
        assert sm.state == 'PLACE'

        sm.arm_success()   # deploy_ready + gripper open → DONE
        assert sm.state == 'DONE'
        assert sm.is_terminal

    def test_happy_path_arm_disabled(self):
        """arm_enabled=False: nav still runs, arm steps skipped."""
        sm = PickPlaceStateMachine(arm_enabled=False)
        sm.start(TARGET, DROP)
        assert sm.state == 'NAV_TO_PREGRASP'

        sm.nav_success()
        assert sm.state == 'STOW', 'Should skip PICK when arm disabled'

        sm.arm_success()   # stow skips directly (arm disabled path handled in node)
        assert sm.state == 'NAV_TO_DROP'

        sm.nav_success()
        assert sm.state == 'DONE', 'Should skip PLACE when arm disabled'
        assert sm.is_terminal


class TestPickPlaceNavFail:
    """Nav failure at each navigation step transitions to FAILED."""

    def test_nav_fail_pregrasp(self):
        sm = PickPlaceStateMachine(arm_enabled=True)
        sm.start(TARGET, DROP)
        sm.nav_fail()
        assert sm.state == 'FAILED'
        assert sm.is_terminal

    def test_nav_fail_drop(self):
        sm = PickPlaceStateMachine(arm_enabled=True)
        sm.start(TARGET, DROP)
        sm.nav_success()  # PICK
        sm.arm_success()  # STOW
        sm.arm_success()  # NAV_TO_DROP
        sm.nav_fail()
        assert sm.state == 'FAILED'
        assert sm.is_terminal


class TestPickPlaceArmFail:
    """Arm failure at each arm step transitions to FAILED."""

    def test_arm_fail_pick(self):
        sm = PickPlaceStateMachine(arm_enabled=True)
        sm.start(TARGET, DROP)
        sm.nav_success()  # NAV_TO_PREGRASP → PICK
        sm.arm_fail()
        assert sm.state == 'FAILED'
        assert sm.is_terminal

    def test_arm_fail_stow(self):
        sm = PickPlaceStateMachine(arm_enabled=True)
        sm.start(TARGET, DROP)
        sm.nav_success()
        sm.arm_success()  # PICK → STOW
        sm.arm_fail()
        assert sm.state == 'FAILED'
        assert sm.is_terminal

    def test_arm_fail_place(self):
        sm = PickPlaceStateMachine(arm_enabled=True)
        sm.start(TARGET, DROP)
        sm.nav_success()
        sm.arm_success()  # PICK → STOW
        sm.arm_success()  # STOW → NAV_TO_DROP
        sm.nav_success()  # NAV_TO_DROP → PLACE
        sm.arm_fail()
        assert sm.state == 'FAILED'
        assert sm.is_terminal

    def test_reset_after_failure(self):
        sm = PickPlaceStateMachine(arm_enabled=True)
        sm.start(TARGET, DROP)
        sm.nav_fail()
        assert sm.is_terminal
        sm.reset()
        assert sm.state == 'IDLE'
        assert not sm.is_terminal
        assert sm.target_pose is None
