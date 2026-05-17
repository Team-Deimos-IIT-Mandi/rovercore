"""
Unit tests for ScienceMissionStateMachine — no ROS, no hardware.
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from science_mission import ScienceMissionStateMachine


SITE = object()  # dummy pose sentinel


class TestScienceMissionHappyPath:
    def test_happy_path_all_sensors_ok(self):
        """Full pipeline: every step succeeds, arm enabled."""
        sm = ScienceMissionStateMachine(arm_enabled=True)
        sm.start(SITE)
        assert sm.state == 'NAV_TO_SITE'

        sm.nav_success();     assert sm.state == 'PROBE'
        sm.probe_success();   assert sm.state == 'DRILL'
        sm.drill_success();   assert sm.state == 'SENSE_SOIL'
        sm.soil_success();    assert sm.state == 'SPECTROMETER'
        sm.spec_success();    assert sm.state == 'MICROSCOPE'
        sm.micro_success();   assert sm.state == 'LOG'
        sm.log_success();     assert sm.state == 'RETRACT'
        sm.retract_success(); assert sm.state == 'DONE'
        assert sm.is_terminal
        assert sm.drill_ok and sm.soil_ok and sm.spec_ok and sm.micro_ok

    def test_happy_path_arm_disabled(self):
        """Arm disabled: PROBE and RETRACT skipped."""
        sm = ScienceMissionStateMachine(arm_enabled=False)
        sm.start(SITE)
        sm.nav_success();   assert sm.state == 'DRILL', 'Should skip PROBE'
        sm.drill_success(); assert sm.state == 'SENSE_SOIL'
        sm.soil_success()
        sm.spec_success()
        sm.micro_success(); assert sm.state == 'LOG'
        sm.log_success();   assert sm.state == 'DONE', 'Should skip RETRACT'
        assert sm.is_terminal


class TestDrillFailNonFatal:
    def test_drill_fail_continues(self):
        """Drill failure is non-fatal — pipeline continues to sensor phase."""
        sm = ScienceMissionStateMachine(arm_enabled=True)
        sm.start(SITE)
        sm.nav_success()
        sm.probe_success()
        sm.drill_fail()
        assert sm.state == 'SENSE_SOIL', 'Drill fail must NOT abort mission'
        assert sm.drill_ok is False

        # Complete remaining sensors + log
        sm.soil_success()
        sm.spec_success()
        sm.micro_success()
        sm.log_success()
        sm.retract_success()
        assert sm.state == 'DONE'

    def test_drill_fail_flag_preserved(self):
        sm = ScienceMissionStateMachine(arm_enabled=False)
        sm.start(SITE)
        sm.nav_success()
        sm.drill_fail()
        assert not sm.drill_ok
        # Ensure mission completes despite drill flag
        sm.soil_success(); sm.spec_success(); sm.micro_success()
        sm.log_success()
        assert sm.state == 'DONE'


class TestAllSensorsDeadAbort:
    def test_all_sensors_fail_aborts(self):
        """soil+spec+micro all fail → FAILED (nothing to log)."""
        sm = ScienceMissionStateMachine(arm_enabled=True)
        sm.start(SITE)
        sm.nav_success()
        sm.probe_success()
        sm.drill_success()
        sm.soil_fail()
        sm.spec_fail()
        sm.micro_fail()
        assert sm.state == 'FAILED', 'All sensors dead must abort mission'
        assert sm.is_terminal
        assert not sm.any_sensor_data

    def test_partial_sensor_failure_continues(self):
        """Two sensors fail, one succeeds → mission continues to LOG."""
        sm = ScienceMissionStateMachine(arm_enabled=True)
        sm.start(SITE)
        sm.nav_success()
        sm.probe_success()
        sm.drill_success()
        sm.soil_fail()
        sm.spec_fail()
        sm.micro_success()   # only microscope survives
        assert sm.state == 'LOG', 'Partial failure must not abort'
        assert sm.any_sensor_data
        sm.log_success()
        sm.retract_success()
        assert sm.state == 'DONE'


class TestNavFail:
    def test_nav_fail_aborts(self):
        sm = ScienceMissionStateMachine(arm_enabled=True)
        sm.start(SITE)
        sm.nav_fail()
        assert sm.state == 'FAILED'
        assert sm.is_terminal

    def test_reset_after_failure(self):
        sm = ScienceMissionStateMachine(arm_enabled=True)
        sm.start(SITE)
        sm.nav_fail()
        sm.reset()
        assert sm.state == 'IDLE'
        assert sm.drill_ok and sm.soil_ok and sm.spec_ok and sm.micro_ok
        assert not sm.is_terminal
