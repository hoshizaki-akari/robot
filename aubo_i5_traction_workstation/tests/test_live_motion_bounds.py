"""Offline tests of live-move acceptance; no ROS or hardware imports."""

import runpy
from pathlib import Path

import pytest

_module = runpy.run_path(str(Path(__file__).resolve().parents[1] / 'scripts/live_motion_bounds.py'))
require_within_envelope = _module['require_within_envelope']
best_effort_stop = _module['best_effort_stop']
require_target_reached = _module['require_target_reached']
pose_is_after_fence = _module['pose_is_after_fence']


def test_final_overshoot_and_lateral_motion_cannot_pass():
    for position in ((.0502, 0, 0), (.04, .002, 0), (-.001, 0, 0)):
        with pytest.raises(ValueError):
            require_within_envelope(position, (0, 0, 0), (1, 0, 0), .05)
    assert require_within_envelope((.0499, 0, 0), (0, 0, 0), (1, 0, 0), .05)[0] == .0499


@pytest.mark.parametrize('failing_stage', (0, 1, 2))
@pytest.mark.parametrize('exception', (RuntimeError, KeyboardInterrupt))
def test_each_stop_stage_exception_still_attempts_remaining_actions(failing_stage, exception):
    called = []

    def action(index):
        def invoke():
            called.append(index)
            if index == failing_stage:
                raise exception('Injected offline failure')
            return True
        return invoke

    result = best_effort_stop(action(0), action(1), action(2))
    assert called == [0, 1, 2]
    assert result['stop_errors']


def test_refused_or_timed_out_stop_cannot_be_accepted():
    result = best_effort_stop(lambda: None, lambda: False, lambda: True)
    assert result['measured_stop_confirmed']
    assert not result['stop_request_accepted']
    assert result['stop_errors']


def test_final_retreat_or_undershoot_does_not_pass_target_acceptance():
    for displacement in (0, .02, .0496):
        with pytest.raises(ValueError):
            require_target_reached((displacement, 0, 0), (0, 0, 0), (1, 0, 0), .05)
    for displacement in (.0497, .0499, .05):
        require_target_reached((displacement, 0, 0), (0, 0, 0), (1, 0, 0), .05)


def test_queued_old_pose_cannot_pass_final_source_time_fence():
    assert not pose_is_after_fence(99, 100)
    assert not pose_is_after_fence(100, 100)
    assert not pose_is_after_fence(None, 100)
    assert pose_is_after_fence(101, 100)


def test_coarse_one_mm_profile_has_explicit_half_step_final_tolerance():
    require_target_reached((.0496, 0, 0), (0, 0, 0), (1, 0, 0), .05, .0005)
    with pytest.raises(ValueError):
        require_target_reached((.0494, 0, 0), (0, 0, 0), (1, 0, 0), .05, .0005)
    with pytest.raises(ValueError):
        require_target_reached((.04, 0, 0), (0, 0, 0), (1, 0, 0), .05, .01)
