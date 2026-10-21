"""Offline unit tests for the fail-closed AUBO KWR75 wrench bridge."""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import sys

import pytest


_NODE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "aubo_wrench_transform_node.py"
_SPEC = importlib.util.spec_from_file_location("aubo_wrench_transform_node", _NODE_PATH)
module = importlib.util.module_from_spec(_SPEC)
assert _SPEC.loader is not None
sys.modules[_SPEC.name] = module
_SPEC.loader.exec_module(module)


def _header(frame_id="kwr75_sensor", sec=10, nanosec=0):
    return SimpleNamespace(
        frame_id=frame_id,
        stamp=SimpleNamespace(sec=sec, nanosec=nanosec),
    )


def _sensor(
    force=(1.0, 2.0, 3.0),
    torque=(4.0, 5.0, 6.0),
    frame_id="kwr75_sensor",
    sec=10,
):
    return SimpleNamespace(
        header=_header(frame_id, sec),
        wrench=SimpleNamespace(
            force=SimpleNamespace(x=force[0], y=force[1], z=force[2]),
            torque=SimpleNamespace(x=torque[0], y=torque[1], z=torque[2]),
        ),
    )


def _pose(
    quaternion=(0.0, 0.0, 0.0, 1.0),
    position=(0.1, 0.2, 0.3),
    frame_id="base_link",
    sec=10,
):
    return SimpleNamespace(
        header=_header(frame_id, sec),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=position[0], y=position[1], z=position[2]),
            orientation=SimpleNamespace(
                x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3]
            ),
        ),
    )


def _core(**overrides):
    values = dict(
        q_flange_sensor=(0.0, 0.0, 0.0, 1.0),
        p_flange_sensor_m=(0.0, 0.0, 0.0),
        mounting_verified=True,
        pose_frame_verified=True,
        sensor_timeout_s=0.2,
        pose_timeout_s=0.2,
    )
    values.update(overrides)
    return module.WrenchTransformCore(module.TransformConfiguration(**values))


def _accept_both(core, sensor=None, pose=None, receipt=10.0, ros_now=10.0):
    assert core.accept_sensor(sensor or _sensor(), receipt, ros_now) is None
    assert core.accept_pose(pose or _pose(), receipt, ros_now) is None


def test_identity_transform_preserves_sensor_wrench():
    """The legacy identity mounting still passes its unaltered wrench through."""
    core = _core()
    _accept_both(core)

    result = core.output_if_fresh(10.1)

    assert result is not None
    assert result.stamp == (10, 0)
    assert result.force_base == (1.0, 2.0, 3.0)
    assert result.torque_base_about_flange == (4.0, 5.0, 6.0)


def test_clock_step_keeps_last_sample_only_for_original_receipt_lifetime():
    core = _core()
    _accept_both(core)
    assert core.accept_sensor(_sensor(), 10.01, 11.48) is not None
    assert core.accept_pose(_pose(), 10.01, 11.48) is not None
    assert core.output_if_fresh(10.02).stamp == (10, 0)
    assert core.accept_sensor(_sensor(), 10.19, 11.66) is not None
    assert core.output_if_fresh(10.21) is None
    sensor, pose = _sensor(sec=12), _pose(sec=12)
    _accept_both(core, sensor, pose, receipt=10.53, ros_now=12.0)
    assert core.output_if_fresh(10.54).stamp == (12, 0)


def test_backward_clock_step_and_duplicate_source_stamp():
    core = _core()
    _accept_both(core)
    _accept_both(core, _sensor(sec=9), _pose(sec=9), receipt=10.01, ros_now=9.0)
    assert core.output_if_fresh(10.02).stamp == (9, 0)
    assert core.accept_sensor(_sensor(sec=9), 10.19, 9.18) is not None
    assert core.output_if_fresh(10.22) is None


def test_90_degree_sensor_to_flange_rotation_is_applied_before_base_rotation():
    """Sensor-frame values rotate into flange coordinates before base rotation."""
    half_sqrt_two = math.sqrt(0.5)
    core = _core(q_flange_sensor=(0.0, 0.0, half_sqrt_two, half_sqrt_two))
    _accept_both(
        core,
        sensor=_sensor(force=(1.0, 0.0, 0.0), torque=(0.0, 1.0, 0.0)),
    )

    result = core.output_if_fresh(10.01)

    assert result is not None
    assert result.force_base == pytest_approx((0.0, 1.0, 0.0))
    assert result.torque_base_about_flange == pytest_approx((-1.0, 0.0, 0.0))


def test_lever_arm_shifts_torque_to_flange_origin():
    """A nonzero mounting lever arm adds the expected torque contribution."""
    core = _core(p_flange_sensor_m=(0.0, 0.1, 0.0))
    _accept_both(core, sensor=_sensor(force=(10.0, 0.0, 0.0), torque=(0.0, 0.0, 0.0)))

    result = core.output_if_fresh(10.01)

    assert result is not None
    assert result.force_base == (10.0, 0.0, 0.0)
    assert result.torque_base_about_flange == (0.0, 0.0, -1.0)


def test_stale_receipt_and_stale_source_timestamp_never_publish():
    """Expired receipts and source stamps cannot produce an output."""
    core = _core()
    _accept_both(core)
    assert core.output_if_fresh(10.21) is None

    core = _core()
    assert core.accept_sensor(_sensor(sec=9), 10.0, 10.0) is not None
    assert core.accept_pose(_pose(), 10.0, 10.0) is None
    assert core.output_if_fresh(10.01) is None

    core = _core(sensor_timeout_s=0.1, pose_timeout_s=1.0)
    assert core.accept_sensor(_sensor(sec=9), 10.0, 9.85) is not None


def test_calibration_gate_defaults_fail_closed():
    """The unchanged default calibration flags continue to suppress output."""
    core = module.WrenchTransformCore(
        module.TransformConfiguration(
            q_flange_sensor=(0.0, 0.0, 0.0, 1.0),
            p_flange_sensor_m=(0.0, 0.0, 0.0),
        )
    )
    _accept_both(core)

    assert core.output_if_fresh(10.01) is None


def test_malformed_data_clears_cache_and_never_reuses_prior_wrench():
    """Invalid input clears cached state rather than reusing a prior wrench."""
    core = _core()
    _accept_both(core)
    assert core.output_if_fresh(10.01) is not None

    assert core.accept_sensor(_sensor(force=(math.nan, 0.0, 0.0)), 10.02, 10.02) is not None
    assert core.output_if_fresh(10.03) is None

    assert core.accept_sensor(_sensor(frame_id="wrong_frame"), 10.04, 10.04) is not None
    assert core.accept_pose(_pose(quaternion=(0.0, 0.0, 0.0, 0.0)), 10.04, 10.04) is not None
    assert core.output_if_fresh(10.05) is None


def test_fixed_pose_tare_subtracts_sensor_frame_offsets_before_transform():
    """Configured offsets are removed in sensor coordinates before transformation."""
    core = _core(
        fixed_pose_tare_enabled=True,
        force_bias_sensor_n=(1.0, 2.0, 3.0),
        torque_bias_sensor_nm=(4.0, 5.0, 6.0),
    )
    _accept_both(core, sensor=_sensor(force=(3.0, 2.0, 1.0), torque=(5.0, 7.0, 9.0)))

    result = core.output_if_fresh(10.01)

    assert result is not None
    assert result.force_base == (2.0, 0.0, -2.0)
    assert result.torque_base_about_flange == (1.0, 2.0, 3.0)


def test_fixed_pose_tare_rotates_bias_corrected_sensor_wrench():
    """The corrected sensor wrench is subsequently rotated through the mount."""
    half_sqrt_two = math.sqrt(0.5)
    core = _core(
        fixed_pose_tare_enabled=True,
        q_flange_sensor=(0.0, 0.0, half_sqrt_two, half_sqrt_two),
        force_bias_sensor_n=(1.0, 1.0, 0.0),
    )
    _accept_both(core, sensor=_sensor(force=(2.0, 1.0, 0.0), torque=(0.0, 0.0, 0.0)))

    result = core.output_if_fresh(10.01)

    assert result is not None
    assert result.force_base == pytest.approx((0.0, 1.0, 0.0), abs=1e-12)


def test_force_sensor_sign_follows_bias_correction_before_mount_rotation():
    """Explicit force signs apply after tare correction and before rotation."""
    half_sqrt_two = math.sqrt(0.5)
    core = _core(
        fixed_pose_tare_enabled=True,
        q_flange_sensor=(0.0, 0.0, half_sqrt_two, half_sqrt_two),
        force_bias_sensor_n=(1.0, 0.0, 0.0),
        force_sensor_sign=(-1.0, 1.0, 1.0),
    )
    _accept_both(core, sensor=_sensor(force=(3.0, 0.0, 0.0), torque=(1.0, 2.0, 3.0)))

    result = core.output_if_fresh(10.01)

    assert result is not None
    assert result.force_base == pytest.approx((0.0, -2.0, 0.0), abs=1e-12)
    assert result.torque_base_about_flange == pytest.approx((-2.0, 1.0, 3.0), abs=1e-12)


def test_fixed_pose_tare_treats_opposite_quaternion_sign_as_same_pose():
    """Quaternion sign aliases describe the same accepted fixed tare pose."""
    core = _core(fixed_pose_tare_enabled=True)
    _accept_both(core, pose=_pose(quaternion=(0.0, 0.0, 0.0, -1.0)))

    assert core.output_if_fresh(10.01) is not None


def test_fixed_pose_tare_accepts_small_orientation_drift_with_wider_tolerance():
    """Small controller pose variation stays within the widened tare gate."""
    angle = 0.02
    core = _core(fixed_pose_tare_enabled=True, tare_pose_tolerance_rad=0.03)
    _accept_both(
        core, pose=_pose(quaternion=(0.0, 0.0, math.sin(angle / 2.0), math.cos(angle / 2.0))))

    assert core.output_if_fresh(10.01) is not None


def test_fixed_pose_tare_rejects_orientation_drift_beyond_wider_tolerance():
    """Large orientation drift still invalidates a fixed-pose tare."""
    angle = 0.04
    core = _core(fixed_pose_tare_enabled=True, tare_pose_tolerance_rad=0.03)
    _accept_both(
        core, pose=_pose(quaternion=(0.0, 0.0, math.sin(angle / 2.0), math.cos(angle / 2.0))))

    assert core.output_if_fresh(10.01) is None


@pytest.mark.parametrize(
    "overrides",
    [
        {"force_bias_sensor_n": (math.nan, 0.0, 0.0)},
        {"force_sensor_sign": (1.0, 0.0, 1.0)},
        {"torque_bias_sensor_nm": (0.0, math.inf, 0.0)},
        {"tare_q_base_flange": (0.0, 0.0, 0.0, 0.0)},
        {"tare_pose_tolerance_rad": math.nan},
    ],
)
def test_fixed_pose_tare_rejects_invalid_configuration(overrides):
    """Nonfinite offsets, invalid quaternions, and invalid tolerances are rejected."""
    values = dict(fixed_pose_tare_enabled=True)
    values.update(overrides)
    with pytest.raises(ValueError):
        _core(**values)


def pytest_approx(values):
    """Avoid a pytest runtime dependency inside the bridge module itself."""
    return pytest.approx(values, abs=1e-12)


def test_recorded_pose_does_not_silence_session_managed_wrench():
    """Regression: 20 Oct hand drag reached 0.0359 rad from the *old* tare."""
    options = dict(
        fixed_pose_tare_enabled=True,
        tare_q_base_flange=(-0.3319807820, -0.5099078170, 0.7850861323, 0.1158557012),
        tare_pose_tolerance_rad=0.03,
        force_bias_sensor_n=(-2.98652, 13.09922, 1.40692),
    )
    recorded_pose = _pose(
        quaternion=(-0.3407960066, -0.4961931284, 0.7907836653, 0.1109579012))
    old = _core(**options)
    _accept_both(old, pose=recorded_pose)
    assert old.output_block_reason(10.01) == 'FIXED_TARE_ORIENTATION_MISMATCH'
    assert old.output_if_fresh(10.01) is None

    current = _core(**options, session_managed_tare=True)
    _accept_both(current, pose=recorded_pose)
    result = current.output_if_fresh(10.01)
    assert result is not None
    assert current.output_block_reason(10.01) == ''
    q = current._pose.q_base_flange
    # The manager owns the baseline: no second subtraction of the old offset.
    assert result.force_base == pytest.approx(module.rotate_vector(q, (1.0, 2.0, 3.0)))
    assert current.output_if_fresh(10.3) is None
    assert current.output_block_reason(10.3) == 'SENSOR_SAMPLE_STALE'


def test_session_baseline_keeps_base_gravity_and_external_force_separate():
    """Current-pose base baseline cancels gravity across small pose changes."""
    core = _core(fixed_pose_tare_enabled=True, session_managed_tare=True,
                 force_bias_sensor_n=(9.0, 8.0, 7.0))
    gravity_base = (0.0, 0.0, -13.5)
    external_base = (2.0, -1.0, 3.0)
    _accept_both(core, sensor=_sensor(force=gravity_base))
    baseline = core.output_if_fresh(10.01).force_base
    for index, angle in enumerate((0.02, 0.035903, 0.05), start=1):
        q = (0.0, math.sin(angle / 2.0), 0.0, math.cos(angle / 2.0))
        inverse = (0.0, -q[1], 0.0, q[3])
        total = tuple(a + b for a, b in zip(gravity_base, external_base))
        raw = module.rotate_vector(inverse, total)
        sensor, pose = _sensor(force=raw), _pose(quaternion=q)
        sensor.header.stamp.nanosec = pose.header.stamp.nanosec = index * 1000000
        _accept_both(core, sensor=sensor, pose=pose,
                     receipt=10.0 + index * 0.001, ros_now=10.0 + index * 0.001)
        output = core.output_if_fresh(10.01)
        assert output is not None
        corrected = tuple(a - b for a, b in zip(output.force_base, baseline))
        assert corrected == pytest.approx(external_base)
