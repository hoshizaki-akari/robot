"""Offline unit tests for the fail-closed AUBO KWR75 wrench bridge."""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import sys


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
    core = _core()
    _accept_both(core)

    result = core.output_if_fresh(10.1)

    assert result is not None
    assert result.stamp == (10, 0)
    assert result.force_base == (1.0, 2.0, 3.0)
    assert result.torque_base_about_flange == (4.0, 5.0, 6.0)


def test_90_degree_sensor_to_flange_rotation_is_applied_before_base_rotation():
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
    core = _core(p_flange_sensor_m=(0.0, 0.1, 0.0))
    _accept_both(core, sensor=_sensor(force=(10.0, 0.0, 0.0), torque=(0.0, 0.0, 0.0)))

    result = core.output_if_fresh(10.01)

    assert result is not None
    assert result.force_base == (10.0, 0.0, 0.0)
    assert result.torque_base_about_flange == (0.0, 0.0, -1.0)


def test_stale_receipt_and_stale_source_timestamp_never_publish():
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
    core = module.WrenchTransformCore(
        module.TransformConfiguration(
            q_flange_sensor=(0.0, 0.0, 0.0, 1.0),
            p_flange_sensor_m=(0.0, 0.0, 0.0),
        )
    )
    _accept_both(core)

    assert core.output_if_fresh(10.01) is None


def test_malformed_data_clears_cache_and_never_reuses_prior_wrench():
    core = _core()
    _accept_both(core)
    assert core.output_if_fresh(10.01) is not None

    assert core.accept_sensor(_sensor(force=(math.nan, 0.0, 0.0)), 10.02, 10.02) is not None
    assert core.output_if_fresh(10.03) is None

    assert core.accept_sensor(_sensor(frame_id="wrong_frame"), 10.04, 10.04) is not None
    assert core.accept_pose(_pose(quaternion=(0.0, 0.0, 0.0, 0.0)), 10.04, 10.04) is not None
    assert core.output_if_fresh(10.05) is None


def pytest_approx(values):
    """Avoid a pytest runtime dependency inside the bridge module itself."""
    import pytest

    return pytest.approx(values, abs=1e-12)
