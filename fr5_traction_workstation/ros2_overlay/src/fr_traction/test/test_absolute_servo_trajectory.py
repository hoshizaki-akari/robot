"""Targeted offline trajectory regressions; no Node or SDK is constructed."""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

spec = importlib.util.spec_from_file_location(
    "absolute_trajectory_driver",
    Path(__file__).resolve().parents[1] / "scripts/fr5_direct_driver_node.py",
)
driver_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(driver_module)
Driver = driver_module.Fr5DirectDriver


def trajectory_driver():
    velocity = driver_module.Twist()
    velocity.linear.x = 0.020
    driver = SimpleNamespace(
        _native_drag_active=False, _servo_enabled=True,
        _hardware_fault_latched=False, _hardware_emergency_latched=False,
        _next_motion_at=0.0, _last_motion_at=0.0, _motion_period_s=0.01,
        _auto_tension_active=False, _return_active=False,
        _last_command_at=100.0, _command_timeout=0.25,
        _twist=velocity, _max_speed=0.05, _base_servo_sign=1.0,
        _traction_mode=driver_module.TractionCommand.POSITIONING,
        _latest_pose=[200.0, 300.0, 400.0, 180.0, 0.0, 90.0],
        _cartesian_target_pose=None, _max_servo_tracking_error_mm=5.0,
        _tracking_limit_since=None, _tracking_limit_anchor_pose=None,
        packets=[],
    )
    driver._servo_cart = lambda mode, target: driver.packets.append((mode, target)) or 0
    driver.get_logger = lambda: SimpleNamespace(warning=lambda _message: None)
    return driver


def send(driver, now):
    driver._last_command_at = now
    Driver._send_motion(driver, now, 0.01)


def test_constant_velocity_survives_feedback_lag_without_losing_each_increment():
    driver = trajectory_driver()
    # Synthetic actuator follows 25% of its target error per tick. This is a
    # deliberate lag model, not a claim about the FR5 controller internals.
    for tick in range(100):
        send(driver, 100.0 + tick * 0.01)
        mode, target = driver.packets[-1]
        assert mode == 0
        for axis in range(3):
            driver._latest_pose[axis] += 0.25 * (target[axis] - driver._latest_pose[axis])
    assert math.isclose(driver.packets[-1][1][0], 220.0, abs_tol=1e-8)
    assert driver._latest_pose[0] > 219.3
    assert driver.packets[-1][1][3:] == [180.0, 0.0, 90.0]


def test_contact_stop_discards_outstanding_absolute_target_lead():
    driver = trajectory_driver()
    send(driver, 100.0)
    assert driver.packets[-1][1][0] > driver._latest_pose[0]
    driver._twist.linear.x = 0.0
    send(driver, 100.01)
    assert driver.packets[-1] == (0, driver._latest_pose)


def test_unresponsive_actuator_cannot_accumulate_an_unbounded_catch_up_target():
    driver = trajectory_driver()
    with pytest.raises(RuntimeError, match="not following"):
        for tick in range(200):
            send(driver, 100.0 + tick * 0.01)
    assert driver.packets[-1][1][0] - driver._latest_pose[0] <= 5.0


def test_single_search_lag_sample_is_limited_without_false_fault():
    driver = trajectory_driver()
    driver._cartesian_target_pose = list(driver._latest_pose)
    driver._cartesian_target_pose[0] += 4.99
    driver._twist.linear.x = 0.050
    send(driver, 100.0)
    assert math.isclose(driver.packets[-1][1][0] - driver._latest_pose[0], 5.0)
    # Slow but progressing hardware may remain at the lead bound. It must
    # not pile up excess targets or be treated as a stationary obstruction.
    for tick in range(1, 201):
        driver._latest_pose[0] += 0.2
        send(driver, 100.0 + tick * 0.01)
        assert driver.packets[-1][1][0] - driver._latest_pose[0] <= 5.0 + 1e-8


def test_late_callback_drops_missing_motion_instead_of_packing_it_in_one_target():
    driver = trajectory_driver()
    send(driver, 100.0)
    driver._latest_pose = list(driver.packets[-1][1])
    send(driver, 101.0)
    assert math.isclose(driver.packets[-1][1][0], 200.4, abs_tol=1e-9)


def test_search_moves_along_requested_tool_axis_and_stops_at_five_second_budget():
    driver = trajectory_driver()
    driver._traction_mode = driver_module.TractionCommand.DIRECTION_SEARCH
    driver._twist.linear.x, driver._twist.linear.z = 0.0, -0.1125
    driver._direction_search_max_speed = 0.12
    driver._direction_search_started_at = None
    driver._direction_search_start_pose = None
    driver._direction_search_stopped = False
    driver._direction_search_max_duration_s = 5.0
    driver._direction_search_max_travel_m = 0.30
    driver._limit_direction_search = lambda linear, now: Driver._limit_direction_search(
        driver, linear, now
    )
    driver.get_logger = lambda: SimpleNamespace(warning=lambda _message: None)
    send(driver, 100.0)
    assert driver.packets[-1][1][2] < 400.0
    driver._latest_pose = list(driver.packets[-1][1])
    send(driver, 105.0)
    assert driver._direction_search_stopped
    assert driver.packets[-1] == (0, driver._latest_pose)
