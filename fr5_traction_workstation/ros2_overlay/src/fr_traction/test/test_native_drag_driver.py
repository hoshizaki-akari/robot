"""Exercise native-drag SDK handoff without connecting to a real FR5."""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace


spec = importlib.util.spec_from_file_location(
    "fr5_direct_driver_node",
    Path(__file__).resolve().parents[1] / "scripts" / "fr5_direct_driver_node.py",
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
Fr5DirectDriver = module.Fr5DirectDriver


class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)

    def warning(self, message):
        self.messages.append(message)

    def error(self, message):
        self.messages.append(message)


class OldSdk:
    def __init__(self):
        self.drag_state = 0
        self.auto_flags = []
        self.calls = []
        self.references = []
        self.servo_end_calls = 0
        self.servo_start_calls = 0
        self.robot_state_pkg = SimpleNamespace(
            frame_cnt=1,
            EmergencyStop=0,
            rbtEnableState=1,
            robot_mode=1,
            ft_sensor_active=1,
            main_code=0,
            sub_code=0,
        )

    def ServoMoveEnd(self):
        self.servo_end_calls += 1
        return 0

    def ServoMoveStart(self):
        self.servo_start_calls += 1
        return 0

    def FT_SetRCS(self, ref, coord):
        self.references.append((ref, list(coord)))
        return 0

    def SetForceSensorDragAutoFlag(self, status):
        self.auto_flags.append(status)
        return 0

    def EndForceDragControl(
        self, status, adaptive, interference, mass, damping, stiffness,
        threshold, maximum_force, maximum_speed,
    ):
        self.calls.append((status, list(threshold), maximum_speed))
        self.drag_state = status
        return 0

    def GetForceAndTorqueDragState(self):
        return 0, self.drag_state, 0


class NewSdk(OldSdk):
    def EndForceDragControl(
        self, status, adaptive, interference, singularity, collision,
        mass, damping, stiffness, threshold, maximum_force, maximum_speed,
    ):
        self.calls.append((status, list(threshold), maximum_speed))
        self.drag_state = status
        return 0


class StopFailureSdk(OldSdk):
    def EndForceDragControl(
        self, status, adaptive, interference, mass, damping, stiffness,
        threshold, maximum_force, maximum_speed,
    ):
        if status == 0:
            return 1
        return super().EndForceDragControl(
            status, adaptive, interference, mass, damping, stiffness,
            threshold, maximum_force, maximum_speed,
        )


def fake_driver(sdk, raw_force):
    driver = SimpleNamespace(
        _robot=sdk,
        _native_drag_mass=[8.0, 8.0, 8.0, 0.5, 0.5, 0.1],
        _native_drag_damping=[120.0, 120.0, 120.0, 5.0, 5.0, 1.0],
        _native_drag_stiffness=[0.0] * 6,
        _native_drag_threshold=[5.0] * 6,
        _native_drag_max_force_n=50.0,
        _native_drag_max_joint_speed_deg_s=50.0,
        _startup_rpc_timeout_s=10.0,
        _native_drag_active=False,
        _servo_cleanup_pending=False,
        _native_drag_tool_xy_flip=True,
        _force_reference_custom_active=False,
        _servo_enabled=False,
        _return_active=False,
        _auto_tension_active=False,
        _hardware_fault_latched=False,
        _hardware_emergency_latched=False,
        _latest_pose=[0.0] * 6,
        _latest_joints=[0.0] * 6,
        _latest_wrench=list(raw_force),
        _last_realtime_state_at=module.time.monotonic(),
        _realtime_state_timeout_s=0.5,
    )
    logger = FakeLogger()
    driver.get_logger = lambda: logger
    driver._switch_lists = Fr5DirectDriver._switch_lists
    driver._reset_command_proxy = lambda timeout_s=None: None
    def hardware_command(command, *args):
        if command == "Mode":
            sdk.robot_state_pkg.robot_mode = args[0]
            sdk.robot_state_pkg.frame_cnt += 1
        if command == "RobotEnable":
            sdk.robot_state_pkg.rbtEnableState = args[0]
        return 0, ""

    driver._run_hardware_command = hardware_command
    driver._publish_health = lambda healthy: None
    driver._set_native_drag = lambda enabled: Fr5DirectDriver._set_native_drag(driver, enabled)
    driver._end_servo = lambda context: Fr5DirectDriver._end_servo(driver, context)
    driver._robot_motion_readiness_error = lambda expected_mode=None: (
        Fr5DirectDriver._robot_motion_readiness_error(driver, expected_mode)
    )
    driver._select_motion_mode = lambda mode, force=False: (
        Fr5DirectDriver._select_motion_mode(driver, mode, force)
    )
    driver._set_drag_force_reference = lambda custom: (
        Fr5DirectDriver._set_drag_force_reference(driver, custom)
    )
    driver._native_drag_state = lambda: Fr5DirectDriver._native_drag_state(driver)
    driver._wait_native_drag_state = lambda expected: (
        Fr5DirectDriver._wait_native_drag_state(driver, expected)
    )
    driver._stop_native_drag_best_effort = lambda context: (
        Fr5DirectDriver._stop_native_drag_best_effort(driver, context)
    )
    driver._disable_after_failed_native_stop = lambda context: (
        Fr5DirectDriver._disable_after_failed_native_stop(driver, context)
    )
    return driver


def test_old_sdk_uses_vendor_threshold_independent_of_raw_idle_bias():
    sdk = OldSdk()
    driver = fake_driver(sdk, [1.5, -2.7, 0.2, 0.0, 0.0, 0.0])
    response = SimpleNamespace(success=False, message="")
    Fr5DirectDriver._on_native_drag_start(driver, None, response)
    assert response.success
    assert sdk.auto_flags == [0]
    assert sdk.calls[0][1][:3] == [5.0, 5.0, 5.0]
    assert sdk.drag_state == 1
    assert sdk.references == [(2, [0.0, 0.0, 0.0, 0.0, 0.0, 180.0])]
    Fr5DirectDriver._on_native_drag_stop(driver, None, response)
    assert response.success
    assert sdk.drag_state == 0
    assert not driver._native_drag_active
    assert sdk.references[-1] == (1, [0.0] * 6)


def test_new_sdk_accepts_extra_controller_flags():
    sdk = NewSdk()
    driver = fake_driver(sdk, [0.0] * 6)
    response = SimpleNamespace(success=False, message="")
    Fr5DirectDriver._on_native_drag_start(driver, None, response)
    assert response.success
    assert sdk.calls[0][1][:3] == [5.0, 5.0, 5.0]


def test_raw_sensor_payload_does_not_fault_drag_at_start():
    sdk = OldSdk()
    driver = fake_driver(sdk, [12.0, -9.0, 0.0, 0.0, 0.0, 0.0])
    response = SimpleNamespace(success=False, message="")
    Fr5DirectDriver._on_native_drag_start(driver, None, response)
    assert response.success
    assert sdk.calls[0][1][:3] == [5.0, 5.0, 5.0]
    assert sdk.drag_state == 1


def test_nonfinite_sensor_data_still_rejects_drag():
    sdk = OldSdk()
    driver = fake_driver(sdk, [math.nan, 0.0, 0.0, 0.0, 0.0, 0.0])
    response = SimpleNamespace(success=False, message="")
    Fr5DirectDriver._on_native_drag_start(driver, None, response)
    assert not response.success
    assert not sdk.calls


def test_failed_native_stop_disables_robot_instead_of_reporting_ready():
    sdk = StopFailureSdk()
    driver = fake_driver(sdk, [0.0] * 6)
    response = SimpleNamespace(success=False, message="")
    hardware_commands = []
    driver._run_hardware_command = lambda command, *args: (
        hardware_commands.append((command, args)) or (0, "")
    )
    Fr5DirectDriver._on_native_drag_start(driver, None, response)
    assert response.success
    Fr5DirectDriver._on_native_drag_stop(driver, None, response)
    assert not response.success
    assert driver._hardware_emergency_latched
    assert ("RobotEnable", (0,)) in hardware_commands


def test_drag_reference_rotation_preserves_tool_z_and_base_feedback():
    convert = module._custom_drag_wrench_to_base
    assert convert([1.0, 2.0, 3.0, 4.0, 5.0, 6.0], [0.0] * 6) == [
        -1.0, -2.0, 3.0, -4.0, -5.0, 6.0,
    ]
    rotated = convert([1.0, 2.0, 3.0, 4.0, 5.0, 6.0], [0, 0, 0, 0, 0, 90])
    assert all(abs(a - b) < 1e-9 for a, b in zip(rotated, [2, -1, 3, 5, -4, 6]))


def test_reference_switch_failure_rejects_drag_and_attempts_base_restore():
    class RejectCustomReference(OldSdk):
        def FT_SetRCS(self, ref, coord):
            super().FT_SetRCS(ref, coord)
            return 42 if ref == 2 else 0

    sdk = RejectCustomReference()
    driver = fake_driver(sdk, [0.0] * 6)
    response = SimpleNamespace(success=False, message="")
    Fr5DirectDriver._on_native_drag_start(driver, None, response)
    assert not response.success
    assert sdk.drag_state == 0
    assert [ref for ref, _ in sdk.references] == [2, 1]
    assert not driver._force_reference_custom_active


def test_disabling_xy_flip_keeps_existing_base_reference():
    sdk = OldSdk()
    driver = fake_driver(sdk, [0.0] * 6)
    driver._native_drag_tool_xy_flip = False
    response = SimpleNamespace(success=False, message="")
    Fr5DirectDriver._on_native_drag_start(driver, None, response)
    assert response.success
    assert sdk.references == []
    Fr5DirectDriver._on_native_drag_stop(driver, None, response)
    assert response.success
    assert sdk.references == []


def test_base_restore_failure_reports_error_and_latches_stop():
    class RejectBaseReference(OldSdk):
        def FT_SetRCS(self, ref, coord):
            super().FT_SetRCS(ref, coord)
            return 43 if ref == 1 else 0

    sdk = RejectBaseReference()
    driver = fake_driver(sdk, [0.0] * 6)
    response = SimpleNamespace(success=False, message="")
    Fr5DirectDriver._on_native_drag_start(driver, None, response)
    assert response.success
    Fr5DirectDriver._on_native_drag_stop(driver, None, response)
    assert not response.success
    assert driver._hardware_emergency_latched
    assert driver._force_reference_custom_active


def test_emergency_recovery_closes_interrupted_servo_before_restarting_drag():
    sdk = OldSdk()
    driver = fake_driver(sdk, [0.0] * 6)
    driver._servo_enabled = True
    driver._servo_cleanup_pending = True
    response = SimpleNamespace(success=False, message="")
    Fr5DirectDriver._on_hardware_emergency_stop(driver, None, response)
    assert response.success
    assert driver._servo_cleanup_pending
    assert not driver._servo_enabled
    Fr5DirectDriver._on_hardware_emergency_recover(driver, None, response)
    assert response.success
    assert sdk.servo_end_calls == 1
    assert not driver._servo_cleanup_pending
    assert sdk.robot_state_pkg.robot_mode == 1
    assert sdk.robot_state_pkg.rbtEnableState == 1
    Fr5DirectDriver._on_native_drag_start(driver, None, response)
    assert response.success
    assert sdk.drag_state == 1


def test_servo_traction_switches_auto_and_next_native_drag_returns_manual():
    sdk = OldSdk()
    driver = fake_driver(sdk, [0.0] * 6)
    activate = SimpleNamespace(
        activate_controllers=["cartesian_velocity_controller"],
        start_controllers=[], deactivate_controllers=[], stop_controllers=[],
    )
    switch_response = SimpleNamespace(ok=False)
    Fr5DirectDriver._on_switch(driver, activate, switch_response)
    assert switch_response.ok
    assert sdk.robot_state_pkg.robot_mode == 0
    assert sdk.servo_start_calls == 1
    deactivate = SimpleNamespace(
        activate_controllers=[], start_controllers=[],
        deactivate_controllers=["cartesian_velocity_controller"], stop_controllers=[],
    )
    Fr5DirectDriver._on_switch(driver, deactivate, switch_response)
    assert switch_response.ok
    assert sdk.servo_end_calls == 1
    drag_response = SimpleNamespace(success=False, message="")
    Fr5DirectDriver._on_native_drag_start(driver, None, drag_response)
    assert drag_response.success
    assert sdk.robot_state_pkg.robot_mode == 1


def test_recovery_forces_manual_command_even_with_stale_manual_feedback():
    sdk = OldSdk()
    driver = fake_driver(sdk, [0.0] * 6)
    driver._hardware_emergency_latched = True
    original_command = driver._run_hardware_command
    commands = []

    def delayed_mode_feedback(command, *args):
        commands.append((command, args))
        if command == "Mode" and args == (0,):
            # The realtime packet can still show manual after Mode(0) returns.
            return 0, ""
        return original_command(command, *args)

    driver._run_hardware_command = delayed_mode_feedback
    response = SimpleNamespace(success=False, message="")
    Fr5DirectDriver._on_hardware_emergency_recover(driver, None, response)
    assert response.success
    assert ("Mode", (1,)) in commands
    assert sdk.robot_state_pkg.robot_mode == 1


def test_failed_servo_cleanup_keeps_emergency_latched():
    class FailingServoEndSdk(OldSdk):
        def ServoMoveEnd(self):
            self.servo_end_calls += 1
            return 42

    sdk = FailingServoEndSdk()
    driver = fake_driver(sdk, [0.0] * 6)
    driver._servo_cleanup_pending = True
    driver._hardware_emergency_latched = True
    commands = []
    driver._run_hardware_command = lambda command, *args: (
        commands.append((command, args)) or (0, "")
    )
    response = SimpleNamespace(success=False, message="")
    Fr5DirectDriver._on_hardware_emergency_recover(driver, None, response)
    assert not response.success
    assert driver._hardware_emergency_latched
    assert driver._servo_cleanup_pending
    assert ("RobotEnable", (0,)) in commands


def test_drag_rejects_unenabled_robot_instead_of_false_ready():
    sdk = OldSdk()
    sdk.robot_state_pkg.rbtEnableState = 0
    driver = fake_driver(sdk, [0.0] * 6)
    response = SimpleNamespace(success=False, message="")
    Fr5DirectDriver._on_native_drag_start(driver, None, response)
    assert not response.success
    assert "enable" in response.message
    assert sdk.drag_state == 0


def test_recovery_reactivates_inactive_force_sensor_before_drag():
    sdk = OldSdk()
    sdk.robot_state_pkg.ft_sensor_active = 0
    driver = fake_driver(sdk, [0.0] * 6)
    driver._hardware_emergency_latched = True
    commands = []

    def hardware_command(command, *args):
        commands.append((command, args))
        if command == "Mode":
            sdk.robot_state_pkg.robot_mode = args[0]
            sdk.robot_state_pkg.frame_cnt += 1
        if command == "RobotEnable":
            sdk.robot_state_pkg.rbtEnableState = args[0]
        if command == "FT_Activate" and args == (1,):
            sdk.robot_state_pkg.ft_sensor_active = 1
        return 0, ""

    driver._run_hardware_command = hardware_command
    response = SimpleNamespace(success=False, message="")
    Fr5DirectDriver._on_hardware_emergency_recover(driver, None, response)
    assert response.success
    assert ("FT_Activate", (1,)) in commands
    Fr5DirectDriver._on_native_drag_start(driver, None, response)
    assert response.success
