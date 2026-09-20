"""Exercise native-drag SDK handoff without connecting to a real FR5."""

import importlib.util
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
        _native_drag_threshold=[3.0, 3.0, 3.0, 5.0, 5.0, 5.0],
        _native_drag_effective_threshold=[3.0, 3.0, 3.0, 5.0, 5.0, 5.0],
        _native_drag_bias_margin_n=2.0,
        _native_drag_max_force_n=50.0,
        _native_drag_max_joint_speed_deg_s=50.0,
        _native_drag_active=False,
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
    driver._reset_command_proxy = lambda: None
    driver._run_hardware_command = lambda command, *args: (0, "")
    driver._publish_health = lambda healthy: None
    driver._set_native_drag = lambda enabled: Fr5DirectDriver._set_native_drag(driver, enabled)
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


def test_old_sdk_idle_bias_does_not_arm_below_threshold():
    sdk = OldSdk()
    driver = fake_driver(sdk, [1.5, -2.7, 0.2, 0.0, 0.0, 0.0])
    response = SimpleNamespace(success=False, message="")
    Fr5DirectDriver._on_native_drag_start(driver, None, response)
    assert response.success
    assert sdk.auto_flags == [0]
    assert sdk.calls[0][1][:3] == [3.5, 4.7, 3.0]
    assert sdk.drag_state == 1
    Fr5DirectDriver._on_native_drag_stop(driver, None, response)
    assert response.success
    assert sdk.drag_state == 0
    assert not driver._native_drag_active


def test_new_sdk_accepts_extra_controller_flags():
    sdk = NewSdk()
    driver = fake_driver(sdk, [0.0] * 6)
    response = SimpleNamespace(success=False, message="")
    Fr5DirectDriver._on_native_drag_start(driver, None, response)
    assert response.success
    assert sdk.calls[0][1][:3] == [3.0, 3.0, 3.0]


def test_large_stationary_sensor_load_rejects_drag():
    sdk = OldSdk()
    driver = fake_driver(sdk, [9.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    response = SimpleNamespace(success=False, message="")
    Fr5DirectDriver._on_native_drag_start(driver, None, response)
    assert not response.success
    assert not sdk.calls
    assert sdk.drag_state == 0


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
