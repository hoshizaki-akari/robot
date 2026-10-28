"""Recover successful delayed servo commands with an in-memory SDK, without FR5 IO."""

import math
from types import SimpleNamespace

import pytest

from test_native_drag_driver import Fr5DirectDriver, OldSdk, module, motion_driver


class TimedSdk(OldSdk):
    def __init__(self, clock, delay=1.007, code=0, stop_code=0, transport_error=False):
        super().__init__()
        self.clock = clock
        self.delay = delay
        self.code = code
        self.stop_code = stop_code
        self.transport_error = transport_error
        self.packets = []

    def ServoCart(self, mode, pose, cmdT):
        self.packets.append((mode, list(pose), cmdT))
        self.clock.now += self.delay
        if self.transport_error:
            raise TimeoutError("synthetic command transport timeout")
        return self.code

    def ServoMoveEnd(self):
        self.servo_end_calls += 1
        return self.stop_code


def timing_driver(monkeypatch, delay=1.007, code=0, stop_code=0, transport_error=False):
    clock = SimpleNamespace(now=100.0)
    monkeypatch.setattr(module.time, "monotonic", lambda: clock.now)
    driver = motion_driver(module.TractionCommand.POSITIONING, [0.020, 0.0, 0.0])
    sdk = TimedSdk(clock, delay, code, stop_code, transport_error)
    driver._robot = sdk
    driver._servo_cleanup_pending = True
    driver._last_tick = clock.now
    driver._motion_rate_hz = 100.0
    driver._motion_statistics_since = 0.0
    driver._motion_statistics_packets = 0
    driver._motion_statistics_nonzero_packets = 0
    driver._motion_statistics_max_rpc_s = 0.0
    driver._tracking_limit_since = None
    driver._tracking_limit_anchor_pose = None
    driver.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=int(clock.now * 1_000_000_000))
    )
    driver.health_events = []
    driver._publish_health = driver.health_events.append
    driver._servo_cart = lambda mode, target: Fr5DirectDriver._servo_cart(driver, mode, target)
    driver._send_motion = lambda now, dt: Fr5DirectDriver._send_motion(driver, now, dt)
    driver._restart_motion_cadence_after_delay = lambda elapsed: (
        Fr5DirectDriver._restart_motion_cadence_after_delay(driver, elapsed)
    )
    driver._limit_motion_recovery_velocity = lambda linear, dt: (
        Fr5DirectDriver._limit_motion_recovery_velocity(driver, linear, dt)
    )
    driver._on_twist = lambda message: Fr5DirectDriver._on_twist(driver, message)
    driver._disable_after_failed_servo_stop = lambda context: (
        Fr5DirectDriver._disable_after_failed_servo_stop(driver, context)
    )
    return driver, sdk, clock


def stamped_velocity(driver, clock, velocity=(0.020, 0.0, 0.0), source_ns=None):
    message = module.TwistStamped()
    stamp = int(clock.now * 1_000_000_000) if source_ns is None else source_ns
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    message.header.frame_id = "base_link"
    message.twist.linear.x, message.twist.linear.y, message.twist.linear.z = velocity
    Fr5DirectDriver._on_stamped_twist(driver, message)


def test_successful_one_second_rpc_keeps_traction_and_discards_old_target(monkeypatch):
    driver, sdk, clock = timing_driver(monkeypatch)
    previous_mode = driver._traction_mode

    Fr5DirectDriver._tick(driver)

    assert driver._servo_enabled
    assert driver._servo_cleanup_pending
    assert driver._traction_mode == previous_mode
    assert sdk.servo_end_calls == 0
    assert not driver._hardware_fault_latched
    assert not driver._hardware_emergency_latched
    assert driver.health_events == []
    assert driver._motion_recovery_generation == 1
    assert driver._motion_recovery_active
    assert driver._last_command_at == 0.0
    assert driver._velocity_epoch_ns == int(clock.now * 1_000_000_000)
    assert driver._cartesian_target_pose == driver._latest_pose
    assert sdk.packets[0][1] != driver._cartesian_target_pose


def test_delayed_velocity_is_ignored_and_fresh_velocity_resumes_gently(monkeypatch):
    driver, sdk, clock = timing_driver(monkeypatch)
    Fr5DirectDriver._tick(driver)
    sdk.delay = 0.0
    stamped_velocity(driver, clock, source_ns=100_000_000_000)
    assert driver._last_command_at == 0.0
    clock.now += 0.01
    Fr5DirectDriver._tick(driver)
    assert sdk.packets[-1][1] == driver._latest_pose

    previous = 0.0
    for _tick in range(30):
        clock.now += 0.01
        stamped_velocity(driver, clock)
        Fr5DirectDriver._tick(driver)
        current = driver._motion_recovery_velocity[0]
        assert 0.0 <= current <= 0.020 + 1e-12
        assert current - previous <= 0.08 * 0.01 + 1e-12
        if previous == 0.0:
            assert math.isclose(current, 0.0008)
        driver._latest_pose = list(sdk.packets[-1][1])
        previous = current
    assert math.isclose(previous, 0.020)
    assert sdk.servo_end_calls == 0
    assert driver._servo_enabled


@pytest.mark.parametrize("stop_kind", ["zero", "disabled", "stale", "search_contact"])
def test_required_zero_velocity_is_immediate_during_recovery(monkeypatch, stop_kind):
    driver, sdk, clock = timing_driver(monkeypatch)
    Fr5DirectDriver._tick(driver)
    sdk.delay = 0.0
    driver._motion_recovery_velocity = [0.020, 0.0, 0.0]
    driver._cartesian_target_pose = [3.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    clock.now += 0.01
    stamped_velocity(driver, clock)
    if stop_kind == "zero":
        driver._twist.linear.x = 0.0
    elif stop_kind == "disabled":
        driver._traction_mode = module.TractionCommand.DISABLED
    elif stop_kind == "stale":
        driver._last_command_at = clock.now - 1.0
    else:
        driver._traction_mode = module.TractionCommand.DIRECTION_SEARCH
        # This is the driver's final safety stop when the manager has reached
        # contact / the original search budget; it must bypass recovery slew.
        driver._direction_search_stopped = True

    Fr5DirectDriver._tick(driver)

    assert driver._motion_recovery_velocity == [0.0, 0.0, 0.0]
    assert sdk.packets[-1][1] == driver._latest_pose
    assert driver._cartesian_target_pose == driver._latest_pose
    assert driver._servo_enabled
    assert sdk.servo_end_calls == 0


def test_repeated_successful_delays_reanchor_without_replaying_displacement(monkeypatch):
    driver, sdk, clock = timing_driver(monkeypatch)
    for expected_generation in range(1, 4):
        if expected_generation > 1:
            clock.now += 0.01
            stamped_velocity(driver, clock)
        Fr5DirectDriver._tick(driver)
        assert driver._motion_recovery_generation == expected_generation
        assert driver._cartesian_target_pose == driver._latest_pose
        assert driver._motion_recovery_velocity == [0.0, 0.0, 0.0]
        assert driver._last_command_at == 0.0
        assert driver._servo_enabled
        assert not driver._hardware_fault_latched
    assert len(sdk.packets) == 3
    # The delayed time never contributes v * 1.007 s to a target packet.
    assert max(abs(packet[1][0]) for packet in sdk.packets) <= 0.2 + 1e-12
    assert sdk.servo_end_calls == 0


def test_search_delay_keeps_mode_axis_and_original_search_budget(monkeypatch):
    driver, sdk, clock = timing_driver(monkeypatch)
    driver._traction_mode = module.TractionCommand.DIRECTION_SEARCH
    driver._twist.linear.x = 0.0
    driver._twist.linear.z = -0.1125
    original_start = clock.now - 1.0
    original_origin = [0.0, 0.0, 0.0]
    driver._direction_search_started_at = original_start
    driver._direction_search_start_pose = list(original_origin)

    Fr5DirectDriver._tick(driver)

    assert driver._traction_mode == module.TractionCommand.DIRECTION_SEARCH
    assert driver._direction_search_started_at == original_start
    assert driver._direction_search_start_pose == original_origin
    assert not driver._direction_search_stopped
    assert sdk.packets[0][1][2] < 0.0
    sdk.delay = 0.0
    clock.now += 0.01
    stamped_velocity(driver, clock, (0.0, 0.0, -0.1125))
    Fr5DirectDriver._tick(driver)
    assert sdk.packets[-1][1][2] == pytest.approx(-0.008)
    assert sdk.packets[-1][1][:2] == [0.0, 0.0]

    clock.now = original_start + 5.0
    stamped_velocity(driver, clock, (0.0, 0.0, -0.1125))
    Fr5DirectDriver._tick(driver)
    assert driver._direction_search_stopped
    assert driver._direction_search_started_at == original_start
    assert sdk.packets[-1][1] == driver._latest_pose


def test_recovery_bounds_vector_acceleration_when_direction_changes(monkeypatch):
    driver, _sdk, _clock = timing_driver(monkeypatch)
    driver._motion_recovery_velocity = [0.020, 0.0, 0.0]
    output = Fr5DirectDriver._limit_motion_recovery_velocity(
        driver, [0.0, 0.020, 0.0], 0.01
    )
    change = math.sqrt((output[0] - 0.020) ** 2 + output[1] ** 2 + output[2] ** 2)
    assert change == pytest.approx(0.0008)


def test_search_to_target_handover_discards_recovery_speed_before_first_target_packet(monkeypatch):
    driver, sdk, clock = timing_driver(monkeypatch)
    Fr5DirectDriver._tick(driver)
    sdk.delay = 0.0
    driver._traction_mode = module.TractionCommand.DIRECTION_SEARCH
    driver._motion_recovery_velocity = [0.0, 0.0, -0.100]
    command = module.TractionCommand()
    command.mode = module.TractionCommand.POSITIONING
    stamp = int(clock.now * 1_000_000_000)
    command.header.stamp.sec, command.header.stamp.nanosec = divmod(stamp, 1_000_000_000)

    Fr5DirectDriver._on_traction_command(driver, command)

    assert driver._motion_recovery_velocity == [0.0, 0.0, 0.0]
    clock.now += 0.01
    stamped_velocity(driver, clock)
    Fr5DirectDriver._tick(driver)
    assert driver._motion_recovery_velocity == pytest.approx([0.0008, 0.0, 0.0])
    assert sdk.packets[-1][1][:3] == pytest.approx([0.008, 0.0, 0.0])


@pytest.mark.parametrize("transport_error", [False, True])
def test_failed_slow_command_keeps_hard_fault(monkeypatch, transport_error):
    driver, sdk, _clock = timing_driver(monkeypatch, code=14, transport_error=transport_error)

    Fr5DirectDriver._tick(driver)

    assert driver._hardware_fault_latched
    assert not driver._servo_enabled
    assert sdk.servo_end_calls == 1
    assert False in driver.health_events


def test_real_stop_failure_still_disables_robot(monkeypatch):
    driver, sdk, _clock = timing_driver(monkeypatch, code=14, stop_code=14)
    commands = []
    driver._run_hardware_command = lambda command, *args: (
        commands.append((command, args)) or (0, "")
    )

    Fr5DirectDriver._tick(driver)

    assert sdk.servo_end_calls == 2
    assert driver._hardware_fault_latched
    assert driver._hardware_emergency_latched
    assert driver._servo_cleanup_pending
    assert not driver._servo_enabled
    assert ("StopMotion", ()) in commands
    assert ("RobotEnable", (0,)) in commands


def test_normal_successful_rpc_does_not_change_motion_cadence(monkeypatch):
    driver, sdk, _clock = timing_driver(monkeypatch, delay=0.010)

    Fr5DirectDriver._tick(driver)

    assert driver._servo_enabled
    assert not driver._motion_recovery_active
    assert not driver._hardware_fault_latched
    assert driver._cartesian_target_pose is not None
    assert sdk.servo_end_calls == 0
