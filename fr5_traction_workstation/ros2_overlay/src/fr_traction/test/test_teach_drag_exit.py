"""Read only SDK state snapshots; never construct a robot or ROS node."""
from types import SimpleNamespace

from test_native_drag_driver import Fr5DirectDriver, OldSdk, fake_driver, module


def setup():
    driver = fake_driver(OldSdk(), [0.0] * 6)
    driver._teach_drag_seen = False
    driver._teach_drag_ignore_until_clear = False
    driver.events = []
    driver._teach_drag_exit_pub = SimpleNamespace(publish=driver.events.append)
    driver.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        to_msg=lambda: module.WrenchStamped().header.stamp))
    driver._robot.robot_state_pkg.robot_state = 1
    return driver


def observe(driver, state, wrench=None):
    driver._robot.robot_state_pkg.robot_state = state
    Fr5DirectDriver._observe_teach_drag_state(driver, driver._robot.robot_state_pkg,
                                             wrench or [20.0, -3.0, 4.0, 0.0, 0.0, 0.0])


def test_exit_event_requires_prior_drag_and_is_emitted_once_with_current_force():
    driver = setup()
    observe(driver, 1)
    assert not driver.events
    observe(driver, 4)
    observe(driver, 4)
    observe(driver, 1)
    observe(driver, 1)
    assert len(driver.events) == 1
    assert driver.events[0].wrench.force.x == 20.0
    assert driver.events[0].header.frame_id == "base_link"


def test_native_drag_and_its_trailing_packets_do_not_trigger_recalibration():
    driver = setup()
    driver._native_drag_active = True
    observe(driver, 4)
    driver._native_drag_active = False
    observe(driver, 4)
    observe(driver, 1)
    assert not driver.events
    observe(driver, 4)
    observe(driver, 1)
    assert len(driver.events) == 1


def test_fault_and_plain_mode_changes_do_not_trigger_recalibration():
    driver = setup()
    driver._robot.robot_state_pkg.robot_mode = 0
    observe(driver, 1)
    driver._robot.robot_state_pkg.robot_mode = 1
    observe(driver, 1)
    assert not driver.events
    observe(driver, 4)
    driver._robot.robot_state_pkg.EmergencyStop = 1
    observe(driver, 1)
    driver._robot.robot_state_pkg.EmergencyStop = 0
    observe(driver, 1)
    assert not driver.events


def test_software_drag_rejects_external_physical_drag_without_motion_commands():
    driver = setup()
    driver._robot.robot_state_pkg.robot_state = 4
    response = SimpleNamespace(success=True, message="")
    Fr5DirectDriver._on_native_drag_start(driver, None, response)
    assert not response.success
    assert response.message.startswith("PHYSICAL_TEACH_ACTIVE:")
    assert not driver._robot.calls
    assert not driver._robot.references


def test_force_gate_cycles_keep_reference_until_final_operator_stop():
    driver = setup()
    response = SimpleNamespace(success=False, message="")
    Fr5DirectDriver._on_native_drag_start(driver, None, response)
    assert response.success
    assert len(driver._robot.references) == 1
    Fr5DirectDriver._on_native_drag_gate_stop(driver, None, response)
    assert response.success
    assert not driver._native_drag_active
    assert driver._force_reference_custom_active
    assert len(driver._robot.references) == 1
    Fr5DirectDriver._on_native_drag_start(driver, None, response)
    assert response.success
    assert len(driver._robot.references) == 1
    Fr5DirectDriver._on_native_drag_stop(driver, None, response)
    assert response.success
    assert not driver._force_reference_custom_active
    assert len(driver._robot.references) == 2
