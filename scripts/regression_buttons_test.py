import time
from pathlib import Path

from platform_a.calcaneus_robot.device import RealDeviceAdapter
from platform_a.calcaneus_robot.controller import RobotController
from platform_a.calcaneus_robot.models import ControlParameters, PatientCase, ControlState
from platform_a.calcaneus_robot.storage import RecordStore


def notify(message: str) -> None:
    print("EVENT", message, flush=True)


def wait_valid(device: RealDeviceAdapter, label: str, timeout: float = 90.0) -> dict:
    deadline = time.monotonic() + timeout
    result: dict = {}
    while time.monotonic() < deadline:
        result = device.pry_vision.result
        if result.get("valid"):
            print(f"VISION {label} valid width={result.get('width_mm')}", flush=True)
            return dict(result)
        time.sleep(0.5)
    raise RuntimeError(f"{label} vision timeout: {result}")


def main() -> None:
    device = RealDeviceAdapter()
    store = RecordStore(Path("/tmp/platform_a_regression"))
    controller = RobotController(device, store, notify)
    try:
        print("STEP open interface/connect", flush=True)
        controller.connect()
        store.begin_case(PatientCase("REG-PRY-CLAMP-20260820", "P001", "左", "codex"))
        controller.update_parameters(ControlParameters(
            target_pry_mm=50.0, target_clamp_mm=5.0, speed_mm_s=20.0,
            force_limit_n=80.0, torque_limit_nm=8.0, hold_seconds=3.0,
        ))

        print("STEP home before pry", flush=True)
        controller.home()
        print("STEP start pry preview", flush=True)
        controller.start_positioning()
        result = wait_valid(device, "pry")
        result.update({"pry_direction": "Y_MINUS", "pry_angle_deg": 45.0, "pry_lever_arm_mm": 50.0})
        print("STEP start pry motion", flush=True)
        device.start_pry_workflow(result)
        controller.complete_pry_workflow()
        print("STEP home after pry", flush=True)
        controller.home()

        print("STEP start clamp preview", flush=True)
        controller.begin_clamp_preview()
        wait_valid(device, "clamp")
        print("STEP start clamp motion", flush=True)
        # This is the same callback used by the real UI's "启动夹挤"
        # branch; the generic controller method is reserved for simulation.
        device.start_clamp_workflow_v2(controller.params.target_clamp_mm, controller.params.speed_mm_s)
        controller._set_state(ControlState.COMPLETED, "夹挤完成")
        print("STEP home after clamp", flush=True)
        controller.home()
        print("PASS pry_then_clamp", flush=True)
    finally:
        try:
            device.stop_pry_vision()
        except Exception as error:
            print("CLEANUP", error, flush=True)


if __name__ == "__main__":
    main()
