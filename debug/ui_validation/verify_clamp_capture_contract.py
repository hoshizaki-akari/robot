"""Non-hardware regression check for the UI clamp capture contract."""

from platform_b import gateway
from platform_a.clamp_planner import build_clamp_plan


def main() -> None:
    transformed = build_clamp_plan(
        {
            "valid": True,
            "clamp_contact_a_camera_mm": [90.0, 200.0, 300.0],
            "clamp_contact_b_camera_mm": [110.0, 200.0, 300.0],
            "heel_plane_point_camera_mm": [100.0, 200.0, 300.0],
            "heel_plane_normal_camera": [0.0, 0.0, 1.0],
        },
        {
            "valid": True,
            "age_ms": 0,
            "flange_pose_mm_deg": [278.98, -217.54, 435.02, -166.01, -3.48, -139.14],
        },
    )
    assert transformed["valid"] is True
    assert transformed["clamp_contact_center_camera_mm"] == [100.0, 200.0, 300.0]

    original_plan = gateway.clamp_plan
    original_launch = gateway.launch_workflow
    try:
        source = {
            "valid": True,
            "clamp_contact_center_camera_mm": [101.0, 202.0, 303.0],
            "heel_width_mm": 57.5,
        }
        gateway.clamp_plan = lambda: dict(source)
        captured = gateway.capture_clamp_plan()
        assert captured["captured"] is True
        assert captured["capture_id"]

        # The execution entry must never re-read a newer live detection.
        gateway.clamp_plan = lambda: (_ for _ in ()).throw(
            AssertionError("move must not re-read live vision")
        )
        submitted: dict[str, object] = {}
        gateway.launch_workflow = lambda command, branch, label: submitted.update(
            command=command, branch=branch, label=label
        ) or submitted
        result = gateway.move_clamp(
            gateway.ClampMotionRequest(
                confirmed_clear=True,
                confirm_text="确认运动",
                capture_id=captured["capture_id"],
                clamp_mm=5.0,
                speed_mm_s=10.0,
            )
        )
        command = result["command"]
        assert "101.00000,202.00000,303.00000" in command
        assert "57.50000" in command
        print("PASS: frozen capture is the only source for clamp execution arguments")
    finally:
        gateway.clamp_plan = original_plan
        gateway.launch_workflow = original_launch


if __name__ == "__main__":
    main()
