#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from platform_b.force_control import calculate_step, load_force_config


def close(a: float, b: float, tolerance: float = 1e-6) -> bool:
    return abs(a - b) <= tolerance


def main() -> None:
    config = load_force_config()
    assert config["send_robot_motion"] is False
    assert config["mode"] == "preview_only"

    waiting = calculate_step([1, 0, 0], [0, 0, 0], 10, config=config)
    assert waiting.direction is None
    assert waiting.state == "等待轻拉以确定方向"

    increase = calculate_step([5, 0, 0], [0, 0, 0], 10, config=config)
    assert increase.direction == (1.0, 0.0, 0.0)
    assert increase.state == "增加牵引力"
    assert increase.movement_mm[0] < 0

    hold = calculate_step(
        [9.8, 0, 0], [0, 0, 0], 10, direction=(1, 0, 0), config=config
    )
    assert hold.state == "接近目标"
    assert all(close(value, 0) for value in hold.movement_mm)

    decrease = calculate_step(
        [14, 0, 0], [0, 0, 0], 10, direction=(1, 0, 0), config=config
    )
    assert decrease.state == "减小牵引力"
    assert decrease.movement_mm[0] > 0

    limit = calculate_step(
        [90, 0, 0], [0, 0, 0], 10, direction=(1, 0, 0), config=config
    )
    assert limit.state == "超过安全上限"
    assert all(close(value, 0) for value in limit.movement_mm)

    projection = calculate_step(
        [5, 12, 0], [0, 0, 0], 10, direction=(1, 0, 0), config=config
    )
    assert close(projection.traction_force_n, 5)
    assert close(projection.lateral_force_n, 12)
    assert projection.movement_mm[0] < 0
    assert close(projection.movement_mm[1], 0)

    simulated_force = 4.0
    integral = 0.0
    for _ in range(120):
        step = calculate_step(
            [simulated_force, 0, 0],
            [0, 0, 0],
            10,
            direction=(1, 0, 0),
            dt_s=0.1,
            config=config,
            integral_error_n_s=integral,
        )
        integral += step.error_n * 0.1
        simulated_force += -step.movement_mm[0] * 20.0
    assert abs(simulated_force - 10) <= 0.8

    print("PASS：人手只确定方向，机械臂计算方向与人手受力方向相反")
    print("PASS：只用牵引方向上的力追踪目标，侧向力不会冒充牵引力")
    print("PASS：模拟实时闭环能把牵引力稳定到 10 N 附近")
    print("PASS：真机运动发送保持关闭")


if __name__ == "__main__":
    main()
