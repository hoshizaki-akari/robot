#!/usr/bin/env python3
"""Safely set the AG-160-95 gripper opening without moving the robot arm."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import serial


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from state_service.ag95_reader import (
    REG_ACTUAL_POSITION,
    REG_GRIP_STATUS,
    REG_INIT_STATUS,
    crc16,
    open_ag95_port,
    read_register,
)


REG_INITIALIZE = 0x0100
REG_FORCE = 0x0101
REG_POSITION = 0x0103
REG_SPEED = 0x0104
MAX_OPENING_MM = 95.0


def write_register(port: serial.Serial, register: int, value: int) -> None:
    payload = bytes((1, 6, register >> 8, register & 0xFF, value >> 8, value & 0xFF))
    request = payload + crc16(payload)
    port.reset_input_buffer()
    port.write(request)
    port.flush()
    response = port.read(8)
    if response != request:
        raise RuntimeError(f"AG95 写入失败：寄存器 0x{register:04X}")


def wait_until(port: serial.Serial, register: int, expected: int, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if read_register(port, register) == expected:
            return
        time.sleep(0.2)
    raise TimeoutError("等待夹爪初始化超时")


def main() -> int:
    parser = argparse.ArgumentParser(description="把 AG95 夹爪调整到指定开度")
    parser.add_argument("opening_mm", type=float, help="目标开度，范围 0～95 mm")
    parser.add_argument("--speed", type=int, default=15, help="速度百分比，默认 15")
    parser.add_argument("--force", type=int, default=20, help="夹力等级 20～100，默认 20")
    parser.add_argument("--yes", action="store_true", help="跳过人工确认")
    args = parser.parse_args()

    if not 0.0 <= args.opening_mm <= MAX_OPENING_MM:
        parser.error("开度必须在 0～95 mm 之间")
    if not 1 <= args.speed <= 100:
        parser.error("速度必须在 1～100 之间")
    if not 20 <= args.force <= 100:
        parser.error("夹力等级必须在 20～100 之间")

    target_raw = round(args.opening_mm / MAX_OPENING_MM * 1000)
    print(f"目标开度：{args.opening_mm:.1f} mm（设备值 {target_raw}/1000）")
    print("只移动夹爪，机械臂不会移动。")
    if not args.yes:
        answer = input("确认夹爪内没有手或物体后，输入 YES：").strip()
        if answer != "YES":
            print("已取消。")
            return 2

    # Wait for the read-only state sampler to release the shared port lock.
    with open_ag95_port(timeout_s=8.0) as (port, _device):
        initialized = read_register(port, REG_INIT_STATUS)
        if initialized != 1:
            print("夹爪正在寻找完全张开的位置……")
            write_register(port, REG_INITIALIZE, 1)
            wait_until(port, REG_INIT_STATUS, 1, 15.0)

        # This is a device force level, not Newtons.
        write_register(port, REG_FORCE, args.force)
        write_register(port, REG_SPEED, args.speed)
        write_register(port, REG_POSITION, target_raw)

        deadline = time.monotonic() + 15.0
        last_raw = None
        stable_count = 0
        while time.monotonic() < deadline:
            position_raw = read_register(port, REG_ACTUAL_POSITION)
            motion_status = read_register(port, REG_GRIP_STATUS)
            stable_count = stable_count + 1 if position_raw == last_raw else 0
            last_raw = position_raw
            if abs(position_raw - target_raw) <= 5 and motion_status != 0 and stable_count >= 2:
                actual_mm = position_raw / 1000.0 * MAX_OPENING_MM
                print(f"已到位：设备值 {position_raw}/1000，约 {actual_mm:.1f} mm")
                return 0
            if motion_status == 2 and abs(position_raw - target_raw) > 5:
                actual_mm = position_raw / 1000.0 * MAX_OPENING_MM
                raise RuntimeError(f"夹爪提前碰到物体，停在约 {actual_mm:.1f} mm")
            time.sleep(0.2)

    raise TimeoutError("夹爪未在规定时间内到达 48 mm 附近")


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, serial.SerialException, RuntimeError, TimeoutError) as error:
        print(f"调整失败：{error}", file=sys.stderr)
        raise SystemExit(1)
