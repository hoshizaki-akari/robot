#!/usr/bin/env python3
from __future__ import annotations

import json
import time
from math import sqrt
from urllib.error import URLError
from urllib.request import urlopen


URL = "http://127.0.0.1:8765/api/state"


def read_state() -> dict:
    with urlopen(URL, timeout=1.0) as response:
        return json.load(response)


def main() -> int:
    print("KWR75D 力传感器观察")
    print("这个程序只看数据，不会控制机械臂。")
    print("停止观察时按 Ctrl+C。\n")

    try:
        first = read_state()
    except (OSError, ValueError, URLError) as error:
        print(f"没有找到数据服务：{error}")
        print("请先启动第一个窗口中的数据服务。")
        return 1

    if first.get("source") != "real":
        print("现在显示的还是电脑生成的演示数字，不是真实传感器。")
        print("请停止第一个窗口，重新启动数据服务并选择 2。")
        return 2

    sensor = first["kwr75d"]
    print(f"控制器中的传感器设置：{sensor.get('config', '未读取到')}")
    print(f"传感器是否已经启用：{'是' if sensor.get('active') else '否'}")
    if not sensor.get("active"):
        print("\n原因已经找到：机器人控制器中的力传感器没有启用。")
        print("请先不要校零或修改型号，把机器人网页中的力传感器设置页面截图发给我。")
        return 3

    print("\n已经连接并启用真实设备。现在轻轻按压或拉动夹爪，观察数字变化。")
    print("如果处理后数值为 0，程序会自动改为显示传感器原始数值。\n")
    try:
        while True:
            state = read_state()
            sensor = state["kwr75d"]
            if not sensor.get("valid"):
                message = sensor.get("message", "没有有效数据")
                print(f"\rKWR75D 当前不可用：{message:<60}", end="", flush=True)
            else:
                fx, fy, fz, tx, ty, tz = map(float, sensor["wrench"])
                raw = list(map(float, sensor.get("raw_wrench") or [0.0] * 6))
                processed_all_zero = all(abs(value) < 1e-9 for value in (fx, fy, fz, tx, ty, tz))
                raw_has_value = any(abs(value) >= 1e-9 for value in raw)
                if processed_all_zero and raw_has_value:
                    fx, fy, fz, tx, ty, tz = raw
                    value_name = "原始"
                else:
                    value_name = "处理后"
                force_change = sqrt(fx * fx + fy * fy + fz * fz)
                print(
                    "\r"
                    f"{value_name}：Fx {fx:8.2f} N   Fy {fy:8.2f} N   Fz {fz:8.2f} N   "
                    f"Tx {tx:7.3f}   Ty {ty:7.3f}   Tz {tz:7.3f}   "
                    f"三方向合计 {force_change:8.2f} N",
                    end="",
                    flush=True,
                )
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n已停止观察。")
        return 0
    except (OSError, ValueError, KeyError, TypeError, URLError) as error:
        print(f"\n读取中断：{error}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
