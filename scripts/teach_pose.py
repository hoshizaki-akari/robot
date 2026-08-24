#!/usr/bin/env python3
"""CLI for M1 visual pose teaching and TCP-local Cartesian jog."""
from __future__ import annotations
import argparse
import sys
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
from platform_a.teach_jog import local_move, move_to_taught_pose, teach_pose  # noqa: E402

parser = argparse.ArgumentParser(description="FR5 视觉观察位示教与 TCP 局部微调")
commands = parser.add_subparsers(dest="command", required=True)
teach = commands.add_parser("teach", help="读取当前静止 FR5 状态并保存位姿（不运动）")
teach.add_argument("name", help="例如 wire_a、wire_b、sole")
move = commands.add_parser("move", help="使用保存的关节角 MoveJ 回到示教位")
move.add_argument("name")
move.add_argument("--velocity-percent", type=float, default=5.0)
move.add_argument("--confirmed-clear", action="store_true", help="确认现场已清空且急停可用")
jog = commands.add_parser("jog", help="按当前 TCP 局部坐标系执行小距离 MoveL")
jog.add_argument("dx", type=float, help="local X displacement (m)")
jog.add_argument("dy", type=float, help="local Y displacement (m)")
jog.add_argument("dz", type=float, help="local Z displacement (m)")
jog.add_argument("--velocity-mm-s", type=float, default=10.0)
jog.add_argument("--confirmed-clear", action="store_true", help="确认现场已清空且急停可用")
args = parser.parse_args()
if args.command == "teach":
    pose = teach_pose(args.name)
    print(f"已保存 {args.name}: {pose['joints_deg']}")
elif args.command == "move":
    print(f"MoveJ 已完成 {args.name}: {move_to_taught_pose(args.name, args.velocity_percent, confirmed_clear=args.confirmed_clear)}")
else:
    print(f"Local Jog 已完成，目标 TCP [mm,deg]: {local_move(args.dx, args.dy, args.dz, args.velocity_mm_s, confirmed_clear=args.confirmed_clear)}")
