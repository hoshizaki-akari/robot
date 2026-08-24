#!/usr/bin/env python3
"""在WSL中运行工作区镜像里的足跟局部平面靠近程序。"""

import os
import runpy

os.environ["FR5_PLATFORM_ROOT"] = "/home/zhj/projects/fr5_platform_ws"
runpy.run_path(
    "/mnt/c/Users/zhj/Desktop/骨伤牵引/wsl_project_stage/fr5_platform_ws/"
    "scripts/platform_a_plane_approach.py",
    run_name="__main__",
)
