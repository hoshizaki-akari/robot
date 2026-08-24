# qiaobojiaji

Platform-A 撬拨/夹挤真实机器人控制项目，已整理为可分发目录。

## 包含内容

- `platform_a/`：GUI、视觉、状态机、标定配置和 YOLO 足跟模型
- `scripts/`：撬拨、夹挤、回零、夹爪和状态检查脚本
- `pry_buckle/`：水平直径视觉算法
- `state_service/`：AG95 状态读取模块
- `requirements*.txt`：Python 依赖清单
- `DEPENDENCIES.md`：ROS、D435、FR5 和 AG95 外部依赖说明

## 运行环境

本项目面向 Ubuntu 22.04 / WSL2、ROS 2 Humble、真实 FR5、D435 和 AG95。必须先启动统一状态服务与 D435 ROS driver。

```bash
cd qiaobojiaji
bash scripts/install_dependencies.sh
bash scripts/run_platform_a.sh
```

真实模式需要状态服务：`http://127.0.0.1:8765/api/state`。

## 重要安全说明

这是直接连接真实机械臂的控制软件。首次运行应保持机械臂周围无人员和障碍，并先确认急停可用。项目默认只开放经过现有流程允许的真实操作。

模型、标定文件和零点文件属于当前设备配置，换设备时必须重新确认，不能直接套用。
