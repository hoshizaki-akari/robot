# ROS2 集成层

这里保存本工作站对应的 ROS2 应用源码：`fr_traction`、Cartesian 速度控制器、
FR5 消息接口和 FR5 模型配置。它们属于用户仓库中的新项目目录，不属于旧的
`platform_b`。

当前运行与开发请参阅 [接手总览](../docs/handover/README.md)、
[控制说明](../docs/handover/CONTROL.md) 和 [运行手册](../docs/handover/RUNBOOK.md)。
正式入口为工作站的 `scripts/start_ros_stack.sh 2`，source 本目录 `install/setup.bash`。

正式启动中唯一的 Fairino SDK 连接者是 `fr_traction/scripts/fr5_direct_driver_node.py`，
使用工作站 vendor 下固定版本的官方 Python SDK。FastAPI/rclpy 只订阅 ROS 状态并调用服务。
驱动直接提供 `/controller_manager/wrench`、`/controller_manager/ee_state` 和兼容的
控制器切换服务；这些名称不表示当前运行了完整 ros2_control controller_manager。

`fr_controllers` 和 `reference/` 保留旧 ros2_control/C++ 硬件链路的控制器与源码参考，
当前部署脚本和正式 launch 不加载它们。过去使用的外部 FR5 基础工作区与 SDK 二进制
不是当前直连模式的运行前提；恢复旧链路需要另行补齐硬件包和依赖，不能与直连驱动并行。

版本 2 ROS 启动固定使用本目录；`FR5_ROS_WS` 只影响网页和预检的 source 路径。
工作站代码属于 `hoshizaki-akari/robot` 仓库，不要推送到旧硬件参考仓库。
