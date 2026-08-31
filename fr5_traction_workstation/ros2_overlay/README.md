# ROS2 集成层

这里保存本工作站对应的 ROS2 应用源码：`fr_traction`、Cartesian 速度控制器、
FR5 消息接口和 FR5 模型配置。它们属于用户仓库中的新项目目录，不属于旧的
`platform_b`。

真实 FR5 的官方 SDK 二进制和基础硬件包仍由机器上的 FR5 基础工作区提供，
默认位置是 `/home/zhj/projects/fr5_learning/robot_ws_backup/new_fairino_ws`。
这是运行依赖，不是本项目的提交目标；尤其不要把本项目推送到
`lny20000101-coder/robot_ws_backup`。

`reference/` 保存了硬件包中为“示教器手动操作 → Cartesian 控制器接管”所需的
接口源码参考。运行前，基础工作区必须包含同样的硬件接口改动，并重新编译。
启动脚本支持用 `FR5_ROS_WS` 指向实际的基础工作区。

应用层不连接 Fairino SDK。SDK 连接者只有 ROS2 `fairino_hardware`，网页只通过
FastAPI/rclpy 订阅真实状态并调用牵引管理服务。

当前基础工作区的 Humble ros2_control 会让控制器插件继承
`/controller_manager` 节点名，所以牵引配置使用真实发布地址
`/controller_manager/wrench`、`/controller_manager/ee_state`，而不是假设的
控制器私有名称。
