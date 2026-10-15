# AUBO i5 牵引工作站 ROS 2 层

本目录仅为 `881117c` 牵引工作站的隔离移植。构建时只使用四个包：

- `fr_traction`：沿用的牵引数学、消息与状态机，但 AUBO 真机运动门默认关闭；
- `aubo_kwr75_sensor`：KWR75D 传感器原始六轴读数，单位转换为 SI；
- `aubo_v4_driver`：AUBOPE V4 只读状态与快速停止请求；
- `aubo_i5_description`：官方 i5 可视化模型，未做本机运动学标定。

复制来的 FR5 包均已 `COLCON_IGNORE`，不参与构建或提交。当前
`fr_traction/launch/aubo_i5_readonly.launch.py` 默认不连接机器人、传感器，
也不启动牵引控制器。原 `traction_system.launch.py` 明确拒绝启动。

V4 SDK 的库文件保持外部依赖；当前审计包缺少运行所需的
`libprotobuf.so.9`。不能用系统 `libprotobuf.so.23` 软链接代替。
在取得匹配运行库、完成力/坐标标定及受控真机检查之前，只能使用
离线测试和默认只读入口，不能启用牵引。
