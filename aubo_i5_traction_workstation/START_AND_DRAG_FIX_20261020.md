# 2026-10-20：直接启动与省力拖拽断流修复

## 启动

在 WSL 中执行：

```bash
cd /home/zhj/projects/aubo_i5_traction_ws/aubo_i5_traction_workstation
bash start_workstation.sh
```

脚本负责停止本项目旧实例、启动 COM4 桥／ROS／网页，等真实数据使状态变成
READY 后输出网页地址 http://127.0.0.1:8082/ 。保留终端；Ctrl+C 退出。
启动不会开始牵引。网页仍按“省力拖拽 → 松弛时初始校准 → 开始拖拽”操作。

兼容入口 `bash run_workstation.sh --restart-live` 也使用同一套配置。
重启前先检查配置，配置缺失时不会先关掉正在运行的工作站。

校准引用保存在 `config/live_calibration.local.path`；本机已验证的校准内容复制到
`config/commissioning.local.yaml`，不再依赖 debug 目录保存长期运行配置。
这两份本机配置不进入 Git。Windows 的 SDK 凭据仍从当前用户的
`%LOCALAPPDATA%\AuboTraction\connection.clixml` 读取。

新电脑必须提供本机凭据和对应校准。`configure_live_connection.ps1` 会同步保存
项目本地校准引用；本次没有把本机校准或凭据写入通用代码。

## 本次日志的根因

`session_1792478098127_full_log.zip` 在 62 次跟随接受后报 WRENCH_TIMEOUT。
管理器记录 1.209 s 无新力数据，但机器人位姿仍只有 0.054 s 年龄。
只读检查显示原始传感器仍约 200 Hz，当前姿态相对旧 tare 为
0.035903 rad（2.06°），超过旧固定 tare 门槛 0.03 rad（1.72°）。
旧力转换节点因此静默停止输出，管理器最终把它判成数据超时。
这不是本次日志中的 SDK 通信失败或超力停止。

完整工作站现在明确使用 `session_managed_tare=true`：转换节点只负责有效数据的
传感器坐标到基坐标变换，管理器按现有初始校准流程采集当前基坐标负载零点，
控制器接收 `/traction/corrected_wrench`。旧固定 tare 不再重复扣除或用旧姿态
截断数据。独立固定 tare 用法仍保留，真实传感器／位姿过期仍拒绝输出。

转换节点记录实际断流原因和恢复事件，导出 ZIP 新增 `component_events.log`，
包含会话期间的力转换、传感器、管理器和控制器日志。

## 验证范围

- 离线回归包含记录姿态复现、当前会话基坐标零点、旧固定 tare 保留和真实过期拒绝。
- Windows PowerShell 回归使用测试凭据，覆盖 Windows 配置缺少校准字段时的恢复、
  显式路径优先、缺失配置失败以及原 Windows 路径回退，不连接任何硬件。
- 全栈启动必须等待 READY；只读连续性检查保存到本轮 debug 目录。
- 这次没有主动发起机械臂运动；持续手推拖拽的动态验收由网页实测完成。

只读复核命令（不发运动指令）：

```bash
source /opt/ros/humble/setup.bash
source ros2_overlay/install/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI="file://$PWD/config/cyclonedds_local.xml"
python3 scripts/inspect_wrench_live.py --calibration config/commissioning.local.yaml \
  --seconds 45 --output debug/wrench_inspection.json
```
