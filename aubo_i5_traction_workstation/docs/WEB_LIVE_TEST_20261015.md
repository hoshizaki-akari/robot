# AUBO i5 网页实测记录（2026-10-15）

省力拖拽在早先网页实测中短暂进入 DRAGGING，约 0.48 秒后以
ROS2_CONTROL_ERROR 退出。三种牵引模式均尚未完成真机验收。

本次修复了网页工作模式接口、默认 DDS 实现、空闲流控定时器、
驱动故障留存与重复发布，以及 ROS 诊断等级为 bytes 时导致网页
接收线程退出的问题。新增了 0.25 秒驱动反馈过期判定。

恢复服务后曾成功读取真实机器人和 COM4 力传感器数据。随后驱动报错：

```text
V4 state read failed (waypoint=0, diagnosis=10007)
```

本机厂商 SDK 头文件将 10007 定义为 ErrCode_RequestTimeout。
请求超时的底层原因仍待定位。不能将网页接收线程修复视为连续流控已修复。

新增 `scripts/verify_zero_hold.py`，只有显式传入 `--execute-zero-hold`
才尝试两秒零速度保持，退出时请求停止流。本次运行均在反馈检查阶段
拒绝启动，没有进入运动流。

本次验证：网页/Python 20 项通过；驱动构建通过；驱动测试统计
19 项、0 错误、0 失败、4 跳过。此前同一轮牵引包测试统计
110 项、0 错误、0 失败、18 跳过。

网页恢复于 <http://127.0.0.1:8082/>，默认运行方式：

```bash
cd /home/zhj/projects/aubo_i5_traction_ws/aubo_i5_traction_workstation
bash run_workstation.sh
```

当前运行时 `AUBO_TRACTION_ENABLED=false`。安装方向与位姿语义标定
均未通过，启动参数保持 false；不能把占位单位旋转声明为已验证。
下一真实力控 CHECKPOINT 需要确认传感器 XYZ 标记与法兰/工具的安装关系。

本机完整简报和网页截图保存在 `debug/session_20261015_web_live/`。
该目录由项目忽略规则保留在本机，不随 Git 同步。
