# KWR75D 绳带松紧检测 Demo

这是一个与 `platform_b` 完全分开的第一阶段 Demo。它只读取真实 FR5/KWR75D
状态，不发送机械臂运动、力控、夹爪或传感器校零命令。

## 功能

- 自动选择真实数据源：ROS 2 Wrench → 已有状态服务 → 法奥 SDK 只读直连。
- 所有受力结果统一显示在 `base_link` 坐标系。
- 松绳状态下采集 2 秒基线，去除法兰、带子自重和静态零偏。
- 机械臂移动时持续显示实时力，但暂停松/紧结论；停稳后再确认，抑制惯性力误判。
- 基线后监测法兰朝向；相对变化超过 1° 时暂停判定，避免重力方向变化污染结果。
- 显示合力、Fx/Fy/Fz、实测受力方向、估算增力运动方向和 30 秒曲线。
- 10 N 以上醒目报警，但不自动控制机器人。
- 自动保存每次运行的 CSV 和诊断信息。

## 启动

```bash
cd /home/zhj/projects/fr5_platform_ws/force_tension_demo
source ../.venv/bin/activate
python -m pip install -r requirements.txt
bash run_demo.sh
```

浏览器打开：<http://127.0.0.1:8092>

标准 ROS 2 输入是：

```text
/force_torque_sensor_broadcaster/wrench  geometry_msgs/msg/WrenchStamped
/joint_states                           sensor_msgs/msg/JointState
```

Wrench 的 `frame_id` 必须是 `base_link`，其他坐标系只有在 TF 可转换时才会接收。

法奥 SDK 的实时状态接收端口通常只能由一个进程占用。如果页面提示“实时状态帧未就绪”，
请先停止已经失效或不再使用的旧平台数据服务；如果旧服务仍能提供有效真机数据，本 Demo
会直接复用它，不会再建立第二条 SDK 连接。

## 使用顺序

1. 确认绳带松动、机械臂静止、法兰朝向在本次试验中保持不变。
2. 点击“设为松绳基线”，等待 2 秒采集完成。
3. 只通过示教器沿 base_link 的 X/Y/Z 平移机械臂。
4. 移动过程中观察实时曲线；每次停稳后查看“松动/张紧”结论。
5. 第一次用已知轴向小力验证方向。如果“增力方向”相反，点击“反转增力方向”。
6. 超过 10 N 时停止继续拉紧；固定物可能被提起，不能把此报警当作机械限位。

页面中的受力方向在合力小于 1 N 时保持为空，避免把松绳噪声画成牵引方向。

## 自动检查

```bash
bash run_checks.sh
```

运行数据位于 `debug/sessions/session_*/force_samples.csv`，最近一次可从页面下载。

软件完成后，按 [`HARDWARE_CHECKPOINT.md`](HARDWARE_CHECKPOINT.md) 做真实绳带小力验收。
