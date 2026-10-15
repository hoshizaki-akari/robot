# AUBO i5 牵引工作站（从 `881117c` 独立移植）

本目录是 **仅针对 AUBO i5 / AUBOPE V4** 的牵引工作站副本，位于
`/home/zhj/projects/aubo_i5_traction_ws/aubo_i5_traction_workstation`。
原 FR5 工程 `/home/zhj/projects/fr5_platform_ws` 没有改动。本次范围只包括
原 `fr5_traction_workstation` 的网页、ROS 2 牵引控制、KWR75D 力反馈、
急停/回零、记录与导出；不包括 K-wire、Platform A/B、D435 或 AG95 驱动。
后两者并未被 `881117c` 的工作站启动链路使用。

## 当前状态

这是正在适配的独立工程，**尚不能用它驱动真实牵引**。网页操作流程、力控制
数学和记录代码保留；ROS 消息已去除 Fairino 类型依赖，KWR75D 串口协议已
按本机 COM4 实测帧实现。AUBO V4 只读位姿/诊断适配器和官方 i5 可视化模型
均已通过离线构建。网页当前是只读访客视图，所有牵引动作由页面和服务端
双重拒绝；没有沿用原 FR5 页面中的演示口令。

AUBO V4 的高频连续关节/笛卡尔伺服、急停恢复、回零和软件力控尚需接口
验证及小步真机检查。省力拖拽依赖 FR5 原生接口，AUBO V4 无对应已验证
接口，目前明确拒绝启动。新适配器的快速停止请求不等于实体急停，也不
证明机械臂已经停稳。

新工程中复制来的 FR5 `traction_system.launch.py` 被显式锁住；原 FR5 包
有 `COLCON_IGNORE`，不会参与 AUBO 构建，也被新目录 `.gitignore` 排除
提交。新 `scripts/start_ros_stack.sh` 只启动只读入口，默认不连接任何设备。
`aubo_port_verified: false` 是控制状态机的最后一道软件门，只有各项硬件
检查真实通过后才能手动启用。不要直接把它改成 true 来“试运行”。

## 已确认的设备事实

- 控制器：AUBO i5，AUBOPE V4，控制器地址在先前网络测试中为
  `192.168.1.40`；Windows 有线网卡 `192.168.1.10/24` 能 ping 通它。
- 传感器：KWR75D 接 Windows `COM4`。实测串口在 460800 8N1 下，
  发 `48 AA 0D 0A` 后收到连续的 28 字节有效帧。
- 传感器原始数据单位是 kgf 和 kgf·m；ROS 发布前换算为 N、N·m。
- 传感器到法兰、法兰到基座的方向与力符号没有通过真机检查，
  原 FR5 标定与轴符号不能复用。
- 2026-10-15，AUBOPE V4 SDK 已在 `192.168.1.40:8899` 只读登录并读出
  关节角、末端位姿与诊断；控制器报告 `remote_halt=1`（SDK 注释为
  “远程关机信号”），所以机器人健康
  状态为假，未发送运动指令。
- Windows COM4 经本机 TCP + WSL 伪串口桥接，ROS 已收到真实六维力/力矩；
  `/aubo/kwr75/raw_wrench_valid=true`。两个只读 ROS 节点可同时发布。

## 安装、构建与不接硬件的验证

```bash
cd /home/zhj/projects/aubo_i5_traction_ws/aubo_i5_traction_workstation
# 可选：离线 wheel 目录，例如从 Windows 已下载的依赖
export AUBO_WHEEL_DIR=/mnt/c/Users/zhj/AppData/Local/Temp/aubo_i5_workstation_wheels
bash scripts/tablet_setup_env.sh
```

下面的 AUBO V4 SDK 是外部依赖；项目不包含厂商二进制。这里的临时目录仅
供本机离线编译审计，不适合部署。换电脑时需要另外取得完整匹配的 V4 SDK。

```bash
cd /home/zhj/projects/aubo_i5_traction_ws/aubo_i5_traction_workstation/ros2_overlay
source /opt/ros/humble/setup.bash
export AUBO_V4_SDK_ROOT=/mnt/c/Users/zhj/AppData/Local/Temp/aubo-legacy-v4-audit
colcon list
colcon build --packages-select fr_traction aubo_kwr75_sensor aubo_v4_driver aubo_i5_description
colcon test --packages-select fr_traction aubo_kwr75_sensor aubo_v4_driver aubo_i5_description
colcon test-result --verbose
```

离线结果：4 包编译通过；ROS 测试 145 项、0 失败、26 项跳过；
`bash scripts/run_checks.sh` 的 Python/网页测试 19 项通过；官方 i5 URDF
通过 `check_urdf`。这些只证明离线软件部分，没有证明真实牵引。

只检查启动参数（不接控制器）：

```bash
source /opt/ros/humble/setup.bash
source ros2_overlay/install/setup.bash
ros2 launch fr_traction aubo_i5_readonly.launch.py --show-args
```

在完成 SDK 运行时、串口桥接和现场 CHECKPOINT 之前，不要把
`start_robot:=true` 或 `start_sensor:=true` 作为牵引启动方式。现在它们仅
用于只读数据检查，启动文件不会启动牵引控制器。独立网页可用
`bash run_workstation.sh` 在本机 8082 端口查看，只读且不会自行连接机械臂。

当前 V4 SDK 运行库依赖 `libprotobuf.so.9`。已从 Ubuntu 官方
`libprotobuf9v5_2.6.1-1.3_amd64.deb` 单独提取到本机临时目录，并通过
动态库检查与只读登录；没有安装到系统，也没有把 `.23` 冒充 `.9`。
此运行库与 SDK 仍是外部依赖，换机必须重新准备。

## 本机只读硬件检查（不启动牵引）

WSL 中 ROS 2 进程间通信使用 `rmw_cyclonedds_cpp`；本机已安装对应 Humble
包，默认 Fast DDS 在当前 WSL 多网卡环境中没有通过发布/订阅测试。
先在 Windows PowerShell 保持运行：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "C:\Users\zhj\Desktop\骨伤牵引\aubo_i5_traction_ws_active\aubo_i5_traction_workstation\scripts\kwr75_windows_bridge.ps1"
```

在第一个 WSL 窗口保持运行：

```bash
python3 /home/zhj/projects/aubo_i5_traction_ws/aubo_i5_traction_workstation/scripts/kwr75_wsl_pty_bridge.py
```

在第二个 WSL 窗口执行；这些设置仅允许读状态和传感器，不启动运动：

```bash
cd /home/zhj/projects/aubo_i5_traction_ws/aubo_i5_traction_workstation
export LD_LIBRARY_PATH=/mnt/c/Users/zhj/AppData/Local/Temp/aubo-v4-runtime-audit/extracted/usr/lib/x86_64-linux-gnu:/mnt/c/Users/zhj/AppData/Local/Temp/aubo-legacy-v4-audit/lib/lib64/aubocontroller:/mnt/c/Users/zhj/AppData/Local/Temp/aubo-legacy-v4-audit/lib/lib64/config:/mnt/c/Users/zhj/AppData/Local/Temp/aubo-legacy-v4-audit/lib/lib64/log4cplus
export AUBO_V4_HOST=192.168.1.40 AUBO_V4_PORT=8899
export AUBO_V4_USERNAME=aubo
read -r -s -p 'AUBO V4 password: ' AUBO_V4_PASSWORD; echo
export AUBO_V4_PASSWORD
bash scripts/start_ros_stack.sh start_robot:=true start_sensor:=true serial_path:=/tmp/kwr75d-bridge-tty
```

在第三个 WSL 窗口运行只读核对：

```bash
cd /home/zhj/projects/aubo_i5_traction_ws/aubo_i5_traction_workstation
source /opt/ros/humble/setup.bash
source ros2_overlay/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=42
python3 scripts/verify_readonly_topics.py
```

如控制器账户已更改，应使用实际用户名和口令。口令只在本机终端输入，
切勿提交到 Git。检查完依次停止 ROS、WSL 桥接、
Windows 桥接；Windows 脚本会向传感器发送停止连续输出命令。

## 上线前 CHECKPOINT

1. COM4 已通过本机桥接读入 WSL；还需确认采样间隔、零点、方向，以及
   断线后的健康状态失效时间。
2. 确认 AUBO 返回的末端姿态到底是法兰还是 TCP，完成传感器—法兰安装
   旋转/偏移标定；核对输出 `base_link` 方向。
3. 以示教器受控确认每个关节、工具坐标方向与停止语义。
4. 验证 V4 SDK 高频伺服所需的速度、加速度、丢包和 watchdog 行为，
   再从极小无负载动作逐级验证；绝不以单次直线移动冒充 100 Hz 闭环。
5. 重新确认 AUBO 专用零位、工作空间与允许力阈值后，才开放牵引模式。

此检查点之前，离线构建通过不代表可安全进行真实牵引。

## 资料

- [AUBO 官方 V4 ROS 仓库](https://github.com/AuboRobot/aubo_robot)；
  其 Noetic 分支明确面向 V4 控制器。
- [坤维 KWR75 系列](https://www.kunweitech.com/products/866.html)；
  同系列有多种通信版本，实际协议以本机报文核对为准。
