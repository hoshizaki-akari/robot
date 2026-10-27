# 本电脑现场速查

日期：2026-10-26。适用于本机 WSL2 `Ubuntu-22.04-F`，项目路径 `/home/zhj/projects/robot/fr5_traction_workstation`。软件已完整重新编译并做隔离 ROS/网页联调；机器人未接入，实际硬件链路未验收。

## 出发前的软件状态

- ROS 2 Humble、系统 Python 3.10、项目虚拟环境、固定 Python 依赖、官方 SDK 固定版本、描述/模型资源、编译工具和动态库均已检查。
- 正式四个包已在 `ros2_overlay` 编译成功。现场运行无需 apt、pip、GitHub 或 CDN；这以本机当前文件和环境保持完整为前提。
- 真实 manager/controller/TF/网页桥已在独立 ROS 域用静止测试反馈联通，所有测试进程已清理。该结果与真实机械臂 READY 分开解释。
- WSL 使用 Windows 浏览器：Edge 已存在，Windows→WSL localhost HTTP 已实测 200；无需在 WSL 为双终端方案额外安装 Firefox。

出发前或环境变动后可重新执行，不会连接机器人：

```bash
cd /home/zhj/projects/robot/fr5_traction_workstation
bash scripts/check_local_env.sh
```

输出 `LOCAL_ENV_READY` 仅代表软件环境通过，不能替代现场网络/硬件判断。

## 本机有线网口（已完成离线预设）

已识别真正的物理网卡为 Windows“以太网”/Realtek PCIe GbE Family Controller，GUID `6364643c-8787-4077-83a6-9042b445f162`。Wi-Fi 和 VirtualBox/WSL 虚拟网卡均不用于配置机器人地址。

目标配置为：

```text
本机物理以太网：192.168.58.10
掩码：255.255.255.0（/24）
默认网关：空
手动 DNS：空
机器人控制箱：192.168.58.2
```

当前网线未接。已在 Windows 管理员进程中完成上述持久配置并重载指定网卡；独立读取确认 `EnableDHCP=0`、IP/掩码正确、网关和手动 DNS 为空，活动接口的 DHCP 也为 Disabled。Wi-Fi 默认路由仍为 WLAN→192.168.8.1。脚本返回 `prepared_offline`、`VerifiedPersistent=true`、`VerifiedActive=false`：表示断线状态下预设完成，插线后的链路和机器人通信尚未测试。

已提供 `scripts/configure_windows_robot_network.ps1`，默认只查看；只有 Apply/Restore 才修改明确 GUID 对应的物理网卡。可从 WSL 请求 Windows 提权：

```bash
cd /home/zhj/projects/robot/fr5_traction_workstation
powershell.exe -NoProfile -ExecutionPolicy Bypass \
  -File "$(wslpath -w "$PWD/scripts/configure_windows_robot_network.ps1")" \
  -InterfaceGuid '6364643c-8787-4077-83a6-9042b445f162' -Apply -Elevate
```

这是为本机已核对的专用闲置网口准备的工具，严格限制当前 Realtek GUID；不能用于其他电脑/网卡。配置相同时 Apply 只读返回，不会重新提权、覆盖备份或重载网卡。只有需要修改配置时才请求管理员权限。

备份和最后结果保存在 Windows `%LOCALAPPDATA%\fr5-traction`。本次最终备份为 `network_backup_20261026_164113_021.json`；最初 DHCP 记录 `network_backup_20261026_161718_500.json` 也已保留。脚本可按 RestoreBackup 参数恢复指定快照，旧格式仅支持原 DHCP/无静态路由情形，不能当作完整路由回滚工具。`configured_active` 才表示活动地址/网关/DNS都符合预期；`prepared_offline` 不表示控制箱可达。原始核验输出另保存在本项目 `debug/environment_checks/20261026/windows_network.json`。

## 到现场的最短步骤

1. 本机网口已预设；机器人上电，接线到控制箱 PC 网口，确认 KWR75D 已在控制器配置/激活。
2. Windows 的物理以太网应显示已连接并有 `192.168.58.10/24`。保持 Wi-Fi 默认路由；不要把这个地址配置到 WSL 的 eth0 或 VirtualBox 网卡。
3. 未启动 driver 时，在 WSL 查看路由并探测命令端口：

```bash
ip route get 192.168.58.2
timeout 3 bash -c '</dev/tcp/192.168.58.2/20003' && echo '机器人命令端口可达'
```

WSL NAT 下路由经 `192.168.16.1` 可以是正常现象，不能仅据此判失败。若端口失败，先在 Windows 检查物理网口和到机器人路由，再查 WSL/VPN，避免先改控制算法。正式 driver 使用 20004 状态流；不要在 driver 运行时反复建立第二条 20004 探测连接。

4. 退出其他真机 SDK 使用者；终端一启动：

```bash
cd /home/zhj/projects/robot/fr5_traction_workstation
FR5_TRACTION_DATA_DIR="$PWD/debug/traction_sessions" bash scripts/start_ros_stack.sh 2
```

5. 终端二启动网页：

```bash
cd /home/zhj/projects/robot/fr5_traction_workstation
bash run_workstation.sh
```

6. Windows 浏览器打开 `http://127.0.0.1:8081/`。真实设备进入 READY，关节/力/末端持续更新后，再按对应模式进行初始校准和操作。也可只读查看：

```bash
curl -fsS http://127.0.0.1:8081/api/health
```

期望 `ok/fr5_connected/traction_connected=true`。这证明实时反馈链已更新，不能代替实际运动方向和传感器安装确认。首次现场不要用“回零”测试连接：没有本机持久零位文件时将使用旧现场的绝对位姿，须先确认/设置当前现场零位。

结束时先结束牵引，再在两个终端分别 Ctrl+C。仅关闭 Windows 浏览器不会停止这两个独立终端中的服务。

完整模块/硬件排障见 [HARDWARE](HARDWARE.md) 与 [RUNBOOK](RUNBOOK.md)，本轮实测明细见 [本机记录第 5 节](MACHINE_AND_VALIDATION.md#5-现场准备复查2026-10-26)。
