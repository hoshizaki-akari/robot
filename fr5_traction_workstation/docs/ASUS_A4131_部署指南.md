# ASUS A4131 一体机部署指南

本指南适用于目标一体机：Ubuntu 22.04 x86_64、ROS 2 Humble、Firefox、用户名
`user000`。程序只部署当前三模式版本，不依赖 Windows、WSL 或开发机用户名。

## 1. 连接网络

部署和更新时先使用 Wi-Fi 访问互联网。机械臂使用单独的有线网口直连，默认控制器
地址为 `192.168.58.2`。

在 Ubuntu“设置 → 网络 → 有线 → IPv4”中选择“手动”，建议填写：

- 地址：`192.168.58.10`
- 子网掩码：`255.255.255.0`
- 网关：留空
- DNS：留空

保存后插好机械臂网线。Wi-Fi 继续负责互联网，有线网口只负责机械臂通信。

## 2. 下载项目

打开一次终端，执行：

```bash
mkdir -p ~/projects
git clone --depth 1 --filter=blob:none --sparse \
  --branch feat/multi-mode-traction --single-branch \
  https://ghfast.top/https://github.com/hoshizaki-akari/robot.git \
  ~/projects/fr5_platform_ws
git -C ~/projects/fr5_platform_ws sparse-checkout set fr5_traction_workstation
cd ~/projects/fr5_platform_ws/fr5_traction_workstation
```

其中 `--depth 1 --filter=blob:none --sparse` 会只获取最新版和本工作站目录，避免下载
大型仓库中的其他项目。该地址只为公开 GitHub 仓库增加下载转发，不需要登录或填写
令牌。若此前已经克隆过，
不要重复克隆，改为：

```bash
cd ~/projects/fr5_platform_ws
git switch feat/multi-mode-traction
git pull --ff-only origin feat/multi-mode-traction
```

## 3. 首次安装

ROS 2 Humble 已经安装完成后，执行：

```bash
cd ~/projects/fr5_platform_ws/fr5_traction_workstation
bash scripts/tablet_setup_env.sh
```

脚本会完成以下工作：

1. 检查 Ubuntu 22.04 和 x86_64 架构；
2. 安装编译工具及缺少的 ROS 2 组件；
3. 从法奥官方仓库下载机器人描述和 Python SDK；
4. 创建网页服务所需的 Python 环境；
5. 编译当前 ROS 2 工作区；
6. 在桌面生成“骨伤牵引机器人工作站”图标。

安装过程中需要输入当前用户密码。网络速度较慢时，官方依赖下载和首次编译会花费
一些时间；完成后无需每次重复安装。

## 4. 启动和关闭

一体机和机械臂可以同时开机。双击桌面的“骨伤牵引机器人工作站”，系统先启动
网页服务并以Firefox全屏显示操作页；机械臂未就绪时显示“等待机械臂连接”，运动按钮不可用。
后台约每3秒检查一次机器人20003端口，上线后自动启动ROS 2控制服务；控制器冷启动尚未
完成时保留网页并重试初始化，不必退出再打开。机器人真正连接时仍会按既定流程建立初始基线，
在校准完成前保持绳带松弛。

关闭方法：

- 键盘按 `Alt+F4` 关闭工作站专用 Firefox；
- 浏览器关闭后，本次启动的网页服务和 ROS 2 控制服务会自动一并停止；
- 默认不随Ubuntu开机启动；按第8节配置自动登录/自启动后，可开机自动进入界面。

机械臂未开机/网线尚未连接不会让网页退出。缺少依赖、Firefox或网页服务自身启动失败时，
才弹出环境/网页错误。首次成功连接后的运行故障不会自动重启控制栈或重新校准，网页保留
以便查看状态、由操作人员处理；关闭窗口会取消后台等待并清理本次启动的全部服务。

## 5. 日志和牵引记录

- 启动日志：`~/.local/state/fr5-traction/logs/`；`startup_*.log`记录等待/重试，`ros_*_attemptN.log`保留每次连接日志。
- 牵引记录：`~/projects/fr5_platform_ws/fr5_traction_workstation/debug/traction_sessions/`
- 页面中的“牵引记录”仍可导出单次完整日志压缩包。

这些数据只保存在目标机本地，不会随 `git pull` 上传到 GitHub。

## 6. 更新软件

此前已经安装完成且能正常运行的一体机，先结束牵引并退出工作站，再拉取和本地编译：

```bash
cd ~/projects/fr5_platform_ws
git switch feat/multi-mode-traction
git pull --ff-only origin feat/multi-mode-traction
cd fr5_traction_workstation/ros2_overlay
source /opt/ros/humble/setup.bash
colcon build --symlink-install --parallel-workers 2 \
  --packages-select fairino_description fairino_msgs \
    fairino5_v6_moveit2_config fr_traction \
  --cmake-args -DBUILD_TESTING=OFF
cd ..
git log -1 --oneline
```

编译成功后，双击原桌面图标，或在当前目录执行 `bash scripts/tablet_start.sh`。该更新命令使用本机已有依赖，不重复执行apt/pip/rosdep联网更新。C++或ROS接口改动必须编译；仅Python driver改动也需要重启进程才会加载，不能只刷新网页。更新前先检查并保留本机未提交修改，不使用reset/clean覆盖。

首次安装或确实缺少依赖时，才执行完整的 `bash scripts/tablet_setup_env.sh`。完整脚本会重新联网确认系统/Python/ROS依赖，并在SDK版本不符时重建SDK目录；普通更新不必每次重跑它。

## 7. 常见问题

### 双击图标没有打开页面

先检查机械臂是否开机、网线是否插好。然后查看最新日志：

```bash
ls -lt ~/.local/state/fr5-traction/logs/ | head
```

### 页面一直显示“等待机械臂连接”

界面已成功启动，后台正在等待机器人。检查机械臂控制箱是否完成开机、有线网卡是否为
`192.168.58.10/24`、网线是否连通；恢复网络后不需要重新打开界面。若端口已通但仍未就绪，
查看 `startup_*.log` 与最新 `ros_*_attemptN.log` 中的初始化失败原因。

### 桌面图标提示“不受信任”

右键图标选择“允许运行”。安装脚本已经尝试自动设置该属性，但部分 Ubuntu 桌面
环境仍会要求首次人工确认。

### 如何退出全屏

按 `Alt+F4`。不要只切换到其他窗口；关闭工作站专用 Firefox 才会触发控制服务的
正常退出。

## 8. 开机自动进入全屏工作站

在Ubuntu设置→用户中解锁并开启user000的“自动登录”。然后在目标一体机的user000终端执行一次：

```bash
mkdir -p ~/.config/autostart
cat > ~/.config/autostart/fr5-traction.desktop <<EOF
[Desktop Entry]
Type=Application
Name=骨伤牵引机器人工作站
Exec=/bin/bash "$HOME/projects/fr5_platform_ws/fr5_traction_workstation/scripts/tablet_start.sh"
Terminal=false
X-GNOME-Autostart-enabled=true
EOF
```

本版先显示网页再等待机器人，不需要旧自启动项里的sleep 20。已有自启动项也可继续使用，只是页面会延后打开。自动启动只连接/初始化设备，不自动点击“开始牵引”；本节设置应在目标一体机执行，不在开发机WSL中执行。
