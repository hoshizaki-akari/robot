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
git clone --branch feat/multi-mode-traction --single-branch \
  https://github.com/hoshizaki-akari/robot.git \
  ~/projects/fr5_platform_ws
cd ~/projects/fr5_platform_ws/fr5_traction_workstation
```

若此前已经克隆过，不要重复克隆，改为：

```bash
cd ~/projects/fr5_platform_ws
git pull --ff-only
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

启动前确认机械臂已开机、有线网口已连接。双击桌面的“骨伤牵引机器人工作站”，
系统会依次检查机械臂连接、启动 ROS 2、启动网页服务，并以 Firefox 全屏打开操作页。

关闭方法：

- 键盘按 `Alt+F4` 关闭工作站专用 Firefox；
- 浏览器关闭后，本次启动的网页服务和 ROS 2 控制服务会自动一并停止；
- 软件不会随 Ubuntu 开机自动启动，下一次仍由操作人员双击图标启动。

如果机械臂未连接、环境缺失或服务启动失败，页面不会盲目打开，而会显示中文错误
提示及对应日志位置。

## 5. 日志和牵引记录

- 启动日志：`~/.local/state/fr5-traction/logs/`
- 牵引记录：`~/projects/fr5_platform_ws/fr5_traction_workstation/debug/traction_sessions/`
- 页面中的“牵引记录”仍可导出单次完整日志压缩包。

这些数据只保存在目标机本地，不会随 `git pull` 上传到 GitHub。

## 6. 更新软件

收到新版本后，先退出工作站，再执行：

```bash
cd ~/projects/fr5_platform_ws
git pull --ff-only
cd fr5_traction_workstation
bash scripts/tablet_setup_env.sh
```

安装脚本可以重复执行：已存在的官方依赖和 Python 环境不会重复克隆，只会重新确认
依赖、编译当前代码并刷新桌面图标。

## 7. 常见问题

### 双击图标没有打开页面

先检查机械臂是否开机、网线是否插好。然后查看最新日志：

```bash
ls -lt ~/.local/state/fr5-traction/logs/ | head
```

### 显示无法连接 `192.168.58.2:20003`

这是机械臂控制器通信未建立，不是网页故障。检查有线网口是否为
`192.168.58.10/24`、网线是否连通以及机械臂控制箱是否已启动。

### 桌面图标提示“不受信任”

右键图标选择“允许运行”。安装脚本已经尝试自动设置该属性，但部分 Ubuntu 桌面
环境仍会要求首次人工确认。

### 如何退出全屏

按 `Alt+F4`。不要只切换到其他窗口；关闭工作站专用 Firefox 才会触发控制服务的
正常退出。
