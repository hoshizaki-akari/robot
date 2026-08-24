# FR5 足跟复位机器人网页发布版

本目录是基于当前可用网页 `http://127.0.0.1:8080/` 整理出的独立运行版本。原开发目录不参与运行，所有项目内路径均相对于本目录解析。

## 当前页面包含的功能

- 登录、病例建立/关闭、参数保存、日志导出
- 夹挤视觉启停、夹持点与宽度获取
- 撬拨视觉启停、预览、夹持点/足跟表面间距获取
- FR5 状态监测、D435 标注视频、AG95 与 KWR75D 状态显示
- 机器人直接控制入口：暂停、继续、回零、设置零点、急停、急停复位、夹爪开度调整
- 夹挤/撬拨工作流提交接口

真实机械臂与夹爪动作仍然是高风险操作。发布版保留现有业务逻辑和确认参数，但本次整理审核没有执行真实运动。

## 运行前提

- Ubuntu 22.04 / WSL2
- ROS 2 Humble，能加载 `rclpy`、`sensor_msgs`、`cv_bridge`、`realsense2_camera`
- Intel RealSense D435 已通过 USB 接入 WSL
- Fairino FR5 SDK 可在当前 Python 环境导入，控制器地址默认为 `192.168.58.2`
- AG95 串口设备可被发现
- Python 3.10 虚拟环境位于本目录 `.venv/`

安装 Python 依赖：

```bash
cd /home/zhj/projects/fr5_platform_ws_release
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/pip install -r requirements-vision.txt
```

Fairino SDK 与 ROS 2 属于现场运行环境依赖，不复制旧项目的虚拟环境或旧环境路径。若 RealSense 驱动不在 `/opt/ros/humble`，可在启动前设置：

```bash
export REALSENSE_ROS_SETUP=/你的/realsense/ros2/install/setup.bash
```

## 启动与停止

启动发布版：

```bash
cd /home/zhj/projects/fr5_platform_ws_release
bash scripts/platforms.sh start
```

重启、查看状态、停止：

```bash
bash scripts/platforms.sh restart
bash scripts/platforms.sh status
bash scripts/platforms.sh stop
```

网页地址：<http://127.0.0.1:8080/>

启动脚本只启动当前网页所需的 D435、实时状态服务、D435 watchdog 和网页网关，不启动旧版 Tkinter/Platform-A GUI。

## 目录说明

```text
platform_b/control.html              当前网页
platform_b/gateway.py                网关与网页 API
platform_b/robot_control.py          FR5/AG95 直控 API
platform_b/pry_vision_service.py     撬拨视觉隔离服务
platform_b/vendor/jszip.min.js       本地日志压缩依赖
state_service/                       实时状态与 D435 标注流
platform_a/                          当前页面需要的标定、规划、视觉 worker、模型
pry_buckle/                          足跟水平径算法
scripts/                             启动、watchdog、当前页面工作流脚本
```

