# 运行、部署与演示手册

当前行为更新：2026-10-27。依据 `feat/multi-mode-traction` 本次新工作树编写；启动脚本和 `traction_system.launch.py` 是实际运行行为的依据。本文命令中的本机项目位置为 `/home/zhj/projects/robot/fr5_traction_workstation`。迁移机器时更换此路径，项目内部脚本会自行定位目录。

用户已连接真机并允许只读查询。本次新工作流尚未替换原真机运行栈，未执行新流程的实机运动验收；构建、假 SDK 与隔离 ROS 测试范围见 MACHINE_AND_VALIDATION。本文启动命令会接管硬件，不能用于只读确认连通性。

## 1. 先分清当前系统和历史系统

当前推荐入口是 `scripts/start_ros_stack.sh 2`：

```text
FR5 / KWR75D
    ↕ 唯一 SDK 连接：fr5_direct_driver_node.py
    ↕ ROS 2
traction_controller_node + traction_manager_node
    ↕ backend/ros_bridge.py（rclpy）
FastAPI app.py → 本机浏览器
```

同一 launch 还启动 `robot_state_publisher` 供真实关节状态生成 TF。默认不启动 MoveGroup、Gazebo、ros2_control 硬件插件、夹爪或相机；网页没有第二条 SDK 通道。`/controller_manager/...` 是保留的兼容名称，当前不意味着正在运行真正的 `controller_manager`。

| 目录或入口 | 当前职责 | 使用边界 |
|---|---|---|
| `ros2_overlay/src/fr_traction/` | 管理器、控制算法、ROS 消息/服务、Python 真机驱动 | 当前开发和运行主路径 |
| `ros2_overlay/src/fairino_msgs/` | `PoseTwist` 和历史法奥通用消息/服务定义 | 当前驱动/控制器仍引用，必须一起有可用的生成接口 |
| `ros2_overlay/src/fairino5_v6_moveit2_config/` | 当前 launch 借用其中的 FR5 xacro、初始位置及模型资源；另留历史 MoveIt 配置 | 包名不能代表当前采用 MoveIt 控制 |
| `ros2_overlay/src/fr_controllers/` | ros2_control 笛卡尔速度插件：KDL WDLS 逆解、关节速度/加速度/位置限制、陈旧指令保护、FK 末端状态 | 当前安装脚本不构建，当前 launch 不加载；修改它不会改变默认三模式运动 |
| `ros2_overlay/reference/` | 原 `fairino_hardware` 的接口/指令服务参考源码 | 不在 colcon 的 `src` 中，没有本项目构建定义，不会自动部署到任何基础工作区 |
| `scripts/start_ros_stack.sh 1` | 第一版归档恒力基线入口 | 默认寻找项目父目录的 `runtimes/force_stable_v1`；不是自动切换 Git 提交，本机未发现这个默认归档目录 |
| 包内 `constant_force*`、`fuzzy_constant_force`、`admittance_control`、`real_robot` launch | 历史 ROS/MoveIt 控制方案 | 引用仓库当前工作区没有的 `fairino_admittance`、`fr_force` 等；不能替代当前入口 |

历史硬件接口用 C++ SDK 连接，读取关节、速度和力，再把关节位置命令交给 `ServoJ`。它保留了示教器手动操作到位置控制器的接管同步、`ServoMoveStart/End`、力非有限值检查等改动。`command_server` 是广泛暴露法奥运动/IO/脚本命令和旧非实时反馈的另一条接口，也会自行连接机器人。以后确实恢复这一架构时，必须在独立基础工作区补齐 C++ SDK 和硬件包构建，并重新验证；不能与当前 direct driver 并行连接。

## 2. 环境与依赖的实际布局

目标一体机方案：原生 Ubuntu 22.04、x86_64、ROS 2 Humble、系统 Python 3.10、Firefox。当前开发机静态核对为 Ubuntu 22.04.5 / WSL2 / x86_64，存在 `/opt/ros/humble/setup.bash`。WSL 可用于开发与现有启动流程，网络和本机依赖现状以 FIELD_READINESS 与 MACHINE_AND_VALIDATION 的有日期记录为准；原生一体机桌面方案需在其机器独立验证。

```text
/home/zhj/projects/robot/                       ← Git 仓库根目录
  .venv/                                      ← 默认网页虚拟环境（项目父目录）
  fr5_traction_workstation/                    ← 本项目
    vendor/fairino-python-sdk/linux/fairino/Robot.py
    ros2_overlay/
      src/fairino_description/                ← 官方第三方 URDF / STL
      src/fairino_msgs/
      src/fairino5_v6_moveit2_config/
      src/fr_traction/
      src/fr_controllers/                     ← 历史插件，默认未构建
      build/ install/ log/                    ← 当前运行脚本使用的工作区产物
    build/ install/ log/                      ← 本机还存在另一套旧构建产物
```

**不能混用两套 build/install。** 接管时静态读取到：`ros2_overlay/build/fr_traction/CMakeCache.txt` 的 `BUILD_TESTING=OFF`，项目根 `build/fr_traction/CMakeCache.txt` 为 `ON`；两者都指向同一份 `ros2_overlay/src/fr_traction`。默认运行 source 的是 `ros2_overlay/install/setup.bash`。在根目录旧 build 里测试通过，不能证明默认运行二进制已更新；不要随手 source 根目录 `install`。

环境加载顺序是：

```bash
cd /home/zhj/projects/robot/fr5_traction_workstation
source /opt/ros/humble/setup.bash
source ros2_overlay/install/setup.bash
source ../.venv/bin/activate
```

三个启动脚本内部会自己完成相应 source，正常使用不必重复。ROS 的 setup 在严格 `set -u` 下可能读取尚未定义的变量，现有脚本已按“source 后再启用 nounset”处理；写新脚本也遵循这个顺序。

网页 `requirements.txt` 固定了 FastAPI 0.116.1、uvicorn 0.35.0、websockets 15.0.1、pydantic 2.11.7、ruff 0.16.5。rclpy、ROS 消息和 C++ 依赖由系统 ROS/工作区提供，不能只 `pip install -r requirements.txt` 就期待完整运行。虚拟环境使用 `--system-site-packages`，以便访问系统 ROS Python 包。

Python SDK 固定提交为 `62add0e6b7c7d2e157beaaa4485042c9d54e1f84`，接管时本机 vendor HEAD 与此一致。驱动通过 `_load_robot_sdk()` 加载 SDK，并为其未使用的 Cython 选项导入提供兼容 stub，正式路径不要求额外安装 Cython；不要绕过这个 loader 新建 SDK 诊断连接。官方机器人描述当前下载最新浅克隆，脚本没有固定其提交；完整复现部署时应另外保存描述文件来源版本/校验值。

## 3. 新机器首次部署

前提：已经安装 Ubuntu 22.04 x86_64 和 ROS 2 Humble，部署时可访问互联网。已有完整仓库不要重复克隆。若从零部署，可采用 `docs/ASUS_A4131_部署指南.md` 的稀疏克隆方法，只取分支 `feat/multi-mode-traction` 的 `fr5_traction_workstation` 目录。

```bash
cd /home/zhj/projects/robot/fr5_traction_workstation
bash scripts/tablet_setup_env.sh
```

该命令需要 sudo，会 apt 安装工具、获取官方依赖、创建父目录 `.venv`、pip 安装依赖、运行 rosdep、编译 ROS 工作区并生成桌面图标；它不是只读检查。默认 GitHub 镜像前缀为 `https://gh-proxy.com/`。可覆盖前缀；**设为空字符串会选 `git@github.com:...` SSH 地址，不是 HTTPS 直连**，因此需要事先配置 SSH 登录。例如已有可用 SSH 时：

```bash
GITHUB_MIRROR_PREFIX= bash scripts/tablet_setup_env.sh
```

部署脚本选定构建 `fairino_description fairino_msgs fairino5_v6_moveit2_config fr_traction`，`BUILD_TESTING=OFF`，不构建 `fr_controllers`。ROSdep 只扫描 `fairino_msgs` 和 `fr_traction`，没有安装整个历史 MoveIt/Gazebo 运行环境。Firefox 必须预先可用；脚本会装 zenity 等启动辅助工具，但不负责安装 Firefox 或 ROS 发行版。

脚本在 SDK 路径缺失或提交不符时会删除对应 vendor 目录后重建，描述包缺失时也会重建其目录；不要在第三方下载目录保存自己未提交的修改。已有 `fairino_description/package.xml` 时不会自动更新描述包。

## 4. 硬件网络前提

机器人默认 IP 为 `192.168.58.2`。原生一体机建议单独有线网卡 `192.168.58.10/24`，网关和 DNS 留空，Wi-Fi 负责互联网，避免机器人网口成为默认路由。

实际布线和 KWR75D 的控制器配置见同目录硬件交接文档。电脑通过 FR5 SDK 获取已接入机器人控制器的力传感器数据，当前工作站不直接打开 KWR75D 的串口或独立网络地址。

之后现场可做这些网络探测；端口探测会建立短暂连接，应在没有运行 driver 时使用，尤其不要反复连接单一所有者正在使用的 20004 状态流。已有 driver 时优先查看 socket 和日志：

```bash
ip -br address
ip route get 192.168.58.2
ping -c 3 192.168.58.2
timeout 2 bash -c '</dev/tcp/192.168.58.2/20003'
timeout 2 bash -c '</dev/tcp/192.168.58.2/20004'
```

20003 是命令 XML-RPC，20004 是 SDK 实时状态流。TCP 能连通只证明网络端口可达；不能证明力传感器已激活、数据帧兼容、机器人使能或方向正确。`tablet_start.sh` 只探测 20003，没有同时探测 20004。WSL 故障先区分 Windows 宿主网卡和 WSL 内路由；不要仅给 WSL 虚拟网卡随意配置上述物理网口地址。

## 5. 开发时运行与关闭

以下是真机运行命令，须在现场静止、绳带松弛且允许接管时使用；整个 manager 启动会自动建立一次基线。新构建尚未用于本次真机运动验收。先确保没有旧 `state_service` 真机读取、`fr_force`、`fr_robot_driver`、旧 `ConstantForce`、另一份 direct driver 或独立 SDK 调试程序连接 FR5。

终端一：

```bash
cd /home/zhj/projects/robot/fr5_traction_workstation
FR5_TRACTION_DATA_DIR="$PWD/debug/traction_sessions" bash scripts/start_ros_stack.sh 2
```

终端二：

```bash
cd /home/zhj/projects/robot/fr5_traction_workstation
bash scripts/preflight_check.sh
bash run_workstation.sh
```

浏览器打开 `http://127.0.0.1:8081/`。确认驱动有 `FR5 direct driver connected` 日志，manager 进入 `READY` 且状态 `calibrated=true`，实时位姿/关节/力更新且没有故障，再进入操作流程；不必手动点击初始校准。驱动连上日志只是中间条件；桌面监督器目前用该行判断驱动就绪，不会等到 manager 的 READY。

终端模式关闭时分别在网页和 ROS launch 所在终端按 Ctrl+C。直接关闭浏览器不会停止终端里独立启动的服务。牵引中的正常停止应先在页面按“结束牵引”完成模式对应流程；实体急停使用现场急停装置。不要将关闭 Python/ROS 进程视为日常卸力方式。

重跑 `start_ros_stack.sh` 会查找当前用户所有匹配的本项目 launch/direct driver/manager/controller 进程，先 INT、再 TERM；匹配不按路径或 ROS domain 隔离。重跑 `run_workstation.sh` 会停止占用所选 TCP 端口的进程，也不核实它一定属于本项目。两者都不是无副作用的检查命令；换端口可避免接管其他服务。

| 环境变量 | 默认 / 实际用途 |
|---|---|
| `FR5_ROBOT_IP` | `192.168.58.2`，当前 ROS 入口和桌面连通性检查使用 |
| `FR5_SDK_PYTHON_PATH` | 项目内 `vendor/fairino-python-sdk/linux`；`tablet_start.sh` 会固定为此路径 |
| `FR5_TRACTION_DATA_DIR` | 开发入口默认相对路径 `debug/traction_sessions`；桌面入口固定为项目内绝对路径。开发时也建议如上显式设绝对路径 |
| `WORKSTATION_PORT` | 8081；网页只监听 `127.0.0.1`；改成 8082 时浏览器也使用 8082 |
| `FR5_VENV_DIR` | 仅 `run_workstation.sh` 可覆盖，默认父目录 `.venv`；安装脚本和桌面完整性检查仍认固定位置 |
| `FR5_ROS_WS` | 仅网页/预检脚本用于选择 install 路径；**版本 2 ROS 启动固定使用本项目 overlay，不接受此变量切工作区** |
| `FR5_BASELINE_WS` | 仅版本 1 的归档工作区；使用旧网页时还需将 `FR5_ROS_WS` 显式指向相同归档 |
| `FR5_USE_RVIZ` | false；当前包缺少 launch 引用的 `launch/moveit.rviz`，启用前需补资源并安装 RViz。网页 3D 不依赖该开关 |
| `FR5_ZERO_SENSOR_ON_ACTIVATE` | true，但当前 launch 仅传给 xacro；direct driver 不声明此参数也不执行 FT_SetZero，不能用它控制当前硬件校零。manager 启动自动采集一次软件松绳基线；顶栏可手动重新校准 |
| `ROS_DOMAIN_ID` / `RMW_IMPLEMENTATION` | 脚本没有覆盖；ROS 栈、网页、诊断终端必须保持一致。发现不了节点时先比较这些变量 |

直接执行 `ros2 launch fr_traction traction_system.launch.py` 还提供 `use_sim_time`、`use_web_bridge` 等参数。`use_sim_time=true` 不会改成仿真驱动，仍会创建 SDK 连接；正式硬件运行保持 false。`use_web_bridge=true` 启动的是额外 rosbridge_server，当前网页使用 rclpy 桥，不需要它；安装脚本也没有为这个可选入口确保依赖。

## 6. 桌面一键运行

原生一体机执行过安装脚本后，双击“骨伤牵引机器人工作站”，或手动执行：

```bash
cd /home/zhj/projects/robot/fr5_traction_workstation
bash scripts/tablet_start.sh
```

该监督器检查环境、Firefox 与 `ROBOT_IP:20003`，分别用 setsid 启动 ROS 和网页，等待日志/HTTP 响应，再打开独立 Firefox 配置的 kiosk 窗口。关闭该专用窗口（Alt+F4）会触发 INT/TERM 清理两个后台进程组；普通 Firefox 窗口不由它管理。再次双击会替换记录在 PID 文件中的旧监督器。它不安装开机自启。

| 位置 | 内容 |
|---|---|
| `~/.local/state/fr5-traction/logs/ros_*.log` | 桌面 ROS 启动和运行日志 |
| `~/.local/state/fr5-traction/logs/web_*.log` | 桌面网页日志 |
| `${XDG_RUNTIME_DIR:-/tmp}/fr5-traction-$UID/supervisor.pid` | 桌面监督器 PID |
| `~/.local/share/fr5-traction/firefox-profile/` | 独立 Firefox 配置；snap 版改用 `~/snap/firefox/common/fr5-traction-profile/` |
| `~/.local/state/fr5-traction/settings.json` | 网页持久化参数；可由 `FR5_WORKSTATION_SETTINGS_PATH` 覆盖 |
| `~/.local/state/fr5-traction/zero_pose.json` | 历史遗留文件，新流程没有公开回零/设零/返回运动，不用为部署创建或迁移该文件 |
| `debug/traction_sessions/` | 默认 session 文件和网页 `operations.csv` |

网页 `app.py` 的操作记录/单次导出路径固定在项目 `debug/traction_sessions`，而 manager 的 `data_directory` 可独立配置。两者若不同，可能历史摘要存在但单次文件导出缺失；保持统一，迁移时一起备份。它们、venv、vendor、build/install 都不随普通 Git 提交保存。

## 7. 演示边界与操作顺序

页面启动直接进入本地操作模式，无登录或退出登录窗口；设置与操作入口按设备状态开放，操作记录标记“本机操作员/本地操作”，不再要求演示账号。

真机启动前保持绳带松弛、机械臂静止；启动后健康反馈连续至少 0.5 s、采到至少 20 个新力样本，自动完成本次程序的软件基线。默认进入省力拖拽。页面刷新、普通结束、切模式、故障/急停恢复或同进程反馈重连不重校准；重启 manager 是新一次程序启动，会重新采集基线。

1. **省力拖拽（2，默认）**：自动校准完成后点击开始；由法奥原生辅助拖拽执行。结束等待原生 OFF 确认后停止并保位，不要求再次校准。
2. **目标牵引（1）**：待机时医生施力至少 3 N，方向每 2 s 稳健更新；可直接点击开始，使用已有有效方向并锁定。全程直到点击结束都不改变方向。
3. **没有方向时直接开始**：按用户确认沿真实工具Z−规划5秒内30厘米的平滑搜索，理论峰速112.5mm/s。达到3N立即停止，静止确认持续受力方向后进入正常牵引；若瞬时尖峰在停后消失，可在原5秒/30厘米预算内从静止继续，不重新计时。到限仍无稳定方向则停止提示，保留校准，不自动开始新搜索。该操作会实际移动机器人。
4. **目标到位**：控制向设定值中心逼近；实际力在目标±2.5N内连续2s后停止保位，不要求controller已SETTLING。出带或反馈过期重新计时。例如10N目标在7.5–12.5N范围连续2秒即可停止，确认期间仍朝10N控制。之后外力变化不会自动追赶；修改目标可重新牵引，结束则保位并解锁方向。
5. **卸力（3）及一键卸力**：有历史牵引方向时优先使用，否则使用最新有效方向。允许目标牵引/保持/拖拽期间一键卸力，原生拖拽须先确认退出再交接。沿减力方向最多 20 mm/s、临近 5 N 减速，真实合力 <5 N 即刻停止并保位。无方向时不自动猜方向。

恒力模式已经取消。可调目标上限默认 100 N，可在设置中提高至 100 N 以上；后端与 manager ROS 参数同步。实际受力超过 100 N / 150 N 不再触发工作站软件超力故障，有限值、通信、急停和控制柜自身保护仍保留。

顶栏小按钮“重新校准”仅在停止、健康且无交接时使用；它是显式替换力基线，不是继续任务的前置步骤。当前页面及 HTTP/bridge/ROS 公开层均移除回零、设置零点、返回牵引前位姿和手动方向确定。旧文档的回零/返回流程不适用于本版本。

没有硬件时能阅读代码、运行离线测试，或仅打开真实网页壳查看布局和离线提示。当前项目没有完整的三模式仿真/假力演示后端；网页不会凭空生成成功运行和牵引历史。只查看页面时，可在独立 Python 进程中显式禁用 ROS bridge 的启动：

```bash
cd /home/zhj/projects/robot/fr5_traction_workstation
../.venv/bin/python3 - <<'PY'
import app
import uvicorn
app.bridge.start = lambda: None  # 仅本次预览进程禁用 ROS；不修改源码
uvicorn.run(app.app, host="127.0.0.1", port=8082)
PY
```

浏览器打开 `http://127.0.0.1:8082/`，预览结束 Ctrl+C。此方式不创建 ROS node/SDK，页面应保持未连接，不能演示真实受力和动作；设置/关闭等本机 API 仍存在，不要用预览更改正式参数。本次已用同样方式在临时端口验证资源与 WS；没有进行浏览器图形验收。`tablet_start.sh` 无硬件会在 20003 检查失败；把 `demo.launch.py` 或 Gazebo launch 当无硬件替代也不成立，现有 xacro 默认仍是 `fairino_hardware`，Gazebo include 被注释且依赖不完整。

## 8. 检查脚本覆盖与盲区

`scripts/preflight_check.sh` 只 source Humble 和选定 install，再列节点、检查一组话题与服务名称是否存在。缺少项打印 `[缺少]`，通常仍以 0 退出。它没有检查 TCP 端口、实际发布频率、消息时间戳、数据值/坐标系、单一发布者、节点健康布尔值、READY、SDK 连接独占或已安装源码是否最新；甚至没有列入 `/controller_manager/ee_state` 和硬件急停/恢复服务。不能把退出码 0 当完整就绪。

包内 `check_traction_runtime.py` 是旧检查器，仍查询 `/force_torque_sensor_broadcaster/wrench`，当前正确原始力话题为 `/controller_manager/wrench`；它未接入当前 launch，禁止节点特征也不完整。不要因它对当前栈失败就随手再启动旧广播器。

真机已启动后，在同一 ROS 环境可分别执行这些只读诊断（连续频率命令用 Ctrl+C 结束）：

```bash
ros2 node list
ros2 topic info -v /controller_manager/wrench
ros2 topic echo --once /controller_manager/healthy
ros2 topic echo --once /traction_controller/healthy
ros2 topic echo --once /traction/status
ros2 topic echo --once /controller_manager/wrench
ros2 topic echo --once /controller_manager/ee_state
ros2 topic hz /controller_manager/wrench
ros2 topic hz /controller_manager/ee_state
ros2 topic hz /traction/status
ros2 service list
```

期望 driver 力/位姿约 100 Hz，manager 状态约 20 Hz，力 frame_id 为 `base_link`，原始力话题一个有效发布者。CLI 的订阅频率有调度/QoS 开销，必须结合日志与连续帧时间戳判断，不把一次 hz 数字作为硬件认证。发现 `/controller_manager` 前缀不应直接运行 `ros2 control load_controller`；当前兼容服务不是完整 controller_manager。

## 9. 离线验证与编译

这些测试不初始化实际机器人连接，但需要相应 ROS/编译依赖。不要运行 SDK 的 `RPC()`、驱动 main 或真机 launch 来“验证导入”。

网页/后端快速回归：

```bash
cd /home/zhj/projects/robot/fr5_traction_workstation
bash scripts/run_checks.sh
```

它执行 `compileall app.py backend tests`、`ruff check app.py backend tests`、根目录 Python unittest。它不 source overlay、不编译 ROS、不运行 C++ 或 `fr_traction/test/test_native_drag_driver.py`，也不检查浏览器渲染/JavaScript 语法；这些测试大量采用模拟 bridge/service 或源码契约，不能证明实物行为正确。该脚本固定查父目录 `.venv/bin`，不使用 `FR5_VENV_DIR`。

修改 C++、消息或 ROS 参数/launch 后，在唯一约定的 overlay 内编译。如下命令利用已部署 description/msg/config；首次机器先执行部署脚本：

```bash
cd /home/zhj/projects/robot/fr5_traction_workstation
source /opt/ros/humble/setup.bash
source ros2_overlay/install/setup.bash
cd ros2_overlay
colcon build --symlink-install --packages-select fairino_msgs fr_traction --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select fr_traction --event-handlers console_direct+
colcon test-result --verbose
```

启用测试需要 `ament_cmake_gtest`、`ament_cmake_pytest`、`ament_lint_auto` 等测试依赖。CMake 当前注册六套 C++ 测试（数学、状态机、控制器、安全、合成管线、方向纠偏）、一套 native drag Python pytest 以及 ament lint。native drag 使用 Fake SDK 验证新旧接口、参考系、模式/使能切换、停机失败等，没有连接实体。

也可在加载正确 overlay 的终端只跑该驱动离线测试：

```bash
cd /home/zhj/projects/robot/fr5_traction_workstation
source /opt/ros/humble/setup.bash
source ros2_overlay/install/setup.bash
../.venv/bin/python3 -m pytest -q ros2_overlay/src/fr_traction/test/test_native_drag_driver.py
```

运行 `colcon test-result` 时检查失败项和本次时间，不能用旧 XML 的汇总代替本次测试。`--symlink-install` 不意味着 C++ 二进制自动重新编译；消息/服务更改后还要重启网页进程和 ROS 栈使两边加载同一版本。只改网页 JS/HTML 时刷新浏览器并确认缓存版本；改 app/bridge 后重启网页。

本次同时修改 ROS 消息、manager/controller、driver、网页和 bridge，必须完整重建生成接口，并在机械臂静止、现场允许重新接管时重启 ROS 栈与网页。`--symlink-install` 不能使运行中的 Python/C++ 自动重载，也不能使旧 Python 消息类型自动变成新类型。只重启网页或只刷新浏览器不足以加载本次新工作流。

目前正式真机栈仍是旧版本，新实现先在独立构建/隔离 ROS 中验证；最终验证数据见 MACHINE_AND_VALIDATION。待准备切换时，按上面的正式运行命令启动本项目 `ros2_overlay/install`，并核对 `ros2 pkg prefix fr_traction`、进程路径与状态新增字段。不能 source 项目根旧 `install`，也不能把临时测试工作区路径遗留为现场运行环境。

原生拖拽此前的启用等待和丢回复清理修复仍保留。若报启用未确认，stop_reason 含清理前 dragState、控制器错误码等诊断，按 HARDWARE 4.3 核查，不无限延长等待或跳过确认。

## 10. 更新、备份和定位入口

更新前退出运行栈，保留现场 session、operations、settings，并检查仓库根目录 Git 状态。旧 zero_pose 文件不参与新工作流，可作为历史资料归档。接管前已存在 `tests/test_emergency_recover.py`、`tests/test_set_zero.py` 两项本地修改；不要用 reset/clean 覆盖。正常同步使用 `git pull --ff-only`，之后按修改范围重新编译/重启；只有需要补部署依赖时才重跑完整 setup。

| 现象 / 后续修改 | 首先查看 |
|---|---|
| SDK 未连接、实时帧丢失、20003 可达但力不更新 | 本机路由/20004、桌面 ROS 日志、direct driver 和 SDK 固定版本；见硬件交接 |
| 页面能开但一直 INITIALIZING | bridge 是否成功导入 ROS 消息、source 路径/domain 是否一致、`/traction/status` 新鲜度 |
| manager 不 READY / 报超时 | 原始力、EE、健康话题，参数中的 timeout；不要先改网页状态文字 |
| 参数改了无效 | 改的是 YAML、launch 覆盖还是 class 默认；运行的 `ros2 pkg prefix fr_traction` 指向哪个 install |
| 改算法没有效果 | `fr_traction_core` 及 controller/manager 是否重新编译；是否误改历史 `fr_controllers` |
| 导出缺文件 | manager data_directory 与 app 固定 SESSION_ROOT 是否一致，文件是否留在本地而未迁移 |
| 双击启动报错 | `~/.local/state/fr5-traction/logs/`、环境完整性、Firefox、20003；等待 READY 与仅驱动连上需分开 |
| 调力控/方向/状态机 | 同目录算法/架构交接，`traction_params.yaml` 和相应 core 源码、对应 C++ 测试 |
| 改按钮/曲线/模型/接口 | 同目录界面交接，static → app → ros_bridge → ROS 服务/话题链 |

## 11. 历史文档与当前代码差异

| 文档 | 可保留参考的部分 | 需要用当前源码纠正的部分 |
|---|---|---|
| 根 `README.md` | 单 SDK 所有者、桌面部署大方向 | 2026-10-27 当前流程以本交接和源码为准：自动校准、目标/拖拽/卸力、无公开回零；示例路径仍需按机器调整 |
| `ros2_overlay/README.md` | reference 的来源和旧基础工作区背景 | 接手时已更新为 Python direct driver；旧 fairino_hardware 是历史参考，当前 direct 模式不依赖旧外部基础工作区 |
| `docs/ASUS_A4131_部署指南.md` | 原生 Ubuntu 网卡、稀疏克隆、双击关闭和日志 | 项目目录需按本机改；“依赖不会重复克隆”对 SDK 提交改变情形不成立；SDK pin、镜像等以脚本为准 |
| `docs/FR5骨伤牵引机器人工作站项目说明书.md` | 系统分层、数据链路、方向算法背景、记录格式 | 只覆盖恒力旧版本；1～20 N、校准保存零位、150 mm 行程、旧导纳参数、手动方向确认与恒力模式均不能作为新流程说明；其 preflight 描述比实际脚本能力更广；心跳消息类型应以源码为 `std_msgs/Empty` |
| `docs/software-copyright-docs/*操作说明书*` | 历史恒力版 UI 说明和软件材料 | 不能当三模式版操作/保护/部署的唯一依据，重复文字不是实现证明 |
| `docs/integration_stage_report.md`、`docs/stage_report.txt` | 2026-09-01 某次现场故障、5 N 验证和记录 ID 证据 | 25 Hz、20 N、30 mm 搜索、80 mm 行程、旧零位/超时均已演进；其中 `current_state` 是报告当时快照，不代表现在；历史 5 N 通过不能证明新流程的 30 cm 搜索、卸力、100 N 以上目标或原生拖拽已验收 |

阅读历史报告用于理解改动原因，运行时只依据当前分支、实际加载 install、现时参数、机器人反馈与当场验收。

## 现有一体机更新（2026-10-27）

按ASUS部署指南的一体机路径操作。先结束牵引，再以Alt+F4退出桌面工作站（终端启动则分别Ctrl+C关闭网页和ROS）；更新期间不运行SDK/控制进程。

```bash
cd ~/projects/fr5_platform_ws
git switch feat/multi-mode-traction
git pull --ff-only origin feat/multi-mode-traction
cd fr5_traction_workstation
bash scripts/tablet_setup_env.sh
git log -1 --oneline
```

安装成功后执行 `bash scripts/tablet_start.sh` 或双击原桌面图标。setup会编译本机ROS可执行程序并更新入口；不是下载Windows exe，也不会自动启动机械臂。若一体机使用其他克隆位置，将首行换为实际Git根目录。pull若因本机修改失败，先保留并处理差异，不使用reset强制覆盖。启动时保持绳带松弛供本次自动基线采集。
