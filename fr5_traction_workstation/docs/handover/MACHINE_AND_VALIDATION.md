# 本机现状、验证记录与下一步

检查日期：2026-10-26（Asia/Shanghai）。项目：`/home/zhj/projects/robot/fr5_traction_workstation`，Git 根目录 `/home/zhj/projects/robot`。这是接手时快照，后续运行前需重新核对。

## 1. 已确认的机器状态

| 项目 | 本次结果 |
| --- | --- |
| 系统 | Ubuntu 22.04.5 LTS，x86_64，WSL2；内核 `6.18.54.1-microsoft-standard-WSL2` |
| 分支/基线 | `feat/multi-mode-traction`，HEAD `881117c007c1b77e722c9e5879975f7bf3977d4b` |
| Git 原有改动 | 两个 tests 文件的嵌套 with 合并，未改测试语义；本次保留 |
| ROS | `/opt/ros/humble` 存在；source 后 rclpy、自定义消息/服务导入成功，未调用 rclpy.init |
| 当前 ROS 包路径 | source 推荐 overlay 后 `fr_traction` 指向 `ros2_overlay/install/fr_traction` |
| Python | 系统/父级 venv 均为 3.10.12，venv `include-system-site-packages=true` |
| 网页依赖 | FastAPI 0.116.1、uvicorn 0.35.0、websockets 15.0.1、pydantic 2.11.7、ruff 0.16.5，与 requirements 一致 |
| 测试依赖 | pytest 8.4.2、g++/gtest 可用；httpx 未安装，不依赖 TestClient 完成此次 HTTP 验证 |
| SDK | vendor 文件存在，HEAD 为部署脚本固定值 `62add0e6b7c7d2e157beaaa4485042c9d54e1f84` |
| SDK 导入 | 通过 driver `_load_robot_sdk` 成功，未构造 Robot.RPC；直接 import SDK 因无 Cython 失败，这是 loader 已处理的兼容点 |
| ROS 描述/编译产物 | `fairino_description`、overlay build/install 存在；overlay BUILD_TESTING=OFF，项目根另一套 build 的 BUILD_TESTING=ON |
| 服务/进程 | 初次检查未发现工作站 uvicorn、牵引 driver/manager/controller 或旧真机服务；未见 8081 监听 |
| 持久化设置 | 检查时未见 `~/.local/state/fr5-traction`，未发现本用户已有零位/参数文件；现场仍需再查 |
| WSL 网络 | eth0 `192.168.21.161/20`，默认经 `192.168.16.1`；`ip route get 192.168.58.2` 同样走该默认网关 |

首轮接手时的 WSL 路由输出只说明 Linux 将包交给宿主方向，不能证明宿主已配置机器人有线网卡，也不能据此断定连通或不通。首轮没有 ping、探测机器人 TCP 端口、读取控制箱状态或检查 Windows 网卡。之后为现场准备所做的完整编译、软件联调与 Windows 检查见第 5 节。

## 2. 实际执行的离线验证

| 检查 | 结果 | 覆盖与限制 |
| --- | --- | --- |
| `bash scripts/run_checks.sh` | 通过，25 个 unittest | compileall、ruff、网页/API/bridge/配置源码合约；不启动 ROS 或硬件 |
| driver `pytest` | 21 通过 | 假 SDK 的零位、原生拖拽、使能/模式、停止/恢复和参考系；无实体连接 |
| 当前源码重新编译纯 C++ gtest | 51 通过、7 suites | 48 核心测试 + reference 的 3 个 finite_checks；没有复用旧二进制 |
| `bash -n run_workstation.sh scripts/*.sh` | 通过 | shell 语法；未运行安装和真机启动脚本 |
| 自定义 ROS 类型与 SDK loader 导入 | 通过 | 可加载接口，不证明 ROS graph、网络和真实硬件 |
| 禁用 bridge 的真实 uvicorn/HTTP/WS 检查 | 通过 | 127.0.0.1 临时 18082；检查后进程已停止 |

HTTP 检查对 `/`、`/assets/090105.js`、`/assets/robot_viewer.js`、本地 Three.js、base_link STL、`/api/state`、`/api/traction/history`、摘要 CSV 都得到 200；WS 得到 `connected=false` 的真实离线快照。`/api/health` 的 ok=false 符合禁用连接的预期。没有发任何运动/恢复/校准 POST，没有写入正式会话或设置。此次未做浏览器 WebGL/布局/JavaScript 执行验收。

核心测试编译到 `/tmp/fr5-handover-tests-V8Zhaq/offline_core_tests`，XML 为同目录 `results.xml`，临时目录以后可能被系统清理；永久依据是本记录和可复现命令。CONTROL.md 记录了 48 项核心测试的编译方式，本次另加入 reference 3 项检查，完整命令为：

```bash
cd /home/zhj/projects/robot/fr5_traction_workstation
HANDOVER_TEST_DIR="$(mktemp -d /tmp/fr5-handover-tests-XXXXXX)"
g++ -std=c++17 -O2 -pthread \
  -Iros2_overlay/src/fr_traction/include \
  -Iros2_overlay/reference/fairino_hardware_include \
  ros2_overlay/src/fr_traction/src/traction_math.cpp \
  ros2_overlay/src/fr_traction/src/traction_state_machine.cpp \
  ros2_overlay/src/fr_traction/src/traction_safety.cpp \
  ros2_overlay/src/fr_traction/src/traction_controller_core.cpp \
  ros2_overlay/src/fr_traction/src/direction_correction.cpp \
  ros2_overlay/src/fr_traction/test/test_traction_math.cpp \
  ros2_overlay/src/fr_traction/test/test_traction_state_machine.cpp \
  ros2_overlay/src/fr_traction/test/test_traction_safety.cpp \
  ros2_overlay/src/fr_traction/test/test_traction_controller_core.cpp \
  ros2_overlay/src/fr_traction/test/test_traction_synthetic_pipeline.cpp \
  ros2_overlay/src/fr_traction/test/test_direction_correction.cpp \
  ros2_overlay/reference/fairino_hardware_test/test_finite_checks.cpp \
  -lgtest_main -lgtest -o "$HANDOVER_TEST_DIR/offline_core_tests"
"$HANDOVER_TEST_DIR/offline_core_tests" --gtest_brief=1 \
  --gtest_output="xml:$HANDOVER_TEST_DIR/results.xml"
```

这是首轮接手的纯算法/参考函数测试，不编译 manager/controller ROS 节点或 legacy 硬件插件。首轮未做全套 colcon build/test；之后的正式四包重新编译及真实软件节点联调见第 5 节。改 ROS 代码时仍应按 RUNBOOK 重建推荐工作区。

## 3. 接手时识别、尚未修改的问题

以下不影响本次文档交付，后续任务应按涉及范围处理；静态推导的现场后果没有在硬件上复现。

| 问题 | 证据入口 | 后续从何下手 |
| --- | --- | --- |
| POSITION_HOLD 发生安全故障时缺少 FAULT 转换 | `can_transition` 未包含 hold，`check_safety` 却检查它 | 状态边、enter_fault 与服务/UI 联动测试；详见 CONTROL |
| direction_correction_mode=off 可能持续暂停恒力 | estimate 默认 SENSOR_HOLD，manager 恒力分支只发 RELEASING | 给关闭方向修正定义正确固定方向语义，补 manager 组合验证 |
| 正常 stop 后保留方向直接重新 start，可能没有新会话 CSV | stop finalize，start 无 begin_session | 明确重复启动的记录生命周期，补状态/数据集成测试 |
| 原生 drag ON 已执行但响应丢失时，异常清理曾可能漏发 OFF | 2026-10-27 已改为发送 ON 前占用控制权；Fake SDK 回归通过 | 软件缺口已修；实机故障注入未执行，详见 HARDWARE |
| 回零六维位姿插值时长只按 XYZ 计算 | driver `_start_return_to_pose` 与 `_send_motion` 的 return 分支 | 同位移不同姿态/角速度和路径约束需独立审查 |
| healthy/READY、frame 名称的能力边界 | driver 本地锁存健康；reference timeout 可沿用旧值 | 不以 bool、名称代替实际模式、传感器与参考系核验 |
| 预检存在漏项且缺项通常 exit 0；旧 runtime checker 用错话题 | 两个检查脚本 | 后续需要部署保障时改为明确退出状态、新鲜度和唯一发布者验证 |
| UI 登录是演示角色；服务端无真正会话认证 | JS users 与 app header 检查 | 对外访问前完成真实认证/授权；详见 WEB_AND_DATA |
| 数据目录可独立修改，但 HTTP 导出根固定 | manager data_directory vs app SESSION_ROOT | 新数据路径/迁移需求需统一配置与路径映射 |
| 默认无页面心跳强制保护；无完整仿真后端 | YAML `require_ui_heartbeat=false`、launch 仍直连 | 演示和停机语义需明确；不要以 use_sim_time 作为硬件隔离 |

`can_transition(POSITION_HOLD, FAULT)==false` 和默认 estimate 为 SENSOR_HOLD 已通过离线探针核对；manager 端到端后果仍需对应的组合场景验证。测试全通过说明已有测试场景通过，不能覆盖上表遗漏场景。

## 4. 下一次任务如何进入

- **改算法**：先读 CONTROL 的对应模式和参数流，确认改到主控制实例；保留基线数据，跑 C++ 回归，改到 ROS 状态/坐标时另做组合验证。
- **改界面**：读 WEB_AND_DATA，改 HTML/CSS/JS，跑 Python/UI 合约，再用禁用 bridge 的预览做浏览器检查；不要用假在线数值掩盖断线。
- **准备真机运行**：先按 HARDWARE/RUNBOOK 核对 Windows/WSL 或原生网卡、控制箱/传感器、单 SDK 所有者、零位和现场约束，再启动并验证 graph/反馈/READY。
- **做演示**：离线仅可展示布局和断线状态；动态三模式演示需真实设备或单独实现明确隔离的仿真数据源。本机尚未验收实际拖动方向、回零路径、100 Hz 或 1～100 N 性能。
- **迁移/备份**：代码不足以恢复机器状态；同时保存记录、operations、settings、zero_pose、SDK 固定版本和描述包来源，按新安装条件重新确认零位。

首轮接手只新增资料、自动入口和修正文档漂移，没有修改业务代码、控制参数或原有测试修改。后续完成任何实体验证，都应追加日期、实际机器、分支/提交、运行参数、记录 ID 与结果，替换“待现场确认”为真实证据。

## 5. 现场准备复查（2026-10-26）

为“本电脑到现场插机械臂运行”补做以下实际工作：

| 项目 | 结果 |
| --- | --- |
| 正式四包完整编译 | `fairino_description/fairino_msgs/fairino5_v6_moveit2_config/fr_traction` 全部成功，当前 overlay 产物已更新；BUILD_TESTING=OFF |
| 系统 ROS 依赖 | `rosdep check --from-paths ...fairino_msgs ...fr_traction --ignore-src --rosdistro humble` 满足全部依赖 |
| Python 缺项修复 | 当前用户安装 cffi 2.1.1、pycparser 3.11，补齐既有 PyNaCl 的依赖；系统 Python 与项目 venv 的 pip check 均无缺项 |
| 可复用离线预检 | 新增 `scripts/check_local_env.sh`，实际 7 组通过、exit 0、LOCAL_ENV_READY |
| 网页/驱动回归 | 本轮重新运行根 25 项 unittest、ruff/compileall，以及 driver 21 项假 SDK pytest，全通过 |
| 真实 ROS 软件节点联调 | 独立域 87、localhost only；新编译 manager/controller、robot_state_publisher、真实网页启动脚本完成状态交换；没有启动 driver |
| 隔离测试结果 | manager READY、controller healthy；144 帧速度全零；22 帧 TF 覆盖 6 个活动关节链；HTTP health/state connected=true，WS sequence 19→31→43 |
| 测试数据隔离 | 137 帧/话题的静止合成反馈仅在独立测试域；没有 prepare/start 或任何 POST，不构成实机反馈/牵引记录 |
| 清理 | 四个子进程均正常退出、PID 消失、临时网页端口 18083 已释放 |
| Windows 浏览器链路 | Edge 已安装；Windows 对 WSL 临时 127.0.0.1:18084 页面请求返回 200，临时服务已停止 |
| Windows 物理网口 | Realtek PCIe GbE Family Controller，名称“以太网”，ifIndex 13，GUID `6364643c-8787-4077-83a6-9042b445f162`，目前未插线 |
| 网络准备 | 已完成并独立复核专用 Ethernet 的持久 `192.168.58.10/24`、DHCP Disabled、无网关/手动DNS；Wi-Fi 默认路由保留。网线未插，结果 prepared_offline，未做真实通信验收 |

联调原始证据位于 Git 忽略目录 `debug/environment_checks/20261026/software_integration/` 的 result.json 和四份日志。该测试证明主软件进程可以启动、加载动态库、发现 ROS 接口并由真实 rclpy 桥接到网页，不能证明 SDK 与控制箱的实时通信或实际运动效果。

最终网络核验输出位于 `debug/environment_checks/20261026/windows_network.json`。Windows 最终备份为 `%LOCALAPPDATA%\fr5-traction\network_backup_20261026_164113_021.json`，最初 DHCP 记录也保留；重复 Apply 已可只读检查，不覆盖备份。初次通过裸进程名请求提权未完成；修复完整 `$PSHOME\powershell.exe` 路径与错误处理后，实际设置成功，未要求用户手动修改 UAC 策略。

本轮仍未连接机器人或执行硬件动作。有线网口离线预设已完成；插线后的活动地址/路由与 20003/20004 状态流，以及控制器/传感器实际状态，仍需按现场速查确认。网络脚本默认只查看；实际 Apply/Restore 才修改指定物理网口并需要管理员权限，已满足目标配置时 Apply 直接只读返回。

## 6. 省力拖拽启动故障修复（2026-10-27）

- 本聊天 WSL shell 连接已恢复，可访问项目与 Windows Downloads 的现场导出。
- 导出 `session_1793088080813`：模式2，249帧 MANUAL_SETUP + 1帧 FAULT，持续2.487秒；停止原因为启用状态未确认。代码只在 ON RPC 返回0后走到此原因，旧逻辑仅查询4次，名义约0.4秒。导出未包含控制器错误码/drag状态，具体硬件拒绝原因或延迟仍不能确定。
- 修复驱动的启用确认为默认2秒的可配置时间窗口，等待中继续校验实时反馈与控制器就绪；保留独立的短OFF确认窗口与失败去使能。同时修复ON已执行但响应超时导致漏关拖拽的问题，失败原因附加清理前控制器诊断。未加入、也没有需要回滚的提前限位/向安全区退回功能修改。
- 假SDK先复现6项新增回归在旧代码失败；修复后驱动30项pytest、修改文件ruff、根目录25项unittest/ruff/compileall通过。两份原有用户测试文件内容哈希保持不变。无C++算法改动。
- 正式 `ros2_overlay` 执行 `colcon build --symlink-install --packages-select fr_traction --cmake-args -DBUILD_TESTING=OFF` 成功；离线 `check_local_env.sh` 七组通过并输出 LOCAL_ENV_READY。预检同时提示当前 ROS_LOCALHOST_ONLY=0 和8081已有进程，未修改该环境或终止占用者。
- 只读查看到仍有旧ROS/网页进程：driver PID50247、manager PID50251、web PID50300，使用本项目 `ros2_overlay/install`；driver在1793088224（会话故障约141秒后）开始记录实时流过期。该后续断流与启动时未确认是两个时间点的问题，不能用后续断流反推本次故障原因。
- 本轮没有新建SDK连接、调用硬件服务或重启原有进程。Python install入口是源码符号链接，但已运行的Python进程持有旧代码；让修改生效需要现场确认机械臂静止、允许重新接管后重启正式ROS入口。重启并非只读操作，会配置参考系并清理遗留拖拽。浏览器/API源码未改。


## 7. 三模式流程更新（2026-10-27，用户要求停止追加测试）

当前实现为省力拖拽/目标牵引/卸力，自动启动基线、3N/2秒待机方向、30cm/2秒工具Z−搜索、±2.5N到位、<5N卸力停止；移除回零和软件实际受力超限故障。默认100N目标上限可经设置和ROS原子参数同步提高。结束/切模式/恢复不重校准。

停止追加测试前已完成：独立源码构建；56项C++功能回归、45项假SDK驱动回归、36项网页/API检查（含5项Node行为测试），以及10项独立domain89/localhost的真实manager/controller合成反馈集成场景通过。新增integration文件的ament_flake8/pep257已单独修复并通过，没有在用户要求停止后重跑测试。cppcheck被系统已知版本规则跳过，不能声称静态分析全覆盖。

真机仅订阅既有反馈：约100Hz、healthy=true、关节静止、原始力约31N而已去基线力约0.4N；不代表新运动验收。本轮未新建SDK连接，未发硬件服务，未重启现场进程；正式运行目录完成编译后，现有进程仍持有旧代码。现场准备好后重启ROS与网页，打开时保持绳带松弛，以获得一次新的启动基线。未提交或推送。

## 8. 实机震动与停止故障修正（2026-10-27）

现场记录 `session_1793092272605`：一次工具Z−搜索中速度诊断最高约0.230 m/s；目标43N阶段力约39–42N附近出现多次减速/再加速。`session_1793092349579` 以 `TARGET_TRACTION_STOPPED` 结束，随后当前manager在伺服停止服务响应失败后进入 `CARTESIAN_STOP_FAILED`。同一driver日志显示 `ServoMoveEnd` 连续返回14，两次重试均失败；仅凭日志不能确定厂商错误14的具体含义。该次退出记录先于故障回调，因此其summary本身没有FAULT。

driver日志同次还有 `ServoCart` RPC超过1秒的记录。旧代码以最多50ms的实际间隔计算增量，但cmdT为10ms；延迟后可能把超过一个控制周期的位移压入10ms命令。现将增量时间上限改为当前一个发送周期，丢弃延迟期间未发送的增量；目标速度由20降为12mm/s，目标加/减速由0.080/0.160降为0.025/0.050 m/s²。搜索30cm/2秒要求不变，其峰值速度仍可能约0.281m/s；不能由本次改动断言搜索阶段不会震动。

未确认ServoMoveEnd时不再继续发送Cartesian命令，保留servo cleanup待处理，锁硬件故障并尝试StopMotion和RobotEnable(0)；界面FAULT时提供“故障恢复”。manager保存真实的停止故障原因，避免页面显示上次正常停止原因掩盖故障。不能把返回14当作成功或自动清故障。已编译正式overlay；按用户要求本次没有运行测试，没有操作/恢复真机，运行中的旧进程需要现场确认后重启才能加载新版。真实振动、错误14的控制箱含义和恢复结果未实机验证。

2026-10-27 启动复现：direct driver 连接 `192.168.58.2:20003` 后，初始化 `FT_SetRCS(1, zeros)` 返回 SDK 错误14并退出；该码在 FAIRINO SDK 错误码表中为“接口执行失败”，建议核查 WebApp/示教器报警及实时状态。它与本日 `ServoMoveEnd` 的14相同，但日志无法证明具体控制箱报警。启动不应把该非零码当作先前力参考系仍有效；现场先在 WebApp/示教器确认并处理报警后重新启动。仅将驱动异常改为可操作的诊断文本；本次没有测试、重建或向真机发送清故障/运动命令。
