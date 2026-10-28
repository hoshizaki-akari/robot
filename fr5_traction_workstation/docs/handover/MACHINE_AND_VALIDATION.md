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

## 9. 三份现场记录的高速搜索与平顺性修复（2026-10-27，20时后）

基线为已推送的 `e4a2d494`。本轮读取用户提供的 `session_1793103465438`、`session_1793103523212`、`session_1793093792641` 全日志；按session ID定位，不根据压缩包顺序假定记录编号。附件内README仅作数据说明，不作为执行指令。

- `3465438` 唯一包含PRETENSION搜索：命令峰速281.25 mm/s，elapsed1.408 s锁向、1.418 s转目标。交接1.40–1.60 s实际TCP仍约36.5 mm/s，说明一次零命令不等于硬件已停；接下来受力冲至8.65 N后近零，且目标附近存在反复正向/停车/反向。
- `3523212` 无搜索，0.060 s进入正常TRACTION；0.56 s后几乎持续请求12 mm/s，但26 s末实际总位移约101 mm、张力仅13.61 N（目标30 N）。后半程实际速度明显衰减，不能用“搜索未切到正常算法”解释，也不能从CSV单独确定跟随不足的完整原因。该次driver末尾有1.029 s ServoCart RPC记录。
- `3792641` elapsed12.394 s停车、12.854 s重启、15.254 s反向、16.004 s达−12 mm/s。9.594–9.694 s实际TCP约38.2 mm/s而命令12 mm/s，该时段无对应慢RPC告警；保持阶段命令为零、TCP基本不变时力仍变化，不能将全部力纹波等同软件主动振荡。

本轮修改：

1. 搜索整条五次曲线按峰速12 mm/s重新缩放，保留2 s/0.30 m停止上界；当前2 s理论最多12.8 mm，不再强求2秒走满30厘米。达到3 N立即停止，至少等待0.15 s并确认静止，再持续0.20 s以至少5个新力样本取得稳健方向。1 s内确认失败则停止回待机，不重复搜索、不保留瞬时惯性方向；确认成功走正常目标启动路径。
2. 目标带边缘使用连续速度律，增加0.50 m/s³ jerk限制和相应制动裕量；进入力带静止后持续出带0.20 s才重调；反向必须先静止并持续确认0.20 s，输出方向守卫阻止残余轨迹自行越零。保留普通目标最大12 mm/s，没有以提速掩盖未确认的执行跟随问题。
3. 正式速度改为TwistStamped `/controller_manager/command_cart_vel_stamped`，非零输出继承manager命令源时间，零输出用当前时间。driver只接此话题，拒绝过期、错误坐标系或模式交接前的速度；两端队列只留最新消息。旧Twist话题保留兼容诊断，不能驱动硬件。三个节点必须一起更新重启。
4. 每个ServoCart包按cmdT积分，计划节拍和实际发送间隔双重限制，丢弃卡顿期间错过的时隙，不压缩/补发旧位移。ServoCart回包超过正式0.25 s看门狗锁存通信时序故障，不自动续动。运动或反馈故障先发布unhealthy；若ServoMoveEnd不成功，尝试StopMotion和RobotEnable(0)，保留待清理和故障锁存。每2 s记录实际发包Hz、非零包Hz和最大RPC耗时，供后续追踪实际速度衰减。

验证范围：按用户要求没有运行/新增测试、没有离线模拟或日志回放执行算法，没有连接SDK、调用硬件服务或启动/恢复/重启真机。本次正式overlay执行 `colcon build --symlink-install --packages-select fr_traction --cmake-args -DBUILD_TESTING=OFF` 成功（24.6 s）；只证明构建通过。实际跟随不足及指令为零时力波动的完整物理原因仍未确定，不能宣称全部实机震动已消除；此前离线测试结果不覆盖本次修改。没有提交或推送。

## 10. 搜索恢复与连续绝对轨迹修复（2026-10-27，20:40后）

用户追加两份记录并允许少量测试，随后明确确认：自动搜索应在无接触时5秒走30厘米，达到3N立即停止并确认方向。本节取代第9节的12mm/s、2秒短搜索策略；第9节仍保留为历史过程。

日志证据：`session_1793104794193` 只有约0.83mm位移，瞬时未低通力触发接触停止后，稳态仅约1.8N，1.535s以 `DIRECTION_SEARCH_CONTACT_UNSTABLE` 结束。先前策略既把最大搜索路程缩成12.8mm，又在瞬时力回落后直接终止，不足以有效张紧。`session_1793104827419` 目标27N，12.526s实际受力仅9.22N、总位移约11.31mm，而算法约2秒起持续输出12mm/s。对应driver 84200实测约100.3–100.4Hz、RPC最大3–5ms，排除这次是主机低发包率或长RPC造成；仍不能从日志证明控制箱内部增量执行为何衰减。

实施修复：

- 规划 `D=0.30m/T=5s` 的五次最小jerk搜索，理论峰0.1125m/s，manager/driver各限0.12m/s、0.30m、5s；达到3N即发零速，原静止确认保留。若停稳0.35s后瞬时尖峰消失，允许在同一次剩余预算内从静止继续，额外限制恢复加速度0.08m/s²、jerk0.25m/s³；不重置原搜索计时/起点。driver的保护也跨DISABLED暂停保留，只有新的servo接管才授权新预算。持续有力但方向不稳仍拒绝盲目移动。
- 恢复普通目标20mm/s远场上限，目标加/减速上限0.080/0.160m/s²，jerk1.0m/s³；保留连续近场速度律、出带确认和静止后换向确认。
- 目标/搜索/卸力由重复小增量下发改为显式绝对位姿轨迹：每个实际包以v×cmdT推进已命令的base位姿，ServoCart(mode=0)下发，姿态保持；零速、过期、接触停止取消未完成领先量并锚定实时TCP，模式交接重取起点。不以滞后反馈重新吞掉每个增量，也不无界追赶；新目标距实时TCP超过默认5mm即锁存故障停止，配置最多10mm。基坐标位移符号保持正式配置+1，没有改网页箭头/工具轴补偿。原生拖拽独立，遗留host drag工具增量不变。mode0语义核对本地固定SDK源码及[法奥运动接口文档](https://fairino-doc-en.readthedocs.io/3.8.0/SDKManual/PythonRobotMovement.html)；这不是已经确认某种控制箱内部故障的结论。

针对性验证（未跑全量）：

- 5个新增假执行器/驱动测试通过：有跟随延迟仍正确累计轨迹；停止丢弃领先量；执行器不响应不积压目标；长调度间隔不补跑；工具轴搜索5秒到限停止。只导入driver类，不构造ROS Node/SDK，不代表真实FR5动态已模拟。
- 16个相关旧驱动兼容检查的当前预期已更新并通过定向检查；旧夹具缺少指令看门狗字段造成的单项失败已修正、定向重跑通过，后续又单独确认暂停不重置driver搜索预算。
- 3个纯C++算法检查通过：逼近并稳定且不过度反复纠偏；非线性假负载15N在5秒内稳定；远场1秒内达到至少19.5mm/s且加速受限。
- 4个真实manager/controller隔离ROS检查通过（domain89、localhost、假反馈和假硬件服务、无driver/SDK）：搜索达到稳定3N后锁向转目标；完整无力搜索的指令积分0.30m±0.01m且峰速≤0.113m/s、5秒到限不重启；30cm实际位移上界；瞬时尖峰消失后可继续且原5秒预算不重置。合计19.53s。
- 正式overlay重建成功（22.8s），Python文件语法编译与最终路径核对见交付。未连接SDK、未启动/恢复/重启真机、未提交推送。构建及离线通过不证明实机跟随精度或所有机械震动已消除；绝对轨迹和5秒搜索尚未实机验收。

## 11. 搜索瞬时跟随保护、目标中心与免登录（2026-10-27）

用户记录 `session_1793106132470` 在PRETENSION启动1.2307s后进入 `ROS2_CONTROL_ERROR`，记录最大受力约1.97N。对应driver 88435在 `1793106133.692607751` 明确报 `FR5 Cartesian trajectory not following: 5.477 mm exceeds 5.000 mm; refusing catch-up motion`。这是上一版软件5mm单次跟随差距保护触发，经driver unhealthy转换成manager通用故障；不是本次实际受力超限，也没有证据表明是SDK返回14或机械臂控制柜自身报警。搜索当时命令约56mm/s，短时差距超过普通目标速度下设定的门槛不宜直接锁故障。

本轮修正：

- 5mm改为目标领先量软上限：新目标投影/限幅至实时TCP周围5mm，超出推进量直接丢弃，不积压等待补跑。短时达到上限仅记录warning，硬件有持续位移就继续；连续受限且1s进展不足0.5mm则故障停止。若旧目标与新反馈之间突然偏离超过2倍上限（默认10mm），仍立即故障。新下发目标始终有界，没有通过扩大累计轨迹来消除报警；5s/30cm搜索预算与通信/停止保护保持有效。
- 目标向用户设定值中心收敛。新增controller与manager共享 `position_center_tolerance_n=0.5`，外层±2.5N参数保留，但不再是提前低侧到位条件；近场连续速度静区为中心容差0.4倍（默认0.2N），进入中心±0.5N、轨迹安静且持续1s才保位。设10N时，7.5–8N不能再到位；运动中会继续向10N逼近，而非把目标提高成12.5N。
- 删除HTML登录弹窗、演示账号、登录/退出登录事件和登录session恢复，启动固定“本机操作员/本地操作”，设置与操作功能仍受真实设备状态控制，记录通过原HTTP头写入本机操作员身份；保留关闭程序/急停/恢复入口。脚本cache key更新，避免刷新仍加载旧登录脚本。

验证范围：6个假驱动用例通过（包含单次5mm越限限幅不故障、慢但持续进展可继续、无进展停止、零速取消目标）；4个C++定向检查通过（中心到位、原快速逼近和稳定性）；9个网页行为/契约检查通过（含新免登录启动操作和记录身份，旧固定cache版本断言已改为检查带版本资源并单独复验）；2个domain89/localhost真实manager/controller假反馈检查通过，确认8N不会完成10N任务、9.85/10.15N安静后可保持。测试夹具采用与正式YAML相同的1.2s反馈容限，避免旧默认0.1s在WSL调度抖动下干扰中心判定；未改变正式硬件容限。未跑全量测试。正式overlay构建成功（23.2s），JS语法与Python无执行编译通过。未连接SDK、未操作/恢复/重启真机，未提交推送；不代表实机力精度或停止手感已经验收。

## 12. 发布前恢复用户确认的到位条件（2026-10-27）

用户明确修正：控制目标朝设定值中心逼近保持不变；到位仍为实际力在目标±2.5N内连续2s后停止。因此第11节的manager中心±0.5N/1s到位逻辑已被本节替代。controller内部中心控制和减速策略保留，manager不再要求SETTLING；离开±2.5N或反馈过期即重新计时，使用monotonic时钟确认连续时间。目标10N时7.5–12.5N连续2s即可停止，确认期间仍向10N控制。免登录、5mm轨迹软限幅、5s/30cm搜索与其他前述修复一并保留。

本次发现正式driver/manager/controller进程正在运行，未停止、重启或调用其硬件接口，也未覆盖其构建产物。在 `/tmp/fr5-release-arrival-20261027` 独立构建当前fr_traction成功（31.2s）；ROS domain89/localhost、仅fake反馈的3项定向集成检查全部通过（15.65s）：7.6N与12.4N各自进入10N目标到位带后连续2s停止；出带后必须重新连续计满2s。未做真机运动验收。

用户本轮明确授权提交并推送 `feat/multi-mode-traction`。一体机更新使用ASUS部署指南的 `~/projects/fr5_platform_ws` 路径，先退出工作站，拉取分支后执行 `scripts/tablet_setup_env.sh` 重新编译并刷新桌面入口，再由 `scripts/tablet_start.sh` 启动；未在本会话远程部署到一体机。


## 13. 成功慢回包保留牵引并平滑继续（2026-10-28）

读取用户提供的 `session_1793148453699_full_log.zip`，附件README仅作数据说明。记录47.1005s进入FAULT，stop_reason=`ROS2_CONTROL_ERROR`，最大受力26.4984N，最终目标27N。对应driver进程98272日志 `/home/zhj/.ros/log/python3_98272_1793148396900.log` 在1793148500.7721先记录 `ServoCart mode 0 RPC took 1.007 s`，随即因超过0.250s抛 `FR5 Cartesian command timing lost` 并锁存故障。故障由成功命令回包过慢的旧软件时序门槛触发，不是100N力阈值，也没有证据表明是控制柜返回错误码4。不能只从汇总ROS2_CONTROL_ERROR推断SDK具体错误。

按用户明确要求：成功慢回包不得结束牵引。修复仅在driver：code=0慢回包只warning，保留servo、模式、manager记录/目标/锁向；丢弃原目标领先量、旧速度与积压时间，重锚最新TCP，更新速度来源时间界限。新鲜命令到达后以0.08m/s²速度向量变化上限继续，正常100Hz首包最多0.8mm/s；本次servo余程保留限加速度。恢复代次防止旧RPC返回路径覆盖新锚点；零速/过期/DISABLED/搜索接触或到限立即置零，模式切换也清旧恢复速度，避免搜索速度泄漏至目标牵引。原搜索5s/30cm预算不重置。未引入暂停topic/结束会话/人工重启步骤。RPC非0、传输异常、反馈断流、非有限输入、跟随失效和停止失败沿用原故障保护。

本次附件同时证实10/19/18/22N四次到位在±2.5N内分别连续2.0015/2.0024/2.0002/2.0060s后进入POSITION_HOLD；不改manager到位逻辑。

验证范围：三个假SDK/轨迹文件共65项离线检查通过，包括成功1.007s回包不中断、连续卡顿不补跑、旧来源速度拒绝、恢复加速度、四类立即置零、模式交接、搜索原预算及真实失败保护。网页/API `bash scripts/run_checks.sh` 37项通过；同时修正3处在a7880f7基线就已失效的旧源码契约断言（mode1旧写法、旧节拍负断言、1s旧到位配置）。新增恢复用例已注册为CMake/CTest测试，独立/tmp目录重新配置成功，`ctest -R ^test_servo_timing_recovery$` 通过（0.68s）。

未连接第二个SDK、未调用真机动作、未停止或重启现有ROS/网页进程；当时运行的driver需在任务正常结束后重启ROS加载；之后的本机加载确认见第14节。源码symlink不会热更新已运行Python模块。本机WSL执行沙箱初始无法创建进程，采用经自动审核通过的只读/定向修复执行路径恢复了访问；未修改机器网络或权限配置。未提交、未推送，未声称实机已经验证网络延迟期间的实际张力保持效果。


## 14. 本机运行版本确认与发布（2026-10-28）

用户报告刚测试可运行，并明确要求确认本机版本后提交推送。只读确认当前driver PID102157于本地时间09:03:27启动，晚于最终driver源码09:01:15；安装入口symlink指向本项目源码，当前进程工作目录也是本项目。日志 `/home/zhj/.ros/log/python3_102157_1793149408028.log` 已记录实际servo约99.4–100.4Hz、最大RPC0.0102s，未见旧timing-lost或ERROR。由路径和加载时间可确认此次运行加载了最新修复；尚未遇到成功长延迟，不能宣称实机已覆盖恢复分支，相关故障注入验证仍依据第13节。

manager/controller/math源码内容与a7880f7一致，本轮没有修改C++算法、ROS消息/服务或运行参数。manager正式二进制为2026-10-27 21:29构建，已去掉旧manager中心容差参数，保留独立±2.5N/2s到位；controller和core均在相应源码修改后构建。manager源码较新mtime来自本轮改后撤销，不能用mtime单独推断需要重编。CMake只新增离线pytest注册，正式BUILD_TESTING=OFF运行无需额外编译。

发布前已核对远端feat/multi-mode-traction仍为a7880f7，未出现他人新提交；最终补丁已完成第13节的65项驱动及37项网页/API验证。ASUS部署指南和RUNBOOK已改为已有环境仅pull+本地colcon增量编译，避免每次重复完整联网安装。用户授权发布，不重写旧提交或强推；当前会话未停止/重启/恢复机器人，未在一体机执行部署。


## 15. 实体拖拽退出校准、5N合力门控和1.5s到位（2026-10-28）

用户确认：实体末端拖动示教退出时，无论绳带是否带载，都把当时当前受力重新设为软件零点；省力拖拽按界面三向合力严格>5N触发，降回≤5N停住、再次超过5N继续；目标到位容差保留±2.5N，连续确认时间缩为1.5s。

实现：driver只读SDK同一实时包的robot_state 4（拖动）→1（停止）与robot_mode，不调用额外RPC查询灯光；排除工作站原生力拖拽、servo及其尾帧、启动首次停止及故障状态。退出事件携带同包base系力，manager在空闲/软件拖拽等待施力、静止健康、无交接时软件tare，清旧候选/记录方向；活动目标或卸力期间不改零，事件保留至正常结束，故障丢弃。未调用FT_SetZero、不写回零位姿。真实灯光颜色与SDK状态映射仍需现场确认。

省力拖拽先进入DRAGGING待施力，不直接打开原生控制；按与页面一致的0.1N显示精度判断，>5N持续0.1s请求ON、≤5N请求OFF，下降沿不增加等待。每次ON/OFF沿用driver确认和失败清理，OFF后仍保持本次会话；不改为host ServoCart，不移除厂商各轴5N阈值。实际OFF延迟受RPC和控制柜确认约束，软件请求不能等同于物理瞬时停止。实体拖动活动期间禁止软件接管；退出状态与受力事件跨话题乱序时，必须等校准完成才重新激活软件拖拽。请求代次防止结束/卸力时晚到ON回复重启。

到位时间1.5s同时改manager默认、成员初值与正式YAML；仍为monotonic连续计时，出带或反馈过期清零；方向候选更新2s、搜索5s/30cm、延迟成功回包继续策略保持原值。

验证：独立 `/tmp/fr5-teach-gate-20261028` 编译，未覆盖正式运行产物；70项假SDK/轨迹/实体边沿检查通过（0.28s），网页/API检查37项通过。完整隔离ROS域89/localhost工作流16项通过（58.42s），覆盖1.5秒实际到位计时、带载自动tare、≤5N边界、重复施力、实体按钮与退出力消息乱序互锁及异步卸力/停止交接；新增保持参考系的门控停止接口后，仅受影响的3项专项复验通过（8.47s）。最终独立增量构建成功（21.5s）。未操作、恢复或重启真机，未提交推送。修改必须在当前任务正常结束后重新构建正式overlay并重启ROS加载；源码symlink不会热更新正在运行的Python模块。

补充：5N门控OFF使用专用停止服务保留力参考系，下一次ON不重复设置；最终停止/卸力仍恢复base参考系。假SDK覆盖完整门控OFF/ON/最终停止顺序，确认门控循环只配置一次，最终停止才恢复一次。


## 16. 一体机同时开机时网页优先启动与发布（2026-10-28）

本轮工作目录已迁移到 `/home/zhj/.codex/worktrees/836c/robot/fr5_traction_workstation`，HEAD基线c88ce93，当前为分离HEAD；保留了第15节尚未提交的全部变更。用户要求网页先启动，并明确授权修改后推送GitHub。发布同时包含第15节实体示教退出tare、5N合力门控和1.5s到位。

`tablet_start.sh`改为先启动网页，HTTP可用即打开专用Firefox全屏；后台约3秒探测一次机器人20003，无机器人时界面保持“等待机械臂连接”、动作按钮禁用。端口可用后开始单次ROS启动，45秒内无connected日志、driver初始化异常或进程退出时先清理该次进程组后重试，每次尝试独立日志。首次driver connected后停止自动重启策略，保留运行故障/基线/任务语义；运行控制栈退出后网页仍在，需用户处理。关闭窗口或supervisor收到退出信号均取消等待并清理本次web/ROS进程组，探测返回后再次检查窗口存活，防止取消后迟到的SDK启动。PID文件仅由自身实例移除。

页面初始HTML和实时状态显示“等待机械臂连接”，真实反馈出现后自动转在线；曾连接后断线显示“机械臂连接中断”。RosBridge不依赖机器人服务才能创建节点，不改桥接为模拟在线；无自动开始牵引操作。ASUS部署指南已补充自动登录/autostart配置，可去掉旧sleep20。环境缺失/网页自身启动失败仍报告错误。

验证：`bash -n scripts/tablet_start.sh`与diff检查通过；`bash scripts/run_checks.sh`编译/ruff及43项检查通过（17.896s），包含5项真实监督脚本+假进程行为检查，以及网页离线→后上线→断线按钮约束。初次受限运行因Windows Node互操作被沙箱阻止且测试套件受影响，已停止仅该次离线测试进程，在权限恢复后完成全部检查。启动回归完全在/tmp复制脚本配合假web/Firefox/端口/ROS进程，不访问机器人端口、不运行正式launch或GUI。一体机实际同时冷启动效果仍待现场确认。

发布前只读确认GitHub feat/multi-mode-traction为c88ce93；以正常快进方式推送当前提交到该分支，不强推、不改变另一工作树的分支或文件。不在本机安装自启动配置，不停止/重启真机，不在目标一体机执行部署。

## 17. 只读状态、三个动作位与有限自动恢复（2026-10-28）

工作目录仍为 `/home/zhj/.codex/worktrees/836c/robot/fr5_traction_workstation`，分离HEAD基线 `4d2942b3ccbb92d8f2abeecde9649683866c58af`，本轮开始工作树干净。用户确认“进入”先结束当前动作再切模式、随后手动开始；自动恢复仅限明确暂时性通信问题，恢复后不续动；后续明确废弃网页急停按钮。

页面上方改为只读“拖拽／牵引／卸力”状态；底部按当前状态提供三个动作位，切换走新 `/api/traction/enter-mode`，等待manager停止确认、ready和关节静止后选择模式，保持基线、不自动开始。manager的ready补充原生OFF待确认、Cartesian接管和实体示教/退出置零约束。卸力中可先停止再进入其他状态。网页急停按钮及绑定已删除，故障时顶栏独立显示“恢复设备”；原HTTP/ROS兼容停止服务、实体急停和硬件保护仍保留。

新增web后台恢复轮询，只对白名单WRENCH_TIMEOUT、EE_STATE_TIMEOUT、UI_HEARTBEAT_TIMEOUT尝试复位。连接静止连续1秒后先执行driver被动检查，再调用manager专用 `/traction/reset_transient_fault`，回调内重查当前故障及实时数据；即使检查后变成急停或其他故障也不误清除。最多3次、间隔至少2秒，失败保留人工处理。无SDK复位/清错误/使能/切模式/置零/启动；driver已锁故障、停止失败和控制柜保护不会自动解除。

验证结果：

- `bash scripts/run_checks.sh` 的Python编译、ruff及55项检查全部通过（18.230s），使用原checkout虚拟环境和Windows Node；包含真实JS在DOM替身中的三个动作位、进入不启动/不校准、独立故障恢复及不存在网页急停入口，bridge等待停止确认、停止失败/保护中断，以及自动恢复白名单、静止等待和三次上限。
- 当前C++ manager独立colcon构建成功（24.1s），输出 `/tmp/fr5-context-actions-20261028/install`，复用原overlay的fairino_msgs等依赖，未替换正式install。
- `ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1` 隔离合成反馈工作流5项通过（14.48s）：原生停止确认前禁止切换、延迟拖拽卸力交接取消、合力门控、通信恢复不续动/基线保留、急停拒绝自动复位。仅启动该测试install的manager/controller和假反馈节点，未启动driver或正式launch。
- 假SDK驱动71项通过（0.35s），覆盖原生拖拽、绝对Servo轨迹、成功慢回包恢复及实体示教退出；新增被动恢复检查不发送SDK命令、拒绝保护/锁存/未清理状态。

没有访问机器人端口、操作真机、重启现有工作站或做真实浏览器布局验收。后续用户明确授权本地提交并推送GitHub，发布目标为feat/multi-mode-traction，采用正常快进推送。本机主目录 `/home/zhj/projects/robot` 存在未提交修改，保留原文件，仅在本工作树提交发布。修改尚未加载到运行中的网页与ROS；正式生效须在允许停机时构建本项目ros2_overlay，并重启ROS和网页，不要另起第二个SDK所有者。
