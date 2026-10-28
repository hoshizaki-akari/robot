# FR5 / KWR75D 硬件、网络与直驱交接

当前行为更新：2026-10-27。本文描述新工作树；用户已连接真机并允许只读状态查询，但本次新流程尚未替换原运行栈，也未做实机运动验收。源码、离线测试和实际控制柜状态须分别核对，最终记录见 MACHINE_AND_VALIDATION。

阅读入口：[交接索引](README.md)。本页重点是“机器人如何连接、什么实际下发到控制器、出问题查哪里”；管理器状态机与算法细节见交接目录中的架构和运行文档。

## 1. 当前正式链路及职责

```text
浏览器 :8081
   │ HTTP / WebSocket
FastAPI app.py → backend/ros_bridge.py（ROS 客户端）
   │ ROS 2 话题 / 服务
traction_manager_node（流程、基线、方向、安全、记录）
traction_controller_node（速度算法）
   │ /controller_manager/command_cart_vel_stamped (TwistStamped)
fr5_direct_driver_node.py（唯一 Fairino Python SDK 所有者）
   ├─ TCP 20003 XML-RPC → FR5 控制器：配置、模式、使能、ServoCart、拖拽
   └─ TCP 20004         ← FR5 控制器：实时状态包、关节、TCP、六维力
                                  ↑
                      KWR75D + 带子法兰 + 牵引带
```

正式入口是 `scripts/start_ros_stack.sh 2` → `fr_traction/traction_system.launch.py`。默认只启动 `robot_state_publisher`、Python 直驱、C++ 牵引控制器和 C++ 管理器；RViz 与 rosbridge 是可选项。此启动文件不启动 MoveGroup、`ros2_control_node`、官方旧机器人驱动、夹爪、视觉、Gazebo 或力传感器 broadcaster。

- **直驱负责**：拥有 SDK 连接，发布真实反馈，切自动/手动模式与使能，开始/结束 Cartesian servo，执行目标/卸力 Cartesian 速度、方向搜索的独立限速/位移/时长约束、原生辅助拖拽、硬件停止和恢复。回零与设置零点不再公开。
- **管理器负责**：松绳软件基线、运行状态、允许何时运动、待机方向估计、开始锁定、自动搜索/卸力、故障和会话记录。直驱不是完整的业务安全状态机。
- **算法节点负责**：生成速度。它不连接 SDK，不直接配置 KWR75D。
- **网页负责**：显示真实 ROS 反馈和发送流程操作，不自行读传感器或生成假反馈。
- `/controller_manager/*` 是沿用的兼容命名；当前链路中这些反馈、`switch_controller` 服务实际由直驱提供。不能因为名字就认定存在真正的 controller manager 或已加载 `fr_controllers` 插件。

**禁止同时启动第二个 SDK 所有者。** 旧 `state_service` 真机读取、`fr_robot_driver`、`fr_force`、旧 ConstantForce、旧 `fairino_hardware` 或临时 Python SDK 调试脚本均可能争用机器人。`start_ros_stack.sh` 只识别并停止当前用户下指定的 traction launch/driver/manager/controller 进程，不保证找出其他名称、其他用户、其他电脑的连接者。

## 2. 接线与网络：已知条件和现场空白

| 项目 | 源码 / 文档确认的内容 | 仍需现场确认 |
| --- | --- | --- |
| 机器人 | FR5，默认控制器 IP `192.168.58.2` | 控制器型号、固件/SDK匹配、实际 IP、控制箱网络口 |
| 控制机 | 目标部署为 Ubuntu 22.04 x86_64 + ROS 2 Humble + Firefox；项目也有早期 WSL2 历史 | 当前机器 OS、实际网卡与 WSL/原生拓扑，不能套用旧用户名/绝对路径 |
| 网线 | 部署指南建议控制机有线 `192.168.58.10/24`，直连机器人；有线无网关、无 DNS；Wi-Fi 上互联网 | 实际网卡名、链路灯、是否经交换机/VLAN/USB网卡，以及地址冲突 |
| SDK 命令 | 驱动连接机器人 `20003/TCP`，XML-RPC | 防火墙、路由和端口可达性 |
| SDK 反馈 | SDK 连接机器人 `20004/TCP`，持续接收二进制实时状态 | 真实反馈周期、报文兼容性、丢包/断流 |
| 力传感器 | KWR75D，数据由 FR5 `ft_sensor_data[6]` 回传；项目无独立 KWR75D IP/串口读取器 | 传感器电源、接插件/引脚、末端总线、控制器内厂商/型号配置、重量和重心补偿、安装轴向 |
| 工装 | 末端传感器 + 带子法兰 + 牵引带，历史测试是固定模型 | 当前绳路、法兰/工具/TCP定义、碰撞与可达空间、实体急停位置及接线 |

原生 Linux 的建议网卡配置来自 [ASUS 部署指南](../ASUS_A4131_部署指南.md)，不是本次实测。WSL 网络必须分别核对 Linux 到控制器和 Windows 主机到控制器的路径；本仓库没有自动配置 WSL 路由的脚本。不要把电脑网页 `127.0.0.1:8081` 和旧 SDK 参考代码中的机器人 `8081` 反馈端口混为一谈；正式 Python 链路使用 `20003/20004`。

## 3. 配置来源、覆盖顺序与容易误判的开关

### 3.1 部署、启动与环境变量

| 配置 | 当前作用 |
| --- | --- |
| `FR5_ROBOT_IP` | `start_ros_stack.sh` / `tablet_start.sh` 默认 `192.168.58.2`；传入 launch 的 `robot_ip`。手动 `ros2 launch` 时直接传 `robot_ip:=...` |
| `FR5_SDK_PYTHON_PATH` | 开发启动脚本默认本项目 `vendor/fairino-python-sdk/linux`；launch → 驱动 `sdk_python_path`。桌面启动强制导出该项目内路径 |
| `FR5_USE_RVIZ` | 开发启动的 RViz 开关，默认 false |
| `FR5_TRACTION_DATA_DIR` | 管理器数据路径；开发启动默认相对 `debug/traction_sessions`，相对进程工作目录解析；桌面启动导出项目内绝对路径 |
| `FR5_ZERO_SENSOR_ON_ACTIVATE` | 仅送进 URDF 中旧硬件插件的 `zero_sensor_on_activate`。**当前 Python 直驱没有读取或执行它，不会因此执行硬件 `FT_SetZero`** |
| `FR5_ROS_WS` | 预检和网页启动可用于选择运行时；**当前版本 2 的 `start_ros_stack.sh` 固定本项目 `ros2_overlay`，不根据它选择直驱工作区** |
| `FR5_BASELINE_WS` | 启动脚本版本 1 的归档稳定版运行时，默认项目父目录 `runtimes/force_stable_v1`；需要实际存在，不由本仓库保证 |
| `WORKSTATION_PORT` | 网页端口，默认 `8081`，与机器人命令端口无关 |
| `GITHUB_MIRROR_PREFIX` | 首次安装的官方仓库下载前缀，默认 `https://gh-proxy.com/`；显式空串走脚本中的官方 SSH URL，不是硬件网络设置 |

`tablet_setup_env.sh` 固定 SDK 提交 `62add0e6b7c7d2e157beaaa4485042c9d54e1f84`，从 FAIR-INNOVATION 官方 Python SDK 仓库稀疏获取 `Robot.py`、LICENSE 和 README。机器人描述从官方 `frcobot_ros2` 稀疏获取，**未固定提交**。SDK 与机器人描述是忽略依赖，不要因为本机有文件就认为新克隆包含它们。安装脚本可能删除并重建缺失/版本不匹配的 SDK 目录，属于部署操作，不是只读诊断。

### 3.2 直驱参数：代码默认值不等于正式启动值

一般优先顺序是：节点内默认值 < 启动传入的 ROS 参数。修改 `traction_params.yaml` 不会改变直驱内部参数；直驱的正式值在 `traction_system.launch.py` 的 `direct_driver` 中。安装工作区也必须与源码同步，排查时使用 `ros2 pkg prefix fr_traction`、参数查询和实际进程路径确认运行的副本。

| 参数 | 类内默认 | 正式 launch 值 / 说明 |
| --- | --- | --- |
| `update_rate_hz` | 100 Hz | 100 Hz，反馈线程轮询 SDK 最新包；不保证每次都有新硬件包 |
| `motion_rate_hz` | 25 Hz | **100 Hz**，motion timer，与反馈线程独立 |
| `command_timeout_s` | 0.10 s | **0.25 s**，过期速度置零 |
| `command_rpc_timeout_s` | 1.5 s | 1.5 s，命令 XML-RPC socket timeout |
| `startup_rpc_timeout_s` | 10 s | 10 s，参考系配置使用较长超时 |
| `allow_existing_force_reference` | true | true：启动 `FT_SetRCS(base)` 超时/传输异常时允许继续沿用控制器已有参考系并打印警告；非零返回码仍报错。SDK 返回 14 表示接口执行失败，检查示教器/WebApp 报警和实时状态，不可按“沿用已有参考系”处理 |
| `max_linear_speed_mps` | 0.025 | **0.050**，普通 Cartesian 线速度上限；当前目标和卸力core另限0.020 m/s，方向搜索使用独立上限 |
| `direction_search_max_speed_mps` | 0.12 | 0.12m/s，manager与driver一致；正式5s/30cm曲线理论峰0.1125m/s |
| `direction_search_max_travel_m` | 0.30 | 单次搜索实际起点位移 0.30 m；到限锁止本次搜索 |
| `direction_search_max_duration_s` | 5.0 | 单次搜索独立monotonic时长5s；接触暂停不重置 |
| `max_servo_tracking_error_mm` | 5.0 | 5mm，绝对轨迹目标领先实时TCP的软上限，超出推进量丢弃；1s进展不足0.5mm或旧目标偏差超2倍上限才故障；此参数配置不得高于10mm |
| `base_servo_sign` | **-1.0** | **+1.0**，来自历史实测，不能跳过 launch 用类默认运行 |
| `realtime_state_timeout_s` | 0.5 s | 未在 launch 覆盖；SDK 包对象连续未更新超过该值触发直驱故障锁存 |
| `return_speed_mm_s` / `return_acceleration_mm_s2` | 20 / 100 | 仅历史私有回位 helper，公开回位入口已移除 |
| `tension_search_max_mm` | 30 | 历史 auto-tension helper 参数；不控制新工具 Z− 方向搜索 |
| `native_drag_tool_xy_flip` | true | 未覆盖；拖拽期间使用绕工具 Z 轴 180° 的自定义力参考系 |
| `native_drag_mass` | `[8,8,8,0.5,0.5,0.1]` | 同默认；交给控制器内部拖拽环 |
| `native_drag_damping` | `[120,120,120,5,5,1]` | 同默认 |
| `native_drag_stiffness` | 六个 0 | 同默认 |
| `native_drag_threshold` | 六个 5 | 平移轴 5 N，旋转轴 5 N·m；不是软件去基线后的合力阈值 |
| `native_drag_max_force_n` | 50 | 50 N，厂商原生拖拽参数，不能理解为牵引业务硬超力阈值 |
| `native_drag_max_joint_speed_deg_s` | 50 | 50 °/s，关节速度参数，不是 50 mm/s |
| `native_drag_start_confirm_timeout_s` | 2.0 | 启动后 ON 状态确认窗口；正式 launch 同为 2 秒。每次 RPC 仍受命令连接超时约束 |

大多数直驱参数在构造时缓存；`ros2 param set` 显示成功不代表已改变内部缓存。修改正式配置后应在无运行任务且现场准备好的情况下重新构建/启动，并确认参数。不要通过调大超时掩盖反馈断流。

### 3.3 当前校准与已移除的回零

当前只有软件松绳基线参与日常操作。manager 启动后在静止、反馈健康条件下采至少 0.5 s / 20 个新力样本并自动校准一次；结束、切模式、故障恢复或同进程反馈重连不重复校准。顶栏“重新校准”只在停止且无控制权交接时显式替换基线。程序整体重启会重新采集，因此启动时绳带应松弛。

该操作不调用 SDK `FT_SetZero`。driver 收到 `/traction/slack_calibration` 只记录事件；真实力减基线在 manager。手动带载重校准会扣除当时张力，不能作为掩盖实际受力的排障方法。

页面按钮、HTTP/bridge 和 ROS 公开服务中的回零、设置零点、返回牵引前位姿全部移除。代码中的 `_load_zero_pose`、`_save_zero_pose`、旧 return/auto-tension 私有 helper 或既有 `zero_pose.json` 仅为历史遗留，不再提供正式运动入口；构造时可能仍读取旧文件供历史成员保留，但新动作不会使用其中位姿作为目标。新部署不用配置旧固定零位；旧材料中的“结束后返回”“校准同时设置回零点”均已失效。

## 4. 反馈和下发的实际行为

### 4.1 反馈线程与话题

SDK 构造 `Robot.RPC(ip)` 会建立 `20003` 命令代理，并启动独立线程连接 `20004`；项目反馈线程以约 100 Hz 读取 SDK 的一个最新 `robot_state_pkg` 快照。关节位置、TCP、关节速度、六维力来自同一包，避免分别调用 `GetActualJointPosDegree` / `GetActualTCPPose` 等阻塞 RPC。

| ROS 输出 | 内容和单位 |
| --- | --- |
| `/joint_states` | 关节名 `j1`…`j6`，位置 degree→rad、速度 degree/s→rad/s |
| `/controller_manager/wrench` | `WrenchStamped`，力 N、力矩 N·m，标记 `base_link`；数据源为 `ft_sensor_data`，不是 SDK 的 `ft_sensor_raw_data`，也尚未减管理器松弛基线 |
| `/controller_manager/ee_state` | `fairino_msgs/PoseTwist`：位置 mm→m，姿态 RPY degree→quaternion；该函数没有填写实际 twist，不能据其默认零 twist 断定机器人静止 |
| `/controller_manager/healthy` | reliable + transient_local Bool，反馈更新时发布，表示驱动的硬件故障锁存状态 |

包是否更新用 SDK **对象身份**比较，不是简单比较数值是否变化；静止机器人仍应收到新对象。只要对象更新就可发布新反馈，`frame_cnt` 用于模式/使能确认。若始终还是初始化的状态类型则报不可用，旧对象超过 0.5 s 报 stale。没有新包时不重新发布旧数据或健康确认。

`healthy=true` 的发布路径主要看本地 `_hardware_fault_latched`，**不是对每个新包中的 `EmergencyStop`、`main_code`、`rbtEnableState`、`ft_sensor_active` 的完整健康映射**。这些字段在运动准备/恢复时明确检查；查看绿灯/healthy 不能代替现场状态核对。直驱对 normal feedback 的有限数验证也不能替代管理器的数据保护。

### 4.2 目标牵引 / 卸力 / 方向搜索的速度路径

管理器通过兼容 `/controller_manager/switch_controller` 请求激活 `cartesian_velocity_controller`。直驱不是加载插件，而是检查互斥条件、切自动 `Mode(0)`、必要时 `RobotEnable(1)`、确认新实时状态，然后调用 `ServoMoveStart()`，清掉旧速度后接管。目标牵引、主动卸力和方向搜索都使用这条 Cartesian 通道；保留的牵引前快照不再提供公开返回运动。

`_send_motion` 读取 `/controller_manager/command_cart_vel_stamped`（TwistStamped）的最新线速度；旧Twist话题只保留诊断兼容，不再用于硬件控制：

- 根据源时间检查0.25 s过期及模式/servo接管前旧速度，队列只保留最新一包；非零速度继承manager命令源时间，避免controller把旧模式速度重新打成新时间。有限数检查覆盖Twist六个分量，但实际下发只用XYZ，角速度始终为0。
- 对线速度范数限幅，再乘固定cmdT及1000换成mm步长，累加到明确的绝对目标位姿。计划节拍与实际间隔双重门控：100 Hz下相邻包至少约9 ms；错过时隙直接丢弃，不压缩补发旧位移，也不把早到回调间隔当成更小的积分时间。
- 目标/搜索/卸力改用 `ServoCart(mode=0)` 绝对基坐标位姿，以 `base_servo_sign=+1` 映射速度步长；姿态固定，模式切换/新接管重取反馈起点。零速/接触/过期会取消旧目标领先量并锚定实时TCP；新目标领先反馈限幅5mm并丢弃超出推进量，短时超过不立即报错；连续限幅且1s进展不足0.5mm，或旧目标偏离反馈超过2倍上限（默认10mm）才故障停止，不能积累补跑。遗留host drag仍用mode2工具增量；原生拖拽不发host ServoCart。新绝对轨迹通路尚未实机验收。
- `cmdT` 等于配置的 motion 周期，正式为 0.01 s；低于 `1e-9` 的位姿数精确置 0，避免厂商 XML-RPC 解析极小科学计数失败。
- 每次ServoCart RPC耗时超过50 ms打印警告；成功回包超过正式0.25 s时不再锁故障或调用ServoMoveEnd，牵引会话和锁向保持。恢复时清旧速度/目标领先量并重新锚定最新TCP，拒绝恢复界限前的速度来源时间，收到新鲜命令后按0.08m/s²速度向量变化上限平滑继续，不补跑漏掉的位移。零速、DISABLED、过期和搜索接触停止仍立即生效；本次servo余程保持恢复限加速度，结束后新servo重新初始化。每2 s打印实际发包Hz、非零包Hz、最大RPC耗时。非零结果、RPC异常和真实反馈/硬件故障仍进入原故障收尾。

停用Cartesian controller时调用 `ServoMoveEnd()`。它失败时换新代理重试一次，总延迟仍可能达数秒；失败保留cleanup状态并拒绝新接管。正常停用失败及运动/反馈故障中的servo结束失败均尝试StopMotion和RobotEnable(0)，保留故障及急停锁存；故障收尾前先发布unhealthy并禁止新速度。不能用“设置速度零”替代结束servo会话。

符号至少有三层，改方向前先确定哪层有错：直驱基增量 `base_servo_sign`；算法目标牵引/卸力工具轴映射 `traction_sign_x/y/z=[+1,+1,-1]`；原生拖拽参考系 `native_drag_tool_xy_flip`。新方向搜索由 manager 把物理工具 Z− 转成 base 向量，使用 base ServoCart，不经过 traction_sign 工具轴反号。不要用旧 Y− auto-tension helper 或全局反号替代它。

### 4.3 原生省力拖拽

1. 拒绝已在 servo/return/search、cleanup 未结束、硬件锁存、反馈缺失或过期的启动。
2. 切手动 `Mode(1)` 并等待实时包确认，检查急停输入、传感器激活、控制器错误码和模式；**手动原生拖拽不要求普通 `rbtEnableState=1`，不主动调用 `RobotEnable(1)`**。现场三位使能开关可能让手动模式中该 bit 为 0。
3. 原始力只验证有限数；不会把工具自重/安装偏置加到 5 N 的厂商拖动阈值，也不会用 raw 合力大就拒绝启动。
4. `SetForceSensorDragAutoFlag(0)` 禁止错误清除后自动重开拖拽；按默认设置调用自定义 `FT_SetRCS(2,[0,0,0,0,0,180])` 修正现场工具 X/Y 反号。
5. `EndForceDragControl(1,...)` 启动控制器内部回路，兼容旧 9 参数和新 11 参数 SDK，通过签名判断；附加的适配/干涉等开关设 0。在调用前保守占用 native drag 控制权，即便 ON 回复丢失也必须关闭并确认 OFF。启动成功回复后，按 monotonic 截止时间、最多间隔 0.1 s 查询 `GetForceAndTorqueDragState`，默认允许 2 秒确认 ON；不会重复发送 ON。等待期间也检查实时反馈新鲜度、急停/本地故障锁存、控制器错误、模式和传感器，不要求普通运动使能 bit。OFF 确认保留约 0.4 秒窗口，确认失败后仍停止并去使能；单次 RPC 耗时可额外占用其连接超时时间。
6. 拖拽期间 SDK 力数据从该自定义参考系旋转回 base 发布；零平移参考不增加力臂矩。停止时先确认 drag OFF，再 `FT_SetRCS(1, zeros)` 恢复 base。

自定义参考系配置失败时拒绝拖拽并尝试恢复 base。当前随 SDK 的 `FT_SetRCS` 注释只列 ref=0/1，没有记录 ref=2，因此 ref=2 支持情况和物理方向必须以控制器固件/现场确认，离线测试只能证实调用路径。

若停止/确认 OFF/恢复 base 失败：锁存硬件急停及故障，执行 `StopMotion` + `RobotEnable(0)`，发布 unhealthy。新驱动启动时也查询遗留 drag；若旧进程死后仍 ON，会先尝试关闭，关不掉则去使能并拒绝完成启动。这说明**仅启动驱动也会写硬件配置和停止遗留拖拽**，并非只读连接。

2026-10-27 已修复 ON 执行但回复丢失时漏发 OFF 的异常路径，并补充 Fake SDK 故障注入验证。这里的 `_native_drag_active` 在启动 RPC 发送前就置 true，表示“可能已启用、必须清理”，不能把该本地 flag 当作控制器确认。只有实际 ON 标志与硬件就绪条件均通过，启动服务才返回成功。失败信息在 OFF/参考系恢复前保存最后的两种 drag 状态、main/sub_code、模式、使能、传感器、急停和反馈年龄，随 manager 的 stop_reason 导出。

会话 `session_1793088080813` 的 stop_reason 是 `FR5 did not confirm native force drag activation`，全部 250 帧只有 MANUAL_SETUP/FAULT，没有 DRAGGING；代码路径说明 ON RPC 返回了 0，但旧的四次查询没有确认 ON。原始导出没有保存 drag 标志或控制箱错误码，因此不能仅凭该日志断言是固件拒绝还是异步切换超过旧窗口，也没有证据指向关节限位。修复解决了可复现的等待窗口误报与异常清理缺口；硬件实际启动仍需现场验证。本次未加入任何关节提前限位/退回算法。

调手感从 launch 中 native mass/damping/threshold 开始；当前 `traction_params.yaml` 中 `drag_gain`、host drag 滤波/加速度等主要服务遗留主机拖拽路径，不控制原生 EndForceDragControl 的内部手感。

### 4.4 一次性工具 Z− 方向搜索

目标模式待机时，力至少 3 N 的方向每 2 s 稳健更新；点击开始后全程锁定，到位保持期间也不解锁。若尚无有效方向，点击开始触发一次实际运动搜索，不能把该按钮理解为单纯的无运动查询。

manager 使用当前工具四元数得到真实工具 Z− 在 base 中的向量，并发布 `DIRECTION_SEARCH=7` 与正的 `linear_speed_mps`。controller 跳过牵引工具轴符号补偿；driver 普通 `base_servo_sign=+1` 保持物理含义。按用户确认规划0.30m/5s五次曲线，理论峰速112.5mm/s，manager与driver独立限速120mm/s。

达到去基线未低通合力3 N时立即停止搜索。至少等待0.15 s、确认末端/关节静止并持续0.20 s保持≥3 N，用至少5个新力样本稳健取得方向，然后进入与普通开始相同的目标牵引路径。若停稳0.35s后尖峰已消失且受力<3N，从静止继续同一次剩余搜索；原5s/30cm预算不重置。持续受力但1s内方向不稳则回待机提示。实际起点位移达到0.30m或monotonic时间到5s仍无方向，停止回READY，不丢基线、不故障、不自动重试。driver另独立限制0.30m/5s，DISABLED接触暂停不重置它，只有新servo接管才授权新预算；一次搜索到限后必须由新操作重新授权，不能靠同一未结束搜索命令反复重置起点。反馈中断取消本次搜索。

这条新搜索不使用旧 `auto_tension_tool_y_minus/tool_z_minus/base_z_minus` 的力增量投影、2 mm/s 或 100 mm 参数；旧私有 helper 的参数与诊断不能用来解释新搜索。

### 4.5 主动卸力与保位

卸力接口 `/traction/unload` 可在待机、目标牵引、目标保持或原生拖拽中使用。优先采用已记录的牵引方向，没有牵引记录时采用最新有效方向；没有方向时拒绝猜测。原生拖拽交接必须确认 OFF 并恢复 base 力参考系，再申请 Cartesian，不能同时占有两种控制权。

控制算法沿目标增力方向的反向移动，卸力请求上限20 mm/s，当前还受core普通速度20 mm/s约束，接近5 N减速；去基线未低通合力 `<5 N` 即刻停止并保持当前位置。已有低负载不应继续原目标加力任务。普通结束、卸力完成和到位保持均不回零、不转回历史姿态。

目标控制向设定值中心逼近，但到位独立判定实际力在目标±2.5N内连续1.5s后停止保位；不要求controller已SETTLING，出带或反馈过期重新计时。默认最大可调目标 100 N，可由设置提高；软件不再按实际受力 100 N / 150 N 或其他幅值门槛报故障。控制柜自身的保护和原生拖拽厂商参数（如 `native_drag_max_force_n=50`）仍各自生效，不能把后者误认为工作站牵引目标上限或软件阈值。

## 5. 急停、故障和恢复

这里的硬件急停服务是通过网络发 SDK 控制命令；实体急停输入属于控制器安全链路，不依赖网页、ROS 或网络。现场操作应知道实体急停在哪里。不要把网页按钮的请求已返回等同于物理状态已经核验。

`/traction/hardware_emergency_stop`：

1. 先本地锁急停、清速度/牵引命令/return/search，阻止后续接管。
2. 依次 `StopMotion()` → `RobotEnable(0)` → `Mode(1)`；每个命令失败会记结果并继续尝试后续步骤。
3. 然后关闭 native drag；这样较慢的 drag RPC 不会排在停止/去使能之前。
4. 普通 `_servo_enabled` 清 false，但保留 `_servo_cleanup_pending`，因为去使能不一定关闭控制器内的 ServoMoveStart 会话。
5. 返回 success 的主要条件是 `RobotEnable(0)` 返回码 0；其他失败可以附在 message 中。它不是对去使能位已更新的完整实时确认。

`/traction/hardware_emergency_recover` **会修改硬件并可能恢复使能**，不是“读一下错误”：

1. 确认拖拽已停/参考系已恢复；servo/return/search 正在活动时拒绝。
2. `ResetAllError()` → `Mode(0)` → `RobotEnable(1)`。
3. 若旧 servo cleanup 未完成，调用有界重试 `ServoMoveEnd()`；失败则去使能、回手动，维持锁存。
4. 若实时状态提示传感器未激活，`FT_Activate(1)`；失败也去使能、回手动。
5. **强制再发 `Mode(1)`** 并等待新实时包确认，检查急停释放、错误码为零、传感器激活；手动最终状态不强求普通 enable bit=1。
6. 验证成功才清硬件故障/急停锁存、恢复健康。上层还需要复位回 READY；已有软件基线继续有效，不必重新松绳校准，也不会自动继续旧任务。

通信或反馈 fault 会尝试停 drag/servo，停止 return/search，锁 `_hardware_fault_latched` 并发布 false；不是反馈一恢复就自动消锁。若原生 drag 停不下来另行去使能。反馈线程与 motion timer 是并行的，共用同一个 SDK 实例和可替换 XML-RPC 命令代理；修改故障/重试逻辑时要审查这一并发关系，现有离线测试未覆盖所有真实并发传输交错。

## 6. 可复制的诊断顺序和操作级别

以下命令是供后续操作时参考，本次接管没有执行网络探测或硬件命令。先只读确认，再在现场准备好后进入改变控制器状态的步骤；不要为排错启动临时第二个 `Robot.RPC()`。

### A. 本机只读检查，不连接机器人

从项目目录执行：

```bash
cat /etc/os-release
uname -m
ip -br address
ip route
ip route get 192.168.58.2
ss -tnp
pgrep -af 'fr5_direct_driver|traction_manager|traction_controller|fr_robot_driver|fr_force|state_service|ros2_control_node|ros2_cmd_server'
test -r /opt/ros/humble/setup.bash
test -r ros2_overlay/install/setup.bash
test -r vendor/fairino-python-sdk/linux/fairino/Robot.py
git -C vendor/fairino-python-sdk rev-parse HEAD
ls -lt ~/.local/state/fr5-traction/logs/
```

查网卡掩码/路由是否让目标走机器人有线口；`ip route get` 是本地路由查询。`ss -tnp` 中分别看 `:20003` 与 `:20004`，也看网页本地 `:8081`。查看日志时按具体时间选择文件，重点搜索 `FT_SetRCS`、`stale`、`RPC took`、`ServoMoveEnd`、`Native force drag`、`RobotEnable`。旧零点文件不参与新工作流，缺失不应成为启动或操作故障。

### B. 无运动意图的主动网络探测 / ROS 只读查询

端口测试会建立短暂 TCP 连接；可能占用机器人连接资源，运行中的单所有者系统优先看已有 socket 和日志，不要反复探测反馈端口。

```bash
ping -c 2 -W 1 192.168.58.2
timeout 2 bash -c '</dev/tcp/192.168.58.2/20003'
```

如需判定 20004 是否被防火墙阻断，先查看运行驱动的 `ss` 与 stale 日志；确需单独 TCP 探测时，应在确认没有运行驱动、现场无动作后进行。ping 通不证明 20003/20004 通，20003 通也不证明 20004 已有兼容实时包。

已有 ROS 栈的终端可执行：

```bash
source /opt/ros/humble/setup.bash
source ros2_overlay/install/setup.bash
ros2 pkg prefix fr_traction
ros2 node list
ros2 topic info -v /controller_manager/wrench
ros2 topic info -v /controller_manager/ee_state
ros2 topic info -v /joint_states
ros2 topic echo --once /controller_manager/healthy
ros2 topic echo --once /traction/status
timeout 8 ros2 topic hz /controller_manager/wrench
ros2 param dump /fr5_direct_driver
bash scripts/preflight_check.sh
```

ROS 查询需要 DDS 发现/通信，可能启动 ROS CLI daemon，但不调用硬件动作。`topic hz` 会订阅消息，显示的频率依赖网络、QoS和 CLI 开销。`preflight_check.sh` 只检查部分话题/服务是否存在，**不检验数据新鲜、唯一发布者、错误码、机器人模式、物理松绳或搜索空间可用性**。另一个 `check_traction_runtime.py` 仍硬编码旧 `/force_torque_sensor_broadcaster/wrench`，当前正式链路无该话题，应先修正它再用作门禁。

### C. 会改变运行环境 / 控制器状态的命令

| 操作 | 为什么不是只读 |
| --- | --- |
| `bash scripts/tablet_setup_env.sh` | apt/pip/rosdep、下载/可能重建依赖、编译、生成桌面图标 |
| `scripts/start_ros_stack.sh 2` | 停旧控制栈、创建 SDK 连接、FT_SetRCS、可能关闭遗留 drag |
| `scripts/tablet_start.sh` / 桌面图标 | 先探测 RPC，随后启动上述控制栈和网页/Firefox；二次启动会替换已有 supervisor |
| 关闭专用 Firefox / 页面退出系统 | supervisor 终止网页/ROS进程组，驱动 shutdown 尝试停 servo/drag；并非只关显示窗口 |
| `ros2 param set`、改网络 | 可能改变后续动作或断控制器通信；不作为默认排错起点 |
| prepare、reset/recover、switch_controller、native_drag、unload、start | 写基线、复位/切模式/使能或直接运动；不能为了验证服务存在而调用。回零/设零/返回服务已移除 |
| `kill` / 重启、直接 SDK 例程 | 改变控制权；进程崩溃可能遗留 native drag / servo 会话，必须有现场处置条件 |

## 7. 典型故障从哪里查

| 现象 | 先看什么 | 进入代码的位置 / 注意点 |
| --- | --- | --- |
| 页面打不开 | 本地 `8081` 是否监听、网页日志、端口环境变量 | `run_workstation.sh`、`app.py`；不先重启机器人 |
| 桌面持续等待机械臂连接 | 网线/控制箱、`ip -br a`、`ip route get`、已有 socket | `tablet_start.sh` 在网页打开后后台测试并等待；Wi-Fi互联网正常不能证明有线机器人网络正常 |
| SDK constructor/RPC 失败 | 是否第二个连接者、20003路径、SDK版本、控制器是否开机 | `_load_robot_sdk`、SDK `RPC.__init__`；不要再开临时 SDK 验证 |
| 20003通但一直初始化/反馈 stale | 20004 socket、SDK接收线程、包结构/固件、反馈频率 | `_read_realtime_state`、`_feedback_loop`、vendor `robot_state_routine_thread` |
| WRENCH/EE/health超时 | 先区分真实包断流、命令RPC慢、ROS消息发布/订阅问题和错工作区 | 实时包停止 0.5s直驱会锁存；算法通常 0.25s先停输出，管理器另有较长 1.2s故障窗口 |
| `FT_SetRCS` 启动卡约10s后有 warning | 控制器已有参考系、固件对重复配置响应、是否旧 drag 遗留 | startup 可允许继续；该警告不代表坐标已重新验证。不要直接把超时开关关闭后盲目运动 |
| 力显示大、松绳也有静态载荷 | 物理松绳与姿态、raw/corrected两条话题、上次prepare基线 | 不能以重校零掩盖真实张力；原生drag的厂商内部力与manager软件tare不是同一路 |
| `force sensor is not active` | 控制器力传感器配置/电源/连接、急停后的状态 | readiness检查；正常启动没有自动 FT_Activate，恢复路径才按需激活。不能用修改布尔状态替代修复硬件 |
| drag 显示ON但拖不动 | 是否旧ServoMoveStart未清、手动模式、实际刹车/三位使能条件 | `_servo_cleanup_pending`、恢复 `_end_servo`；不能强制自动/使能来“修”原生drag |
| 拖拽X/Y反向、Z正确 | native自定义参考系是否启用/支持，TCP姿态/安装是否换了 | `_set_drag_force_reference`、`_custom_drag_wrench_to_base`；需现场短距离XYZ验证 |
| 目标牵引/卸力工具Z方向错误 | `traction_sign_z=-1` 是否生效、base_servo_sign是否 +1、运行工作区是否最新 | 算法工具轴校正与直驱分层；不是改native参数 |
| 模式切换/接管被拒绝 | return/search/drag是否仍active，cleanup/fault/emergency锁存、实时状态 | `_on_switch`、`_ensure_enabled_for_motion`；SDK返回0还须新包确认 |
| ServoMoveEnd反复超时 | RPC路径/控制器响应、是否一条命令占用服务线程 | `_TimeoutTransport`、`_end_servo`；约1.5s×重试仍可影响UI等待，不能无限重试或清flag装作成功 |
| 急停恢复后仍故障 | 实体急停输入、错误码、FT激活、Mode(1)实时确认、旧servo清理 | `_on_hardware_emergency_recover`；成功后还需上层复位/准备流程 |
| 新界面还有回零按钮或旧服务 | 网页缓存、app/bridge和ROS是否全部加载新版本 | 公开入口已移除；不是为新机器补旧零点文件 |
| 搜索到30cm/5秒仍无力 | 绳路、工具Z−、3N corrected force、真实位移 | manager `handle_pretension`、driver `_limit_direction_search`；结束后提示重试，不能无限自动累计 |
| 卸力不可用或不降力 | 是否有已记录/最新方向、真实合力、native OFF和Cartesian交接 | manager `handle_unload/handle_releasing`、core UNLOADING；不回旧姿态 |

不要将 `docs/stage_report.txt` 中某个“current_state”当作当下状态。排查实际当前状态应读本次日志和实时 ROS；仓库历史记录只有当时的证据。

## 8. 源码阅读和修改入口

| 目标 | 文件 / 函数 |
| --- | --- |
| 硬件连接与参数默认 | `ros2_overlay/src/fr_traction/scripts/fr5_direct_driver_node.py`：构造函数、`_load_robot_sdk`、`_reset_command_proxy` |
| 正式参数与唯一所有者启动 | `ros2_overlay/src/fr_traction/launch/traction_system.launch.py` |
| 环境/桌面部署 | `scripts/tablet_setup_env.sh`、`start_ros_stack.sh`、`tablet_start.sh`、`preflight_check.sh` |
| 数据源/坐标/单位 | `_read_realtime_state`、`_publish_feedback`、`_custom_drag_wrench_to_base` |
| 运动周期、限幅、超时、增量符号 | `_tick`、`_send_motion`、`_servo_cart` |
| controller接管/释放 | `_on_switch`、`_select_motion_mode`、`_ensure_enabled_for_motion`、`_end_servo` |
| 原生拖拽 | `_on_native_drag_start/stop`、`_set_native_drag`、`_set_drag_force_reference`、`_stop_native_drag_best_effort` |
| 急停与恢复 | `_on_hardware_emergency_stop/recover`、`_disable_after_failed_native_stop`、`_robot_motion_readiness_error` |
| 一次性方向搜索 | manager `handle_pretension`、controller DIRECTION_SEARCH、driver `_limit_direction_search` |
| 卸力和交接 | manager `handle_unload/start_unload_motion/handle_releasing`、core UNLOADING、driver `_on_switch` |
| 历史回位/张紧私有代码 | `_load_zero_pose`、`_start_return_to_pose`、`_start_auto_tension`，不参与正式工作流 |
| 物理协议行为核对 | `vendor/fairino-python-sdk/linux/fairino/Robot.py`：`RPC`、`RobotStatePkg`、`ServoCart`、`EndForceDragControl`、`FT_*` |
| 离线硬件调用路径回归 | `ros2_overlay/src/fr_traction/test/test_native_drag_driver.py` |

vendor 的 `xmlrpc_timeout` 装饰器只是检查类变量 `RPC.is_conect`，**没有真正设置每个调用的 socket timeout**；真正的运行时 1.5s 和配置 10s 超时由项目 `_TimeoutTransport` 提供。SDK初始 `RPC()` 构造在项目替换代理前仍有自己的连接/线程行为。若接收线程因 EOF/建连失败退出，当前代码不保证自动可靠重建实时流；这会进入直驱 stale 故障，而非可无条件自动恢复。不要只根据函数名 `reconnect` 就认定有可用重连策略。

`reference/fairino_hardware_src` / include / test 是旧 C++ `ros2_control` 硬件接口与 command_server 的交接参考：导出关节/力接口、硬件激活、可选FT_SetZero、ServoJ接管/释放等。`fr_controllers` 是旧 Cartesian速度→关节命令插件。它们不是当前 Python直驱的运行前置，也不能与当前链路同时真机启动。`fairino5_v6_moveit2_config/launch/real_robot.launch.py` 会启动旧 `fr_force/fr_robot_driver`，并不是现在的 traction入口。

## 9. 历史证据与下一次现场验收边界

| 文档 | 可用于理解的事实 | 已被当前源码改变的内容 |
| --- | --- | --- |
| `docs/integration_stage_report.md`、`docs/stage_report.txt`（2026-09-01） | 旧逐字段RPC在ServoCart期间阻塞约1s；引入20004同包反馈；ToolY−实测确认base符号；历史5N/10N记录 | 25Hz、1…20N、30mm找张紧、初始校准重写回零、旧超时/行程保护等均不是当前值 |
| `docs/session_fault_analysis_20260901.md` | WSL/SDK约0.5s停顿、旧速度/积分穿过目标造成振荡、重复prepare会话展示问题 | 修复阈值与算法后又继续演进，不能直接照抄当当前参数 |
| `docs/direction_sign_test_20260902.md` | 某个位姿局部二维力响应有交叉耦合，5N及回位的历史记录 | 局部矩阵不能当所有位姿常量；当前主动方向跟随实现应另读算法文档 |
| 根目录 `stage_report.txt` | 旧host drag反向事故、命令超时与servo清理、工具Z符号、15N位置模式历史实测 | 后来原生drag取代host逐帧拖拽；位置到位带宽/确认时间已变；其“等现场验证”状态不能自动视为已完成 |
| `docs/FR5骨伤牵引机器人工作站项目说明书.md`、旧版 `ros2_overlay/README.md` | 早期系统背景、模型/旧硬件链路参考 | 有初始校准更新零点、1…20N、行程限制、梯形回位、fairino_hardware唯一所有者等旧叙述；现以当前源码和本交接为准，overlay README 已在此次接手中更新 |

2026-10-26 驱动 Fake SDK 21 项、2026-10-27 前一轮原生拖拽修复 30 项均为历史离线结果；不是本次完整新流程的最终测试数量。新搜索、卸力交接、移除公开回零以及模式/基线流程的准确验证数据统一见 MACHINE_AND_VALIDATION。本次不把假 SDK 或隔离 ROS 通过写成实机通过。

下一次现场验收应分别确认：控制器/SDK 与参考系；KWR75D 电源/总线和重量/重心配置；TCP/工具安装及真实工具 Z−；搜索空间、3 N 建方向及刹停；目标 ±2.5 N / 1.5 s 到位；运动中一键卸力、低于 5 N 停止与交接中结束；实体急停、恢复保持基线且不会自动续动；拖拽手感、反馈稳定性，以及每种模式/力值的独立负载验收。可输入大于 100 N 的目标不等于该力值已验收。

### 实体按钮与合力门控补充（2026-10-28）

界面所述浅蓝→绿对应的检测使用SDK `robot_state=4→1`，不是摄像头读灯；现场灯光映射仍需结合真实状态确认。driver发布退出帧力给manager，空闲静止时自动软件tare，用户已明确允许扣除当时全部张力。原生力拖拽/servo和尾帧不得触发该事件。新增门控按页面显示合力>5N确认0.1s后请求原生启用、≤5N请求停用，维持原生EndForceDragControl并确认ON/OFF，停止失败仍锁存并下使能。不得以软件OFF请求当作已无运动；不移除控制柜自身各轴阈值。

5N门控停用使用专用 `/traction/native_drag_gate_stop`，确认原生OFF但保留已验证的力参考系；下一次超过门槛启用不重复FT_SetRCS。正常结束/卸力仍通过原 `/traction/native_drag_stop` 恢复base参考系，避免门槛附近频繁重配坐标引入延迟或受力跳变。

2026-10-28桌面启动更新：网页先启动，机器人未准备好时后台等待/重试，20003端口可用后才创建SDK owner；一次失败尝试清理后才开始下一次。driver首次connected后不因运行故障自动重启控制栈，以保留故障锁存/任务/基线语义。
