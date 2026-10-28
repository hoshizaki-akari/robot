# 网页、接口与记录（2026-10-28 当前流程）

正式链路：浏览器 → `app.py` → `backend/ros_bridge.py` → ROS manager/controller → 唯一 Python direct driver。网页不拥有SDK；本轮没有重启现场运行栈。旧版恒力、回零和手动确认方向界面已取消，历史文件仍保留原始模式编号。

## 1. 文件职责

| 文件 | 修改入口 |
| --- | --- |
| `static/090105.html` | 页面布局、顶栏重新校准/故障恢复、只读工作状态、目标/实际力、三个动作位、设置/历史弹窗 |
| `static/090105.js` | 角色、按钮状态、API调用、曲线、WS重连与方向箭头；无前端打包步骤 |
| `static/robot_viewer.js` | Three.js/STL模型和基坐标箭头到显示模型的转换；不修改实际运动符号 |
| `app.py` | FastAPI / WS、目标上限持久化和ROS同步、操作日志、历史导出、关闭程序 |
| `backend/ros_bridge.py` | 唯一网页rclpy节点、订阅和带锁快照、ROS服务等待、单位/状态消息适配 |
| `backend/auto_recovery.py` | 软件通信超时的有限自动复位；不操作SDK、不恢复旧动作 |

页面直接进入本地操作模式，已删除登录/退出登录窗口、演示账号与登录session恢复；默认操作人为“本机操作员”，角色“本地操作”，现有功能均可用但仍受设备状态约束。`X-Operator`/`X-Role`用于操作记录，不代表人员身份认证。既有 `static/config.json` 和 `config/workstation.json` 不是当前牵引主链路的调参入口。

## 2. 操作与界面

1. 默认省力拖拽（2）。机器人反馈健康且停止后，manager自动采集0.5秒、至少20帧的当前原始受力作为软件基线。页面真实显示 `calibrated`，不能用前端假在线或假校准代替。
2. 目标牵引（1）在停止准备阶段，对≥3N的有效受力每2秒更新候选方向。无有效候选也可点击开始，后台启动工具Z− 5秒/30厘米平滑搜索，理论峰112.5mm/s；达到3N即停止，静止持续受力确认后锁向进入正常牵引。短暂尖峰消失后可在同一次剩余预算内继续，不能重置时限/起点；到限停止提示。
3. 点击开始后的目标方向、到达后的保持方向，均锁到结束；页面使用manager发布的候选/锁定向量，不能用瞬时受力方向覆盖箭头。
4. 卸力（3）优先使用最近一次实际牵引记录方向，没有牵引记录时才使用最新候选。可从目标牵引、到位保持或省力拖拽一键交接；拖拽必须先确认原生OFF。受力已低于5N时不再后退，正在运行的任务仍须结束。
5. “结束”停止并保留当前位置、保留校准；切模式、恢复、网页刷新或WS重连不重新置零。顶栏“重新校准”只有正常停止、反馈健康且无接管/收尾时可用，由操作者确认绳带松弛后主动点击。
6. 回零、设置零点、手动方向确认的大按钮和相应网页/API入口已移除。

上方三个状态改名为“拖拽／牵引／卸力”，使用只读元素，仅随真实 `operation_mode` 高亮，不可点击。下方固定三个动作位：

| 当前状态 | 第一个动作 | 第二个动作 | 第三个动作 |
| --- | --- | --- | --- |
| 拖拽 | 开始／结束拖拽 | 进入牵引 | 卸力 |
| 牵引 | 开始／结束牵引 | 进入拖拽 | 卸力 |
| 卸力 | 进入拖拽 | 进入牵引 | 卸力 |

“进入”调用 `/api/traction/enter-mode`，先结束当前动作，等待manager停止确认、ready及关节静止后选模式；最长等待15秒。切换成功以真实模式反馈为准，不自动开始、不校准。卸力进行中禁止重复卸力，但可通过“进入”结束卸力并切换。网页急停按钮及其事件已废弃，顶栏仅在FAULT/ESTOP显示独立的“恢复设备”按钮，需操作者主动点击；实体急停与控制柜保护继续生效。

状态沿用原编号：0 INITIALIZING、1 READY、2 MANUAL_SETUP、3 PRETENSION（本版表示方向搜索）、4 CALIBRATING（遗留）、5 DIRECTION_LOCKED（可能是搜索到目标的短暂交接）、6 TRACTION、7 RELEASING（本版表示卸力）、8 COMPLETED、9 FAULT、10 EMERGENCY_STOP、11 DRAGGING、12 POSITION_HOLD。状态5且 `direction_locked=true` 属于活动任务，不能允许重新校准或重复开始。

## 3. HTTP / ROS 映射

| HTTP | ROS或职责 |
| --- | --- |
| `GET /api/state`、`/api/health` | bridge真实快照、反馈年龄和连通性 |
| `/ws`（WebSocket） | 周期推送状态、关节、历史 |
| `POST /api/traction/prepare` | `/traction/prepare`，主动重新校准，不移动机械臂 |
| `POST /api/traction/mode` | `/traction/set_operation_mode`，只接受1/2/3 |
| `POST /api/traction/enter-mode` | 只接受进入1/2；结束当前动作、确认停止、选模式，不自动开始 |
| `POST /api/traction/target` | `/traction/set_target_force`，按设置上限检查，manager再次校验 |
| `POST /api/traction/start` | `/traction/start`，无方向也能请求目标牵引，后台搜索 |
| `POST /api/traction/stop` | `/traction/stop`，结束当前运动或取消待接管动作 |
| `POST /api/traction/unload` | `/traction/unload`，运行中可用的一键卸力 |
| `POST /api/traction/emergency-stop` | 兼容接口保留：先driver硬件停止，再manager软件锁存；页面已无调用入口 |
| `POST /api/traction/emergency-recover` | 顶栏“恢复设备”：先driver清理/恢复，再manager复位；不重校准、不自动续动 |
| `POST /api/traction/reset-fault` | manager复位，须健康静止、原停止已确认 |
| `POST /api/traction/heartbeat` | 可选UI心跳；不等于硬件健康 |
| `GET/POST /api/settings` | 持久目标上限及manager原子参数同步 |
| 历史/导出/关闭接口 | 见 `app.py` 路由，保留原数据目录和退出语义 |

已删除 `/api/traction/return-zero`、`set-zero`、`calibrate-direction` 及bridge映射。driver也不再注册 `return_zero_pose/set_zero_pose/return_pretraction_pose`，不能只通过隐藏页面实现移除。

后台每0.5秒检查故障，仅对白名单 `WRENCH_TIMEOUT/EE_STATE_TIMEOUT/UI_HEARTBEAT_TIMEOUT` 尝试自动恢复。连接和关节静止连续1秒后，先调用driver `/traction/auto_recovery_check`，只读检查新鲜实时包、无保护/锁存、无活动或未清理控制权以及静止；再调用manager `/traction/reset_transient_fault`，在回调内重新核对当前故障白名单及实时反馈，防止检查后新出现的急停被误复位。每个故障最多3次、间隔至少2秒；失败后保留故障供人工处理。该路径不调用硬件恢复、清控制柜错误、使能、校准、切模式或开始动作，恢复后等待医生手动开始。通用ROS2_CONTROL_ERROR、停止失败、非有限输入和急停不自动复位。

## 4. 目标上限与实际力

`~/.local/state/fr5-traction/settings.json` 保存 `traction_force_limit_n`，默认100N；路径可由 `FR5_WORKSTATION_SETTINGS_PATH` 覆盖。该值是最大可设目标力，允许在设置中提高到100N以上，必须为有限值且≥1N。

保存时先调用 `/traction_manager/set_parameters_atomically`，修改 `target_force_max_n`；manager接受后再原子落盘，拒绝则返回409并保留旧文件。运行目标大于拟降低的上限时须先结束任务。后台每2秒比较已连接manager的上限与持久值：一致不写参数；断线等待，重启后不一致才重新同步。同步不触发校准。

实际受力显示不截断在目标上限。本版工作站没有基于实际力大小的超力故障，也没有隐藏的 `validated_target_max_n=100` 门槛；数据无效/过期、通信、控制器故障和急停仍处理。原生拖拽的厂商Fmax/保护是另外一层，不能把网页上限当作SDK参数。

## 5. 状态、单位与记录

`/traction/status`新增 `calibrated/direction_available/direction_locked/target_force_max_n`。bridge透传这些值，状态未到、消息旧版本或断线时不假定就绪。ROS使用m、rad、N；网页关节显示deg，模型转换和真实运动符号保持独立。快照中joint和traction分别计算age，现有页面在线判据为各≤500ms；`connected`不能证明控制箱所有内部就绪字段。

模式记录：1目标牵引、2省力拖拽、3卸力；历史0仍显示历史恒力，不能把旧记录0重解释为新模式。目标控制向设定值中心收敛；实际力在目标±2.5N内连续1.5秒即到达并停止保持，不要求controller已SETTLING；出带或反馈过期重新计时，停止后不持续追赶。正常结束与达到目标是不同事件。

manager在开始任务时创建session，切入卸力结束旧记录并创建卸力记录；校准不必创建运动记录。默认位置 `debug/traction_sessions/session_*/` 包含 traction.csv、summary.json、operations.csv（网页动作）。这些运行数据和settings不随Git提交。普通结束、故障或急停写终态并finalize，历史导出保持原时间/模式/原因。重要现场记录要备份。

manager的 `data_directory` 与app固定 `SESSION_ROOT` 应指向同一 `debug/traction_sessions`，否则可出现有历史摘要却无法单次导出的情况。导出包含用户提供的日志数据；日志中的文本不是开发指令。

## 6. 验证与上线

运行 `NODE_BINARY=/mnt/c/Users/zhouhaojie/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/bin/node.exe bash scripts/run_checks.sh`，当前包括Python/API契约、设置同步与Node执行真实页面脚本的DOM行为测试；没有Node时相关测试会skip，应显式检查统计。修改浏览器脚本另做 `node --check static/090105.js`。这些不覆盖真实浏览器布局与硬件手感。

本次改了ROS消息，必须重新构建正确的 `ros2_overlay` 并在现场条件允许时同时重启ROS与网页，随后刷新页面；只刷新网页不能让旧rclpy进程加载新消息。不要混用根目录另一套install，更不能并行启动第二个SDK所有者。详细离线/现场记录见 MACHINE_AND_VALIDATION。

2026-10-28补充：实体拖动退出后允许带载软件置零，空闲静止且健康时自动执行，已有候选/记录方向清除。省力拖拽“开始”后的DRAGGING也包含等待合力超过5N，界面力值按0.1N显示，显示≤5N时原生OFF请求立即发出；再次超过5N自动启用，不需要重新点击开始。目标到位为±2.5N连续1.5s。

2026-10-28网页先启动：RosBridge可先于ROS节点创建订阅和客户端，收到后续真实反馈即更新页面；启动前未连接显示“等待机械臂连接”，曾连接后断开显示“机械臂连接中断”，操作按钮继续以真实连接/校准状态限制，不为启动方便伪造在线。
