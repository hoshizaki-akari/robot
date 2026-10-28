# ROS 2 控制、算法与接口接管说明

当前行为更新：2026-10-27，分支 `feat/multi-mode-traction` 工作树。本文描述本次新源码，不能据此认定既有运行进程已加载新版本。新构建尚未替换原真机运行栈；未执行本次新流程的实机运动验收。最终构建与测试结果见 [MACHINE_AND_VALIDATION](MACHINE_AND_VALIDATION.md)。

## 1. 正式控制链和修改入口

```text
浏览器 → app.py → backend/ros_bridge.py → traction_manager_node
                                               ├─ 基线/流程/方向/到位/记录
                                               ├─ /traction/command
                                               ↓
                                      traction_controller_node
                                               ├─ 滤波/纯C++算法/坐标符号/限速
                                               ↓
    /controller_manager/command_cart_vel_stamped (TwistStamped)
                                               ↓
                                 fr5_direct_driver_node.py
                                               ↓
                                       FR5 / KWR75D
```

Python direct driver 是唯一 SDK 所有者。`/controller_manager/*` 是兼容命名空间，由 direct driver 提供，不意味着当前运行标准 ros2_control 硬件插件。原生省力拖拽由 `EndForceDragControl` 在控制柜内执行，不使用 C++ 主机侧拖拽速度；目标牵引、方向搜索和卸力通过 `ServoCart` 执行。

以下路径均相对 `ros2_overlay/src/fr_traction/`：

| 文件 | 当前职责 |
| --- | --- |
| `src/traction_manager_node.cpp` | 自动/手动基线、操作模式、待机方向、启动锁定、搜索/卸力、异步控制权交接、安全与记录 |
| `src/traction_controller_node.cpp` | 100 Hz ROS 适配、力滤波、姿态/符号变换、各模式独立限速、过期置零 |
| `src/traction_controller_core.cpp` | POSITIONING、UNLOADING、DIRECTION_SEARCH；保留的历史算法实例 |
| `src/traction_math.cpp` / `.hpp` | 向量、低通、稳健方向估计、目标牵引预测速度控制 |
| `src/traction_safety.cpp` | 有限值、新鲜度、健康与可选 UI 心跳；没有实际受力幅值故障门槛 |
| `src/traction_state_machine.cpp` | 合法状态边、运动状态分类；新增流程必须补状态与集成测试 |
| `scripts/fr5_direct_driver_node.py` | 唯一 SDK 连接、真实反馈、servo/native drag 接管、搜索独立约束、急停/恢复 |
| `config/traction_params.yaml`、`launch/traction_system.launch.py` | 分别提供管理器/控制器参数、driver 正式参数；以实际加载配置为准 |
| `msg/*.msg`、`srv/*.srv` | ROS 接口契约，修改后须重新生成并重启所有依赖进程 |

## 2. 三种公开操作模式

操作模式与控制命令模式是两组不同数字，不能混用。

| 操作模式 | 数值 | 行为 |
| --- | --- | --- |
| 目标牵引 `POSITION_TRACTION` | 1 | 原位置牵引算法，开始时锁方向，控制向设定值中心收敛，实际力在目标±2.5N连续1.5s后停止保持 |
| 省力拖拽 `ASSISTED_DRAG` | 2，默认 | FR5 控制柜原生辅助拖拽，无需目标力或锁定方向 |
| 卸力 `UNLOADING` | 3 | 优先沿上次牵引方向的减力方向运动，否则用最新有效方向；真实合力 <5 N 停止并保位 |

恒力 `CONSTANT_FORCE=0` 仅保留历史常量/算法，当前模式接口不接受。公开回零、设置零点、返回牵引前姿态及手动“方向确定”接口均已移除。普通结束始终停在当前位置，不执行返回运动。

## 3. 基线只建立一次

1. `INITIALIZING` 等待 controller、driver、正确坐标系的力、关节和末端反馈就绪，进入 `READY`。
2. 启动时自动采集静止反馈：持续至少 `auto_calibration_window_s=0.5 s`、至少 20 个新 wrench 序号样本，取原始力均值作为松绳软件基线。
3. `calibrated=true` 后，结束、切模式、故障/急停恢复或同一管理器进程内的反馈重连不会重复基线采集。重启整个 manager 是新程序启动，会重新采集一次，因此启动时按约定保持绳带松弛。
4. 顶栏“重新校准”调用 `/traction/prepare`，仅设备健康、反馈新鲜、机器人停止且无控制权交接时允许，显式替换基线并清除旧方向。运动中禁用。

`latest_wrench = raw_wrench - baseline`，只扣力三分量；力矩保持原值并检查有限性。`/traction/corrected_wrench` 是去基线但未低通的流。manager 和 controller 各自对该流做 5 Hz 低通，两者不是串联。软件基线不等于 SDK 硬件 `FT_SetZero`，也不是完整重力补偿模型；带载手动重新校准会将当时张力一并当成零偏。

## 4. 待机方向、开始锁定与一次性搜索

目标牵引模式待机时，采集总力至少 `auto_direction_min_force_n=3 N` 的方向样本，每 `auto_direction_update_s=2 s` 做一次稳健估计。算法归一化每个样本、剔除角度离群；张力幅值变化本身不代表方向变化。低力或本窗口没有可靠方向时保留最新有效方向，提示等待有效施力。

点击开始时锁定最新有效方向；若当前已张紧但尚未赶上下一次 2 秒更新，可立即从当前有效受力方向取得方向。目标牵引及到达后的 `POSITION_HOLD` 全程保持锁定；运行中不做旧版主动方向修正。点击结束后才恢复待机取向。

没有方向时，开始会触发一次搜索：

- 用末端四元数把物理工具 `Z−` 转成 base 向量，沿该固定向量运动，角速度为 0。
- `DIRECTION_SEARCH` 使用最小jerk五次曲线：`v=(D/T)×30u²(1−u)²`，用户确认D=0.30m、T=5s，理论峰速0.1125m/s；manager和driver独立上限0.12m/s/0.30m/5s。
- 未低通合力达到3N立即停止搜索。至少等待0.15s且关节/末端静止，再连续0.20s采集至少5个新力样本（均≥3N，稳健方向p95≤15°），锁向后走正常目标启动路径。
- 短暂尖峰在停稳0.35s后消失（受力<3N）时，可从静止继续同一次剩余预算；原始5s计时/30cm起点不重置。恢复速度另限0.08m/s²、0.25m/s³，避免从零突然跳到曲线中段。若持续≥3N但1s内方向不稳，则停止回待机，不盲目移动。
- 到5s或30cm仍无方向则停止提示，不丢基线、不自动开始新搜索；反馈中断同样取消本次任务。

搜索命令已经是真实工具 Z− 的 base 表达，controller 必须跳过 `traction_sign_x/y/z` 补偿。目标牵引和卸力则沿已校准的牵引符号路径，不能为了修正显示箭头改变搜索或实际运动符号。

## 5. 目标牵引、卸力与坐标

目标牵引用 `PositionTractionController` 预测速度环：估计力率和刚度，接近目标时提前制动。默认容差 `position_tolerance_n=2.5 N`，近场窗口 `position_near_window_n=5 N` 必须大于容差。当前正式目标速度恢复最大0.020m/s，加速0.080/减速0.160m/s²；jerk上限 `position_max_jerk_mps3=1.0`。内部中心容差 `position_center_tolerance_n=0.5`，近场连续向设定值逼近（静区仅为中心容差0.4倍即0.2N），不能将外层±2.5N当成低侧停止设点；误差处于±0.5N且残余速度/加速度安静才进入 `SETTLING`。进入力带静止后须持续出带0.20 s才重调；反向纠偏必须先静止并持续确认0.20 s，禁止滤波残余绕过换向确认。

manager的到位条件独立于controller内部中心控制：实测总张力处于目标±`position_reached_tolerance_n=2.5 N`，连续 `position_reached_confirm_s=1.5 s` 即到位，不要求controller已进入 `SETTLING`。确认期间controller继续向设定值中心逼近；出带或运动反馈过期重新计时，计时使用monotonic时钟。随后停止 Cartesian 会话并进入 `POSITION_HOLD`；外力变化不会自动重启。修改目标可重新申请控制器；应先确认上次停止已完成。到位保持仍属本次牵引，方向不解锁。

最大可调目标力默认 `target_force_max_n=100 N`，管理员可以在设置中提高到 100 N 以上。后端将设置与 manager ROS 动态参数原子同步；开始及运行中调目标均使用同一配置范围。已移除 `validated_target_max_n` 的额外 100 N 硬门槛。该上限只约束用户可选目标，不触发实际受力超限故障，也不代表该范围已实机验收。

卸力既可从待机启动，也可在目标牵引/到位保持/拖拽时一键请求。方向优先已记录的牵引方向，没有历史牵引方向才用最新有效方向。没有有效方向时不猜方向；张力已经低于 5 N 时不继续主动减力。原生拖拽必须确认 OFF、恢复参考系后再交接 Cartesian；交接期间结束/急停不得在异步回复后再启动卸力。

`UNLOADING` 沿 POSITIONING 增力方向的反向运动，当前受core普通速度上限约束为0.020m/s（卸力请求参数同为0.020m/s），接近5 N降速，禁止反向拉力。core 的速度律向 4.5 N 靠近，使严格 `<5 N` 判据能实际到达；manager 与 controller 对未低通合力 `<5 N` 立即停止，避免滤波滞后继续移动。停止后保留当前位置，不返回原位。

ROS 位置/角度/力使用 m、rad、N；SDK 使用 mm、deg，转换只在 driver 边界。目标/卸力算法输出 base 速度，经 base→tool、`traction_sign=(+1,+1,−1)`、tool→base，再交给 driver 的 `base_servo_sign=+1`。原生拖拽另用工具 Z 绕转 180° 的自定义力参考系。三处符号职责不同。最终空间命令看 `/traction/controller_velocity_vector`，标量诊断不保证等于校准后向量对原始方向的投影。

## 6. 状态和接口

| 状态 | 值 | 当前含义 |
| --- | --- | --- |
| INITIALIZING / READY | 0 / 1 | 等输入 / 自动校准或正常待机 |
| MANUAL_SETUP / CALIBRATING | 2 / 4 | 保留的准备状态 / 历史方向采样状态，不再要求手动方向确认 |
| PRETENSION | 3 | 当前一次性工具 Z− 方向搜索 |
| DIRECTION_LOCKED | 5 | 本次目标方向已锁定，等待 Cartesian 就绪 |
| TRACTION | 6 | 目标牵引 |
| RELEASING | 7 | 当前主动卸力，不是旧返回位姿 |
| COMPLETED | 8 | 记录终态，随后回 READY |
| FAULT / EMERGENCY_STOP | 9 / 10 | 故障 / 急停锁存 |
| DRAGGING / POSITION_HOLD | 11 / 12 | 原生拖拽 / 目标到位保持 |

控制命令为 `DISABLED=0, PRETENSION=1, TRACTION=2, RELEASING=3, DRAG=4, POSITIONING=5, UNLOADING=6, DIRECTION_SEARCH=7`。其中旧 `TractionCommand.RELEASING=3` 仍代表历史横向控制算法，不能与 manager `TractionState.RELEASING=7` 混淆。新消息增加 `linear_speed_mps`；状态增加 `calibrated/direction_available/direction_locked`。

| ROS 接口 | 当前用途 |
| --- | --- |
| `/traction/prepare` | 停止状态下显式重新校准，不是每次启动前置步骤 |
| `/traction/start`、`/traction/stop` | 启动当前模式 / 结束并保位；success 可能仅表示异步请求已接受 |
| `/traction/unload` | 一键卸力及必要的 native→Cartesian 交接 |
| `/traction/set_operation_mode` | 接受 1/2/3，拒绝历史模式 0 |
| `/traction/set_target_force` | 目标牵引目标，含运行中更新与到位后重新牵引 |
| `/traction/emergency_stop`、`/traction/reset_fault` | manager 软件锁存 / 复位，原基线保持有效 |
| `/traction/reset_transient_fault` | 仅FAULT中的三种软件通信超时可自动复位，回调内重查白名单和反馈，不处理急停 |
| `/traction/auto_recovery_check` | driver只读检查实时健康、静止、无保护/锁存及控制权清理完成，不写SDK |
| `/traction/hardware_emergency_stop`、`/traction/hardware_emergency_recover` | driver 硬件停止、清理和恢复；恢复不自动继续旧任务 |
| `/controller_manager/switch_controller` | driver 兼容 Cartesian 接管/释放 |
| `/traction/native_drag_start`、`/traction/native_drag_stop` | driver 原生拖拽 |
| `/controller_manager/wrench`、`ee_state`、`healthy`、`/joint_states` | driver 原始反馈和健康 |
| `/traction/corrected_wrench`、`command`、`status`、`history` | 去基线反馈、模式命令、状态与会话历史 |
| `/traction/controller_velocity_vector`、`position_control_diagnostics` | 最终速度向量、到位相位/预测/刚度诊断 |

`/traction/calibrate_direction`、`set_zero_pose`、`return_zero_pose`、`return_pretraction_pose` 不再是公开操作接口。排查只查询 topic/service 类型和现状，不调用动作服务探测连通性。

## 7. 安全、暂停和恢复

2026-10-28页面“进入”流程先结束当前动作；bridge等待真实ready和关节静止后选择模式，不自动开始。manager的ready及模式切换限制涵盖原生拖拽OFF待确认、Cartesian停止/接管、实体示教及退出置零，避免异步停止回复前更换控制权。网页急停按钮废弃，FAULT/ESTOP的顶栏“恢复设备”仍需人工点击。后台只对WRENCH_TIMEOUT、EE_STATE_TIMEOUT、UI_HEARTBEAT_TIMEOUT做有限自动复位：连接静止连续1秒、driver被动检查通过后调用专用reset_transient_fault，再由manager当前状态/白名单/新鲜数据原子复核；最多3次、间隔2秒。不清硬件锁存、不使能、不置零、不自动续动。真实控制故障和停止失败继续人工处理。

工作站不再因实际受力超过 100 N、150 N 或其他软件力阈值报故障。`SafetyMonitor` 仍检查有限值、负的异常 force metrics、数据新鲜度、controller/driver 健康和可选 UI 心跳。历史 `SafetyLimits` 字段和 OVERFORCE/HARD_OVERFORCE 枚举可用于读取旧记录，当前 monitor 不生成这些受力幅值故障。机械臂控制器自身的保护保持原样。

正式运动速度使用 `/controller_manager/command_cart_vel_stamped`（TwistStamped），旧Twist话题仅保留兼容诊断，不再驱动硬件。非零速度继承manager原始命令时间戳；driver拒绝过期或模式交接前来源的速度，队列只保留最新一包。ServoCart成功回包（code=0）即使超过0.25 s也不再结束牵引或退出servo：只记warning，保留目标/锁向/会话；丢弃原目标领先量与旧速度，按最新TCP重新锚定，等来源时间晚于恢复界限的新鲜命令，以0.08m/s²的速度向量变化上限平滑继续。本地不补跑延迟位移；搜索仍沿用原5s/30cm预算。RPC非零返回、socket异常、反馈断流和真实硬件故障仍走原故障收尾。目标/搜索/卸力把每个实际包的v×cmdT累加到绝对基坐标位姿，通过ServoCart(mode=0)下发；起点和姿态取接管时反馈，模式切换重新锚定，零速/过期/接触停止丢弃未完成领先量。新目标领先实时TCP限幅5mm并丢弃超出推进量，短时跟随滞后不直接故障；连续受限且1s内实际进展不足0.5mm则停止，旧目标偏离反馈超过2倍限幅（默认10mm）也立即故障，禁止无界追赶。此SDK路径尚未实机运动验收；按厂商文档mode0是绝对基坐标位姿，不代表可保证实际跟随。每个包不补发错过的位移，发送间隔至少约9 ms（100 Hz配置）；每2 s日志记录实际发包频率和RPC峰时。

正常速度命令/corrected wrench 超过 0.25 s 不新鲜，controller 输出零并 reset；manager 的短暂停止窗口也是 0.25 s，力/末端长超时 1.2 s 则故障并请求停止。搜索反馈中断取消本次搜索。原生拖拽运行在控制柜内，主机发 DISABLED 不能证明 native drag 已关闭；必须查询 OFF 并按失败路径处理。

网页急停先请求 driver 硬件停止，再锁存 manager；恢复先执行硬件恢复和旧会话清理，再清 manager 锁存。恢复需要健康、反馈新鲜、停止已确认，保留原基线，要求医生主动开始新任务。只调用 manager 软件急停本身不能证明控制箱已去使能。

## 8. 记录与验证

每次实际开始新动作建立独立 session CSV，结束/故障关闭记录；重复开始无需重新校准，也必须有新记录。目标牵引交接卸力时关闭前一模式记录并建立卸力记录。历史目录保持 `debug/traction_sessions`，应与 app 导出路径一致。方向跟随字段仍可存在于历史 CSV，不能据字段名推断新流程仍主动修正方向。

历史上一版三模式实现执行过离线测试，覆盖±2.5 N力带、卸力和旧30 cm/2 s搜索等。2026-10-27第9节前版按用户要求未测试；最新第10节在用户允许后完成少量针对性离线检查。旧2秒搜索积分/峰速预期不再适用，不能将历史结果当成本版验收。准确范围见MACHINE_AND_VALIDATION最新追加节。

纯 core 测试可在 `/tmp` 编译，不启动 SDK：

```bash
cd /home/zhj/projects/robot/fr5_traction_workstation
test_binary=$(mktemp /tmp/fr-traction-core-tests.XXXXXX)
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic \
  -I ros2_overlay/src/fr_traction/include \
  ros2_overlay/src/fr_traction/src/traction_math.cpp \
  ros2_overlay/src/fr_traction/src/direction_correction.cpp \
  ros2_overlay/src/fr_traction/src/traction_controller_core.cpp \
  ros2_overlay/src/fr_traction/src/traction_safety.cpp \
  ros2_overlay/src/fr_traction/src/traction_state_machine.cpp \
  ros2_overlay/src/fr_traction/test/test_traction_math.cpp \
  ros2_overlay/src/fr_traction/test/test_direction_correction.cpp \
  ros2_overlay/src/fr_traction/test/test_traction_controller_core.cpp \
  ros2_overlay/src/fr_traction/test/test_traction_safety.cpp \
  ros2_overlay/src/fr_traction/test/test_traction_state_machine.cpp \
  ros2_overlay/src/fr_traction/test/test_traction_synthetic_pipeline.cpp \
  -lgtest_main -lgtest -pthread -o "$test_binary" && "$test_binary"
```

## 9. 历史算法和后续修改注意点

`continuous_force_controller_`、`OneDimensionalAdmittance`、`DirectionEstimator`、`AdaptiveDirectionFollower` 和旧 host drag 等仍有纯算法代码及回归测试，供历史分析与以后研究。正式三模式工作流不再开放恒力和运行中方向跟随；`direction_correction_mode=active` 等遗留参数不代表新目标牵引会转向。旧私有回位、auto-tension helper 也不是当前方向搜索入口。

改算法从 math/core 开始；改流程从 manager 和状态边开始；改实际坐标从 controller、driver 和 launch 的三层符号契约开始。更新消息后重建整个依赖链并重启 ROS 与网页，使生成接口一致。普通 `use_sim_time=true` 不能隔离机器人；所有离线集成必须显式使用假反馈和假硬件服务，不能启动正式硬件 launch。

## 实体拖拽退出与合力门控（2026-10-28）

独占SDK的driver从新实时帧识别 `robot_state=4`（拖动示教）到 `1`（停止），以WrenchStamped发布 `/controller_manager/teach_drag_exit_wrench`，保存退出时刻的base系受力。不把普通auto/manual模式位变化当实体按钮；本机原生拖拽/servo期间及其尾帧被排除，启动时首次停止也不生成退出事件。manager仅在工作站空闲或软件拖拽等待施力、反馈健康、关节静止且无接管中应用一次基线，清旧候选/记录方向；按用户确认允许当时有张力，界面将现有受力计入零点。目标牵引或卸力中不改变基线，先保留退出事件、待任务正常结束后处理；故障期间丢弃事件。此操作不调用FT_SetZero、不改持久回零位姿。

省力拖拽的“开始”先建立DRAGGING会话，在界面同一合力（保留0.1N显示精度）>5N连续0.1s后才请求原生ON；显示≤5N或反馈短时不新鲜即请求OFF，没有下降沿延迟。ON/OFF沿用driver确认和失败清理，期间不发送host ServoCart。OFF后仍在DRAGGING等待再次>5N，不结束会话、不丢校准；点击结束/卸力交接通过请求代次取消晚到的启用回复。启用确认、通信与原生保护会影响实际执行延迟，软件请求OFF不能代替控制柜OFF确认。厂商各轴5N阈值仍存在，因此合力门控是附加准入条件，不是对原生速度律的替换。

实体拖拽活动状态另通过可靠锁存Bool话题 `/controller_manager/physical_teach_drag_active` 通知manager：蓝色拖动期间拒绝软件接管，绿色退出后仍等待带载力快照应用完成才允许软件拖拽重新激活，避免两个ROS话题到达乱序造成提前启用。driver也以同一实时状态拒绝实体拖动期间的原生启动请求，作为回调延迟的后备互锁。

5N门控停用使用专用 `/traction/native_drag_gate_stop`，确认原生OFF但保留已验证的力参考系；下一次超过门槛启用不重复FT_SetRCS。正常结束/卸力仍通过原 `/traction/native_drag_stop` 恢复base参考系，避免门槛附近频繁重配坐标引入延迟或受力跳变。
