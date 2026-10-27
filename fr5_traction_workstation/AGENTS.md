# FR5 牵引工作站：新会话接手入口

本目录是 `hoshizaki-akari/robot` 仓库内的独立工作站，Git 根目录在上一级。
开始开发时先读 `docs/handover/README.md`，再按任务阅读 CONTROL、HARDWARE、RUNBOOK、WEB_AND_DATA；机器现状和验证记录在 MACHINE_AND_VALIDATION。它们是仓库内的项目知识，不依赖历史聊天。

## 工作前先确认

- 执行 `git status --short --branch`，保留已有用户修改。2026-10-26 接手时，`tests/test_emergency_recover.py`、`tests/test_set_zero.py` 已有未提交的上下文管理器格式调整，不属于此次接手修改。
- 当前开发分支为 `feat/multi-mode-traction`；接手基线提交为 `881117c007c1b77e722c9e5879975f7bf3977d4b`。以后以实际 Git 状态为准。
- 当前正式入口是 `scripts/start_ros_stack.sh 2` + `run_workstation.sh`；代码改动后确认运行进程加载的是本项目 `ros2_overlay/install`，不要误用项目根的另一套 `build/install`。
- 当前机器是 WSL2 Ubuntu 22.04 / ROS 2 Humble；不是部署文档中的实体 Ubuntu 一体机。IP、路由、零位和依赖都应重新核对，不能照抄旧机器状态。

## 关键架构约束

- 正式链路：浏览器 → `app.py` → `backend/ros_bridge.py` → ROS 2 manager/controller → `fr5_direct_driver_node.py` → FR5/KWR75D。
- Python direct driver 是当前唯一 Fairino SDK 所有者。目标/搜索/卸力使用有界绝对ServoCart轨迹（mode=0，领先实际TCP限幅5mm；连续受限且1s内进展不足0.5mm或原目标偏离反馈超过10mm才故障），不能重新引入无界追赶轨迹。网页不得直接连接 SDK，不要同时启动上级 `state_service` 真机源、旧 force demo、`fr_robot_driver` 或 legacy `fairino_hardware` 链路。
- 算法入口：`ros2_overlay/src/fr_traction/src/{traction_math,traction_controller_core,direction_correction}.cpp`；流程在 `traction_manager_node.cpp`；坐标转换和消息适配在 `traction_controller_node.cpp`。
- 当前模式为省力拖拽(2)、目标牵引(1)、卸力(3)，旧恒力(0)仅保留历史记录意义，不再可选。回零/设零公开功能已移除。普通结束停止并保留当前位置；原生省力拖拽由 FR5 `EndForceDragControl` 执行。
- 启动健康静止反馈后自动校准一次，结束/切模式/恢复/网页重连不重置基线；只有停止时顶栏手动校准才重新置零。目标模式受力≥3N每2s更新候选，开始后锁向至结束。无方向开始时按用户确认执行工具Z− 0.30m/5s平滑搜索（理论峰速0.1125m/s）；达到3N立即停止，经静止持续受力确认后转目标。瞬时尖峰消失可在原时限内从静止继续搜索，不能重置原始5s/30cm预算；到限停止，不自动重启新搜索。卸力优先使用最近牵引方向，受力<5N停止，不返回旧位姿。
- 目标控制继续向设定值中心收敛，controller内部position_center_tolerance_n=0.5；manager独立判定实际力在目标±2.5N内连续2s即停止保位，出带或反馈过期重新计时，不要求controller已SETTLING。页面启动直接进入本地操作模式，无登录/退出登录窗口，记录操作人为本机操作员。
- 本版取消工作站按实际受力大小触发的超力故障；可设目标上限默认100N，由设置经ROS原子参数同步，可提高。不要把已删除的validated/hard_overforce门槛重新接回主链路；控制器原生保护与通信/非有限输入/急停仍独立保留。
- 生效参数首先检查 `traction_params.yaml`、`traction_system.launch.py` 和 driver 的 `declare_parameter`。`static/config.json`、`config/workstation.json` 当前未被主链路加载。
- ROS 使用 m、rad、N；SDK 位姿使用 mm、deg。坐标补偿有三个不同入口，见 HARDWARE/WEB_AND_DATA，不能为了修正视觉箭头顺手改变真实运动符号。
- `use_sim_time=true` 不会隔离硬件；正式 launch 无 mock 模式。只读排查不要执行启动、回零、恢复、切换控制器等会修改硬件状态的操作。后续明确要求真机运行时，按当次任务授权和已确认的现场条件推进。

## 验证与维护

- 网页/API 修改运行 `bash scripts/run_checks.sh`；它不覆盖 C++ 和 driver pytest。
- 本机完整依赖与安装产物预检用 `bash scripts/check_local_env.sh`；此脚本不连接机器人，`LOCAL_ENV_READY` 不代表网络或硬件验收。现场准备状态见 `docs/handover/FIELD_READINESS.md`。
- driver 修改运行 RUNBOOK 中的假 SDK pytest；算法修改运行 CONTROL 中从当前源码构建的离线 gtest，或使用独立测试工作区进行 colcon 验证。
- 预检脚本只检查部分 ROS 名称，退出 0 不代表真机就绪；100 Hz 是设计频率，不代表本机实测。
- 2026-10-27 已补齐 `POSITION_HOLD` 故障/卸力转换；旧恒力的方向修正分支不再参与公开工作流，遗留算法及历史回归不代表新界面的工作流。
- 历史报告保留当时结果；配置值 `validated_target_max_n=100` 不等于当前硬件 100 N 已验收。
- 修改行为、参数、启动方式或硬件连接后，同步更新 handover 文档，并注明验证范围。不要自动提交或推送。
