# AUBOPE V4 周期透传候选后端

本次范围：为 AUBO 省力拖拽增加独立的 TCP2CAN 执行通道，并修复测力／姿态
配对时最新测力覆盖等待样本的问题。原 FR5 工程不修改，位置与恒力牵引仍
使用已有执行路径。候选后端没有替换普通启动默认值。

## 与上一版的区别

上一版仍向 FollowModeJointMove 发送短位置目标；上层的 100 Hz 不能保证
控制柜以相同周期执行。候选后端使用本机 V4 SDK 已导出的 Enter/LeaveTcp2CanbusMode
和 SetRobotPosData2Canbus(vector) 接口。

规划采样间隔固定为 5 ms。SDK 调用按批发送，并非宣称 Windows/WSL 网络
每 5 ms 都准时完成一次 RPC。控制柜余量从 macTargetPosDataSize 读取；依照
官方旧驱动约定，六个关节数值对应一个轨迹点，非六的整数倍直接拒绝。
缓冲目标 10 点（按 5 ms 消费假设为 50 ms），剩余不超过 4 点时补足。
主发送线程在没有新调度目标时也会检查缓冲，保证松手减速能完成；不是依赖
网页每次再发一个点。静止尾段完成后不再无限填充静止点。

速度由轨迹点的时间和间距表达，不再依赖反复调用跟随运动的到位规划。
连续关节速度、共同缩放的加速度、IK 分支连续性及 FK 后的实际笛卡尔增量
都在发送前检查；不会把逆解跳支简单限速后继续发送。轨迹尾端不随噪声方向
旋转旧目标误差，也不追补 RPC 阻塞期间欠下的墙钟位移。

## 模式与停止边界

- 构造、默认启动和 READY 不进入透传；显式启用候选后，首次开始拖拽才尝试进入。
- 进入前要求实测静止和实测位姿/FK 一致；进入后要求控制柜缓冲为空。
- 任何不确定发送结果都不重发旧批次，避免重复轨迹；移动中欠载明确报故障。
- 有限短缓冲仍会带来已提交轨迹的延迟，不能把 50 ms 缓冲称为零延迟。
- 松手对关节速度连续减速，尾速归零后让缓冲排空。正式结束仍调用原停止通道。
- 停止成功并读到空缓冲后退出透传，再允许新的普通运动模式。
- 未能确认清空/退出时，不把“结束请求已发送”伪装成已恢复。
- 诊断/状态留在现有 SDK owner's 独立 read session；仅候选透传激活时不再让
  慢状态查询持有发送线程的普通 RPC 锁。默认 follow/line 行为不变。

新增诊断：drag_execution_transport、tcp_can_active、tcp_can_fault、tcp_can_batches、
tcp_can_points、tcp_can_queued_points、tcp_can_status、tcp_can_cycle_duration_s、
tcp_can_send_duration_s。缓冲数是补点前的实测快照，非预测余量。
tcp_can_active 在进入回复不确定时仍保留为真，以要求停止/退出确认；网页
优先显示 tcp_can_fault，不能把这个不确定状态标成已正常进入。
motion_ack_count
只计实际被 SDK 接受的批次，不计仅保留已有缓冲的循环；ACK 不代表物理到位。

## 测力与姿态配对

候选配置启用 queue_sensor_for_pose。短队列只保存真实输入，选择最新、仍在
原新鲜度预算内、可由真实姿态历史配对的测力样本。输出使用所选样本原始
时间戳和对应姿态，不重打时间戳、不外推姿态、不使用假数据、不自动带载归零。
匹配姿态稍后到达时，等待的测力样本不会已经被下一包原始数据覆盖。
重复／非法输入、时钟回退、过期数据仍按既有规则处理。

## 运行入口

普通已验收执行路径仍为：

```bash
cd ~/projects/aubo_i5_traction_ws/aubo_i5_traction_workstation
bash start_workstation.sh
```

候选真机验收入口：

```bash
cd ~/projects/aubo_i5_traction_ws/aubo_i5_traction_workstation
bash start_workstation.sh --tcp-can-drag
```

两者都会停止前一个工作站后重新启动，网页仍为 http://127.0.0.1:8082/。
启动本身不开始拖拽、不发送透传轨迹。候选模式需核对诊断
drag_execution_transport=tcp2can_experimental；普通模式应是 follow_mode。
回退使用普通启动命令，不应手动同时启动两个 SDK owner。

## 必须区分的验收结果

本轮只完成代码与离线测试，没有切换当前真机模式、发送新轨迹或证明手感改善。
5 ms 消费周期、缓冲计数单位、进入/退出模式和停止清空语义需要在这台具体
V4 控制柜上做 CHECKPOINT。若实测缓冲消耗/请求延迟不适用，必须据实修正
后端，不能用更长的轨迹队列掩盖欠载，那会加重松手/换向迟滞。

第一轮验收应先确认模式交接及结束后的真实停稳，再确认短距离正负方向、
松手排空，最后评价斜向/曲线手感。无需重新做未经变化的传感器安装标定。

离线测试使用明确命名的 SDK double 和代数接口 stub，只验证协议顺序、
缓冲和连续性规则，不是机械臂仿真，也不向网页提供测试数据。
完整结果和构建日志见 debug/session_20261021_tcp_can_candidate/。

依据：[官方旧版驱动](https://github.com/AuboRobot/aubo_robot/blob/Noetic/aubo_robot/aubo_driver/src/aubo_driver.cpp)
及本机 V4 头文件/动态库导出符号。不是以现代 ARCS SDK 的接口替代旧控制柜能力。
