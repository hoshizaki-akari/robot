# 旧版 AUBO 原生 TRACKING 拖拽

## 为什么换接口

普通 LineMove 方案已经在真实手推下达到约 49.810 mm，但操作者明确报告顿挫、
不轻便。每步等待静止后再发下一步的工作方式不能冒称连续拖拽。

本机 SDK 的 `robotServiceFollowModeJointMove` 是独立的 TRACKING move mode，
不是现代 ARCS 的 servoJoint。只在明确开启的省力拖拽中使用；位置／恒力仍走
普通直线步进，不进行整套控制器／固件升级。

## 代码边界

- `enable_follow_drag` 默认 false；开启后只在新鲜的 TractionCommand.DRAG 下选择
  跟随路径。停止后切换模式，位置／恒力仍选回普通 LineMove。
- 最新目标从新鲜真实位姿构造，150 ms 前瞻，最多每 100 ms 更新一次；等待
  上条 SDK ACK 后才可替换，仍只有一个发送任务，不在电脑侧积累轨迹队列。
- 跟随更新不再要求每一步静止，但松手／结束／故障仍撤销意图并经独立急停、
  有序最终停止、三帧新的实际静止反馈确认。数据过期仍停止。
- 固定姿态 IK；保留 5 mm 目标距离、20 mm/s 命令值、关节单步、50 mm 当前会话
  空间边界。关节 profile 从 IK 增量与目标距离计算，最大 0.2 rad/s、0.5 rad/s²，
  每步设置并回读。关节路径不等同于精确笛卡尔直线，方向和停止必须实测。
- 连续 ACK 但 2 s 没有测得位置推进会停止，不能靠新 ACK 无限延长无效运动。
- 拖拽 20 N 与位置／恒力 8 N 分离；未删除有限超力保护。未授权扩大到 20 cm。
- 会话空间边界到达后通过 `/traction/session_limit_stop` 正常结束，先确认实际
  静止再恢复 READY。服务不可用时仍停止并报告交接失败，不自行越界。

## 当前兼容性证据（不等于动态验收）

只执行过一次“当前静止关节位置作为目标”的独立兼容性调用：

- SDK FollowModeJointMove 返回 0，耗时约 0.030325 s。
- 两个停止返回均为 0。
- 严格目标误差检查未通过：最大关节差约 0.000337 rad，不能写成零位移通过。
- 之后只读检查收到三个独立静止帧，RobotState=Stopped。

见 `debug/session_20261019_hardware/follow_current_pose_probe.log` 与
`follow_probe_readonly_stop.log`。兼容性探针是显式硬件工具，不由网页或离线测试
自动运行；凭据仅取进程环境，不在源码和日志中。

跟随 API 的控制柜内部排队／重规划行为不能仅靠头文件或 ACK 推断，必须在实际
拖拽过程中核对位姿、调用间隔、方向、反向和松手停止。若失败，不自动重复运动。

## 验收 CHECKPOINT

最终离线回归：fr_traction 197 reported entries（0 errors/failures、22 skipped），
aubo_v4_driver 127 reported entries（0 errors/failures、14 skipped）；两项 scheduler /
transport CTest 各重复 20 次通过，Python／网页 53 项通过。统计包含 lint 聚合。
日志：`debug/session_20261019_hardware/follow_drag_reviewed_offline.log`。

独立复核发现空闲时间误入进度 watchdog 和范围停止交接未检查应答的问题，均已
修复并加入回归。主线程另修复了 ACK 入队和发送槽释放不原子的竞态，新增
100 次连续立即替换测试。复核者未再次检查修复后的快照；上述最终回归由主线程执行。

先通过离线 scheduler／transport／ROS 回归和独立复核，再部署可选后端。
由操作者从网页启动，真实推拉 X/Y/Z 后松手、结束，导出完整 ZIP。
在数据和操作者反馈都支持前，不宣称“流畅”“省力”或用例已经通过。

当前不等于已验收的生产级人机交互力控，位置／恒力带载验收也仍待完成。
