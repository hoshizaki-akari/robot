# 持续拖拽响应：两份现场日志对应修复

后续实测 `session_1792394528900`：30 次 ACK，最大位移约 49.810 mm，峰值
17.912 N；操作者确认仍有顿挫、不够轻便，故本轮**未通过拖拽手感验收**。
最终因 50 mm 会话边界停止。已将该边界交接为管理器正常停止流程：停稳后
回到 READY，记录 SESSION_TRAVEL_BOUND，不再作为不可恢复设备故障。
针对顿挫的独立原生 TRACKING 路径见 `FOLLOW_DRAG_20261019.md`。

仅修改 AUBO 独立牵引工作站；原 FR5 工程不改。

## 实际证据

- `session_1792393784294_full_log.zip`：本轮记录内最大位移约 0.159 mm，随后
  HARD_OVERFORCE；停止前后的上下文峰值超过 12 N。该轮在明显跟随前就中断。
- `session_1792393789427_full_log.zip`：最大位移约 5.351 mm，并非完全没有 SDK
  驱动；实际完成若干约 1 mm 的步进，每步约 1 s。之后 JSON 和 CSV 同时出现
  约 4.9 s 的采集空档，再报 SDK 反馈过期。空档的宿主机／WSL 根因尚不能从这
  两份日志确定，不能将它直接解释成单个机械臂调用失败。

上版虽然把命令上限接通到 5 mm/s，但实际仍是 1 mm 单步、0.01 m/s² 加速度，
刚起步又只发 1.2 mm/s。只调增益无法使普通直线运动接口成为连续伺服。
现场 8 N 还来自早期联调 profile，原 FR5 配置为 150 N，二者不能混称。

## 本次代码与现场配置

- 拖拽：0.5 N 启动死区，0.010 m/s/N 增益，5–20 mm/s 非零速度；
  0.5 s 规划窗，单步最多 5 mm。低力启动对应约 2.5 mm，而非 0.6 mm。
- 本步速度大于 2 mm/s 时使用显式快速 profile，加速度最高 0.05 m/s²。
  请求不超过 2 mm/s 的位置／恒力调节仍用原 0.01 m/s²，且其规划步距不超过
  1 mm；每步 SDK 参数都回读，切回慢速模式也重新设置并验证。
- `drag_hard_overforce_n` 默认继承普通阈值；本机只对拖拽设为 20 N，位置／
  恒力仍为 8 N，确认时间仍为 0.1 s。保留有限超力停止，不恢复 FR5 150 N。
  仅在实际模式阈值变化时重设监视器，避免每 tick 清掉持续超力计时。
- 保留独立停止通道、实际到位／静止三帧、命令与反馈新鲜度、50 mm 会话包络、
  固定姿态和 IK 关节单步检查。不排队、不自动重发、不自动恢复故障运动。
- RobotState 查询间隔恢复为 100 ms，避免不必要的高频同步查询。
  新日志包含诊断／状态返回码、调用耗时、各输入年龄、历史诊断失败、最大主循环
  间隔。主循环长时间停顿会明确报 execution gap，而不伪装成正常运行。

这些是有界的普通 LineMove 分步控制，不承诺达到 FR5 连续流式伺服的手感。

## 自动验证

`bash scripts/check_stepwise_offline.sh` 完整通过：

- fr_traction：196 reported entries，0 errors / failures，22 skipped。
- aubo_v4_driver：116 reported entries，0 errors / failures，13 skipped。
- scheduler 与 transport 分别重复 20 次通过；Python／网页 53 项通过。
- 新增实际 ROS 参数链、XYZ 起步／释放、快慢 profile 切换、5 mm/1 mm 步距隔离、
  拖拽 12 N 不误停及持续 21 N 仍停止等回归。

统计包含测试／lint 聚合，不能称为同数量的真机测试。
测试 double 只在明确的离线测试里使用，不进入运行网页。

## 部署与验收

本机 supervisor 日志：`debug/session_20261019_152047_workstation/`。
只读 GetParameters 已核对现场阈值、拖拽增益、速度、步距和加速度；启动后
READY、传感器和机械臂在线、motion_idle=true、motion_ack_count=0。

本次按操作者要求，不用空载走一次 5 mm 替代拖拽验收。
验收用例是网页上实际推拉 X/Y/Z，检查持续跟随与松手停止，结束后保留完整 ZIP。
在获得该轮实际数据之前，状态仍是 WAITING_OPERATOR_LOADED_DRAG_CHECKPOINT。
不宣称本轮带载、位置牵引或恒力牵引已经通过。

证据：`debug/session_20261019_hardware/continuous_drag_offline.log`、
`continuous_drag_runtime.json`、`continuous_drag_parameter_check.txt`；现场标定文件
保存在忽略的 debug 目录，不提交其偏置到公共仓库。
