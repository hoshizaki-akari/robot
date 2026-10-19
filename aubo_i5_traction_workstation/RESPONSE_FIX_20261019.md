# AUBO XYZ 拖拽：日志对应修复

本次只修改独立 AUBO 牵引工作站，不修改原 FR5 工程。

## 真实日志对应的问题

输入：`session_1792380700019_full_log.zip`。拖拽约在 5.20 s 出现运动命令，
约 5.32 s 才观察到位置变化；实际最大位移约 0.706 mm，随后持续施力触发
`HARD_OVERFORCE`。SDK 没有报告运动调用失败。三轴命令存在，方向与软件
变换后的力方向一致；这不是 XYZ 被屏蔽的问题，也不是已完成带载验收。

1. 分步控制没有使用非分步拖拽的启动力和增益，而是隐藏地使用另一套默认值。
2. 分步速度命令仅改变目标小步距离；SDK 直线速度一直使用连接时的固定设置。
3. RobotState 查询之间固定等待 100 ms，影响获取命令之后的新停止状态。

## 已修改

- 独立显式参数：拖拽启动死区 0.5 N、增益 0.003 m/s/N、命令上限
  0.005 m/s；三轴继续全覆盖。位置／恒力分步控制仍为 0.002 m/s，未一起加速。
- scheduler 将本步速度随绝对目标一起交给 transport；transport 在 IK 成功后
  设置本步 SDK 速度，回读速度和加速度参数，再次检查取消，再发送 LineMove。
  参数设置／回读失败不发运动，取消过程中不允许晚发旧指令。
- 常驻启动配置将 RobotState 查询等待改为 20 ms；这不是承诺 50 Hz 实测反馈，
  SDK 调用本身耗时和真实 waypoint 速率仍由硬件决定。
- 实际诊断导出本步请求速度、SDK profile 回读、waypoint 间隔和序号、运动 ACK
  计数及命令 ID。重复出现同一 last_sdk_result 不再需要猜测是否有新的 SDK 调用。
  profile 回读不是物理速度测量，ACK 也不是到位／停稳证据。

未删除 8 N / 0.1 s 超力停止、1 mm 单步、50 mm 会话包络、固定姿态、输入
新鲜度或实际停止确认；不通过关闭保护掩盖跟随不足。

## 验证

运行 `bash scripts/check_stepwise_offline.sh`：

- fr_traction：193 reported tests，0 errors / failures，22 skipped。
- aubo_v4_driver：114 reported tests，0 errors / failures，13 skipped。
- scheduler 和 transport 各重复 20 次通过；Python / 网页 52 项通过。
- 随后补充串口桥 SIGTERM 清理回归，Python / 网页合计 53 项通过。
- 新用例覆盖 ROS 参数到三轴输出贯通、松手输出零、力控速度不被拖拽参数改变、
  速度与端点裁剪分离、SDK profile 回读失败、更新 profile 期间取消、非法速度。

上述统计包含测试／lint 聚合，不是实际机器人运动次数。
独立复核代理因额度限制未执行；主线程已人工复核新增取消链路，并通过重复回归。

## 部署与现场零点

旧进程没有新位姿反馈，已停止并使用原常驻启动器重新连接；没有自动开始任务。
用户确认松弛后重新采集 1107 个真实样本，确认末端位置范围为零，更新本地固定
姿态零点。仅更新偏置和 tare 姿态，不重新假定传感器安装方向。
同时修复 WSL 串口桥收到 supervisor 的 SIGTERM 时未清理自有 PTY 链接的问题，
避免重启时误认为已有桥占用。只清理桥自己创建的链接，不覆盖其他设备。

调试证据位于 `debug/session_20261019_hardware/`：

- `review_session_1792380700019.txt`：原始日志诊断。
- `response_fix_offline_acceptance.log`：完整离线回归。
- `response_fix_unloaded.json`：本次松弛零点原始统计。
- `response_fix_runtime.json`：部署后真实状态证据。

## 网页验收 CHECKPOINT

打开 http://127.0.0.1:8082/ ，待数据显示在线，选择省力拖拽，松弛状态初始校准，
再开始拖拽并测试 X/Y/Z，松手检查停止，结束并导出本行完整 ZIP。
新响应参数是否足够跟手，需要本轮带载实测；位置／恒力也尚未通过带载验收。
不声称所有功能已经在真机带载跑通。
