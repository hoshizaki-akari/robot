# AUBO i5 牵引工作站：低速短步后端离线验收

## 边界

本次只修改独立 AUBO 工作站，不修改原 FR5 项目。即使机械臂已开机，本次也没有连接控制柜、COM4 或发送真机运动指令。

这是软件离线验收，不是真机功能验收，也不是医疗使用验收。测试替身只存在于自动化测试目标，不用于网页运行状态或生产驱动。

## 实现

- 新增 `aubo_v4_stepwise_node`：普通 V4 `LineMove` 小步，固定姿态、单步在途、真实反馈确认后再规划，不进入 TCP2CAN，不堆积轨迹。
- 省力拖拽：力驱动有界速度，松手归零，无虚拟惯性积分。
- 位置牵引：达到目标力后等待实测静止，再保持当前位置；不是输入任意 XYZ 目标位置。
- 恒力牵引：根据实测力误差执行短步修正；允许低频响应，不承诺实时力控效果。
- 设置零位／返回零位：零位只保存在本进程内，新增状态 13，重启后须重新设置；不加载 FR5 旧零位。
- 停止、超时、源时间戳与坐标系校验、过期 ACK 丢弃、启动取消、运动确认和网页状态映射。

默认单步上限 0.5 mm，速度上限 2 mm/s，会话行程上限 50 mm。最小命令速度用于避开到位容差吞掉微小目标的情况；实际最小可重复步长仍需真机测定。

## 复核中修复的实质问题

1. 低目标力容差过宽，卸载状态可能误判为到位。
2. 控制策略计算出的微小步长低于调度器到位分辨率，造成无声停滞。
3. stepwise 输出重复应用 legacy 工具轴符号。
4. 回零的 correction-OFF 几何速度被控制器忽略。
5. 活动回零每周期先发 DISABLED 再发 RELEASING，相互抵消。
6. pending 普通牵引／拖拽启动不能被 stop 取消，迟到 ACK 可重新激活。
7. 条件变量退出标志在锁外修改，重复测试出现偶发退出挂起；改为同锁更新并通知。
8. 回零到位等待 idle 的分支未覆盖超时；补齐持续姿态、距离和输入有效性检查。

独立只读复核参与了取消启动和线程退出两项修复。软件测试没有证明控制柜固件的运动队列清空语义。

## 自动验证

- C++ 新增核心：调度 37、传输 18、牵引策略 15、零位策略 3 项。
- 实际 localhost ROS 子进程：驱动离线与 SDK 构造拦截 2、控制器 13、管理器 11 项。
- 原有 Python／网页接口测试 20 项。
- 核心调度／传输 CTest 各重复 20 轮；传输另作 100 轮随机顺序重复。
- 调度和传输 AddressSanitizer／UndefinedBehaviorSanitizer 检查。
- ROS 包完整构建、原有单元测试及静态检查；JavaScript 语法检查。

ROS 测试使用独立域 172/174/175 和 localhost。驱动测试通过 `LD_PRELOAD` 在构造真实 SDK 对象前截断；负对照证明拦截生效。正常离线驱动不创建 SDK、不发布虚构关节或末端位置，健康及静止状态保持 false。

## 如何自己复跑

在 WSL `Ubuntu-22.04-F` 中：

```bash
cd /home/zhj/projects/aubo_i5_traction_ws/aubo_i5_traction_workstation
bash scripts/check_stepwise_offline.sh
```

脚本只构建并测试，不启动硬件工作站。SDK 和旧 protobuf 在外部目录，不提交仓库；换电脑时可指定 `AUBO_V4_SDK_ROOT`、`AUBO_V4_PROTOBUF_DIR`。

离线查看新 ROS 入口（所有硬件与校准门默认关闭）：

```bash
source /opt/ros/humble/setup.bash
source ros2_overlay/install/setup.bash
ros2 launch fr_traction aubo_i5_stepwise.launch.py --show-args
```

新短步入口是 `aubo_i5_stepwise.launch.py`，不是旧 `aubo_i5_traction.launch.py`。禁止两种后端同时启动。现有 `start_ros_stack.sh` 仍保持只读入口，不会自动改成真机运动。

## 验证记录位置

- `ros2_overlay/build/fr_traction/test_results/fr_traction/`
- `ros2_overlay/build/aubo_v4_driver/test_results/aubo_v4_driver/`
- `ros2_overlay/build/aubo_v4_driver/stepwise_transport_repeat100.log`
- `debug/session_20261016_stepwise_offline/stage_report.txt`

## 下一 CHECKPOINT：真机，尚未执行

校准门继续关闭。下一阶段需确认力传感器安装变换／正负号、真实工具方向、短步实际位移和静止反馈，并实测停止后旧命令不会继续执行。SDK 返回成功不等于物理到位，vendor SDK 阻塞调用也不能由当前线程通知强制中断。

没有这些实测证据，不能声称三种牵引已在真机全部跑通。本次不要求用户重新接线或操作机械臂。
