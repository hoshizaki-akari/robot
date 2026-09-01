# FR5 牵引工作站

这是用户仓库中的独立工作站项目。它使用 `090105.html` 的原有界面风格，
接入真实 FR5、KWR75D 和 ROS2 牵引管理器。

网页进程只通过 `FastAPI + rclpy` 订阅 ROS2 状态和调用牵引服务，不直接连接
Fairino SDK。正式运行时唯一的 SDK 连接者是 `fr5_direct_driver_node.py`；它以
25Hz 读取真实 FR5/KWR75D，并独占发送 ServoCart 指令，避免原 ros2_control
路径在当前 FR5 SDK 上约 1Hz 的阻塞问题。

## 当前流程

1. 启动 ROS2 牵引系统，等待 `READY`。
2. 确认绳带松动，点击“校零”。如果已经在 `MANUAL_SETUP`，也可以再次点击这个
   按钮，它会重新采集当前松绳基线并开启一条新的记录。
3. 需要重复回到同一松弛姿态时，可在 `READY`、`MANUAL_SETUP` 或 `COMPLETED`
   状态点击“设置零点”；牵引控制未接管时“回零”也可用。
4. 使用法奥官方示教器手动张紧，只做 XYZ 平移并保持法兰方向不变；也可在
   已确认的现场条件下使用受限的工具 Y− 自动张紧搜索。
5. 停稳后在页面确认方向并锁定。
6. 方向锁定后设置目标力（1～20N），再点击“开始牵引”并记录；未成功设置目标
   时系统拒绝开始。
7. 外界沿锁定轴施加扰动时，系统只沿该轴补偿，不产生横向或旋转运动。
8. 点击“结束牵引”，等待张力小于 1N；必要时点击“回零”。

当前目标力输入和启动范围为1～20N，但20N尚未完成独立实体验收。25N软超力和
30N硬超力仅作为异常超力保护，不是合法目标力范围；这两个保护阈值会在后续安全
评审中单独处理。

## 启动

```bash
cd /home/zhj/projects/fr5_platform_ws/fr5_traction_workstation
source /home/zhj/projects/fr5_platform_ws/.venv/bin/activate
python -m pip install -r requirements.txt
bash run_workstation.sh
```

浏览器打开 `http://127.0.0.1:8081/`。端口可以用
`WORKSTATION_PORT=8082 bash run_workstation.sh` 修改；独立工作站默认不占用旧
`platform_b` 网关正在使用的 8080 端口。

## ROS运行

```bash
./scripts/start_ros_stack.sh
```

启动脚本默认使用现有 FR5 基础工作区；如果现场工作区位置不同，先设置
`FR5_ROS_WS=/实际路径`。ROS 启动后可在另一个终端执行
`./scripts/preflight_check.sh`，再执行 `./run_workstation.sh` 启动网页。

不要同时启动旧的 `state_service` 真机SDK读取、`fr_force`、`fr_robot_driver`、
旧 ConstantForce 或任何第二个Fairino SDK连接者。

## 数据和验证

ROS牵引会话由后端写入其 `data_directory`；网页通过 `/api/traction/history`
读取真实历史，不能用浏览器模拟力或本地缓存冒充正式记录。

```bash
bash scripts/run_checks.sh
```

`ros2_overlay/` 保存了本工作站对应的 ROS2 应用源码和硬件交接参考源码；它不
包含机器专属的 SDK 连接地址、生成目录或第二份 SDK 连接程序。

实时原始力和末端状态分别从 `/controller_manager/wrench`、
`/controller_manager/ee_state` 读取；管理器在松绳准备时减去安装/重力基线，发布
`/traction/corrected_wrench` 给控制器，保证显示、控制和记录使用同一套去零点数据。
网页不读取模拟数据。

真实机械臂测试必须按阶段报告和实体急停要求进行，只允许在固定模型上测试。
