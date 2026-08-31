# FR5 牵引工作站

这是用户仓库中的独立工作站项目。它使用 `090105.html` 的原有界面风格，
接入真实 FR5、KWR75D 和 ROS2 牵引管理器。

网页进程只通过 `FastAPI + rclpy` 订阅 ROS2 状态和调用牵引服务，不直接连接
Fairino SDK。正式运行时唯一的SDK连接者是 ROS2 `FairinoHardwareInterface`。

## 当前流程

1. 启动ROS2牵引系统，等待 `READY`。
2. 确认绳带松动并执行校零/基线。
3. 使用法奥官方示教器手动张紧。
4. 停稳后在页面确认方向并锁定。
5. 设置目标力，开始恒力牵引和记录。
6. 外界沿锁定轴施加扰动时，系统只沿该轴补偿，不产生横向或旋转运动。
7. 点击正常释放，等待张力小于1N。

当前目标力输入范围为1～30N；真实验收开放范围暂为1～15N。20～30N必须在
完成更高力级安全评估后再解锁。25N软超力和30N硬超力阈值不能同时用于合法
保持30N目标，这个边界会在后续安全评审中单独处理。

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

在当前 Humble 版本中，ros2_control 的内部控制器节点会继承
`/controller_manager` 节点名，因此 KWR75D 力广播和末端状态实际从
`/controller_manager/wrench`、`/controller_manager/ee_state` 读取；启动文件已
固定按此真实现场行为配置，网页不读取模拟数据。

真实机械臂测试必须按阶段报告和实体急停要求进行，只允许在固定模型上测试。
