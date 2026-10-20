# 2026-10-20 工作站修复记录

仅修改 AUBO 并行项目，未修改原 FR5 功能源码。

## 已定位与修复

- 连接失败：电脑有线网口只有 FR5 网段地址；用户添加 192.168.1.10/24 后，Windows/WSL 均能访问 192.168.1.40。保留原 192.168.58.10。
- 不完整启动：原命令只管理网页；统一完整启动器接管 COM4 桥、SDK、ROS 和网页。凭据由 Windows 当前用户 DPAPI 文件加载，不进入 Git。
- 拖拽费力：XYZ 拖拽死区 0.15 N、低力增益 0.080 m/s/N、最高 50 mm/s；位置/恒力速度最高 20 mm/s。参数是指令侧数值，不等于负载下已达到该速度。
- 过早停机：移除本机标定覆盖中的早期 20 N 拖拽停止、5 N 目标力等；应用原 FR5 配置的目标力 100 N、超力 150 N 持续 0.2 s。关闭额外会话球形位移边界。保留 IK、关节/反馈校验及控制柜约束，不宣称各型号底层约束完全相同。
- 诊断失真：旧导出曾携带长时间不更新的驱动状态。新增源时间与接收时间双重有效性检查，不再将旧诊断当实时数据。
- WSL 诊断传输：抓包可见较大 UDP 报文经过镜像 loopback0；单改 QoS 后仍收不到诊断。将 DDS 最大报文设为 1200 B、分片设为 1000 B 后恢复。40 次连续采样全部连接/READY/诊断有效，最大源年龄 107 ms，运动确认次数始终 0。
- 后续故障取证：记录健康失效具体分项、SDK 查询失败与最后原因；导出 ZIP 按会话时间筛选各次启动的 robot.log，加入 driver_events.log。历史 ROS2_CONTROL_ERROR 不能从旧日志唯一确定 SDK 内部原因，不能据空载 READY 声称已彻底修复。
- 本次 Z− 日志：7 秒内 22 个 FollowModeJointMove 调用返回 SDK 0；力峰值约 5.21 N，低于 10 N 目标，未触发过力阈值。下一步 IK 返回“stepwise IK failed or exceeded joint-step bound”，接着报告 command acknowledgement failed。实际腕部姿态仍在变化，因此不是仅凭位姿冻结造成的误判。该 ZIP 未保留 IK 返回码或超出关节连续性阈值的幅度，暂时无法判定是控制柜 IK 无解还是 0.05 rad 连续性检查拒绝。新驱动已分别记录 SDK IK 返回码、最大关节解差值/关节编号，并在停止 ACK 之后保留真正的运动失败原因；本轮没有依据放宽关节连续性检查。
- 重启配置：用用户实际的 WSL 启动路径执行只读 `--check-config`，返回凭据文件存在、DPAPI 凭据可读、标定 YAML 在 Ubuntu-22.04-F 可见。启动器现有显式配置检查入口，缺配置时错误会给出实际配置文件路径。

## 自动验证

- fr_traction：208 tests，0 failures，24 skipped。
- aubo_v4_driver：130 tests，0 failures，14 skipped；新增 SDK IK 错误码与关节连续性拒绝分类测试。调度器/传输压力重复各 20 次通过。
- 网页 Python 测试 58 项、编译检查、ruff 通过；包含镜像网络 MTU 配置回归测试。
- 通过只读 ROS 参数服务核对实际运行值：拖拽 0.15/0.080/0.050，位置/恒力速度 0.020；管理器目标力 100、超力 150、拖拽超力 150、持续时间 0.20，与配置一致。
- 完整停止/重启、`--check-config`、凭据解密和只读真机反馈已验证；启动不会自动开始牵引。最新进程再次 READY，25 次采样的诊断源龄最高 97 ms，运动确认数为 0。

## 运行与当前 CHECKPOINT

```bash
cd /home/zhj/projects/aubo_i5_traction_ws/aubo_i5_traction_workstation
bash run_workstation.sh --restart-live
```

网页：http://127.0.0.1:8082/ 。本轮日志：`debug/session_20261020_140620_workstation/`。

当前为网页负载验收 CHECKPOINT：用户校准后测试 XYZ 省力拖拽、位置牵引、恒力牵引并导出完整 ZIP。未以空载只读结果替代三种功能的实际验收；本轮未自动执行机械臂运动。

只读检查本机启动配置：`bash run_workstation.sh --check-config`。
