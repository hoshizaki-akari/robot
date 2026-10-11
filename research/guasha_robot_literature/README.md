# 刮痧机器人算法与文献追踪

本仓库的目标是构建可验证、可复现、可在真实硬件上安全运行的完整系统；文献服务工程决策，台架实验验证跨任务迁移。未通过前置 Gate，不推进人体接触测试。

## 当前主链路
Eye-in-Hand RGB-D 曲面感知 → 人工标线 2D→3D 曲面轨迹 → 手眼/TCP/刮板接触框架 → RGB-D + 外置六维 wrench 融合 → 动态接触点/法向估计 → 曲面恒力/姿态同步柔顺控制 → 前刮/转向/轻力回程 FSM → 独立 safety supervisor。Residual/offline RL 仅在安全 baseline 验证后评估。

## 当前 Gate 状态（Run 14）
- **G0 硬件/API/时序/停止链：阻塞。** 已审查官方 SDK、ROS2 仓库、KWR75D 规格与相关用户 issue；现场版本、命令到运动延迟、状态数据年龄、低载荷校准和实体停止链尚未实测。
- **G1 坐标链/重力与 bias：未验收。** 需要已知力臂工装、独立参考载荷和坐标/符号验证。
- **G2–G4 几何、接触估计与控制：暂停推进。** 不把未验证输入送进闭环。
- **G5–G7 状态机、核心创新与学习策略：不推进。** 人体试验须另行完成风险与伦理/审批条件。

## 路线与索引
- [project_roadmap.md](project_roadmap.md)：系统模块、G0–G7、baseline、接口约定、实验与决策日志。
- [all_round/index.md](all_round/index.md)：零基础教学总览。
- [all_round/index.html](all_round/index.html)：HTML 教学索引。
- [all_round/glossary.md](all_round/glossary.md)：术语表。
- [daily/2026-10-11_run14.md](daily/2026-10-11_run14.md)：本轮详细报告。
- [all_round/round14.md](all_round/round14.md)：本轮七节教学摘要。

## 目录与工作流
- daily/：逐轮详细报告。
- all_round/：七节零基础教学版与索引。
- workstreams/interface/：FR5/ServoCart、KWR75D、状态反馈、时间戳和停止链审计。
- workstreams/geometry/：RGB-D、手眼标定、人工标线到 3D 轨迹。
- workstreams/contact-control/：CoP、动态接触框架、力位混合和采样系统稳定性。
- workstreams/baselines/：baseline 矩阵、创新候选和可证伪条件。

## 历史轮次与自适应方向
- Run 3：宽工具接触位置/工具形状估计与接触控制。
- Run 4：CoP/面接触可观测性、局部法向、标定与离散导纳。
- Run 5：滑动摩擦偏差、软组织刚度与力觉/视觉融合。
- Run 6：人体背部同构 baseline、动态接触减速和有界柔顺。
- Run 7：真实曲面 scraping 全链路与四层系统架构。
- Run 8：人工画线→3D、曲面参数化与误差预算。
- Run 9：人体按摩直接 baseline、曲面约束与安全层。
- Run 10：动态机器人超声、运动环相位滞后与软体形变。
- Run 11：动态接触框架与 FR5 频响/时延实验设计。
- Run 12/13：相关报告位于开放 PR #3，当前标记 **pending**，尚未进入 main；请勿当作已归档事实。
- Run 14：FR5 伺服生命周期、KWR75D 低载荷尺度、统一时序与停止链；G0 仍未通过。

## 追踪原则
优先近五年同行评审论文、官方 SDK/手册、作者仓库、数据集与真实机器人实验。跨机器人超声、抛光/磨削/擦洗、surface following、contact-rich manipulation、visual-force/tactile 与摩擦/滑移等任务必须逐项评估物理接触模式、传感通道、工具几何、控制接口与人体安全要求。区分正式论文、预印本、项目页、第三方实现与用户 issue；不得把其他平台的频率/误差或安全阈值直接移植到 FR5 刮痧。