# 刮痧机器人算法与文献追踪

本目录用于长期保存刮痧机器人研究的每日/每轮深挖报告、算法线索与可复现资源。

## 当前主链路

Eye-in-Hand RGB-D 背部/软体曲面感知 → 人工标线三维轨迹 → scraper contact frame/TCP → RGB-D + 6D F/T 在线接触/法向/接触位置估计 → 曲面恒力/姿态同步柔顺控制 → 前刮恒力与轻力回程状态机 → 后续 residual/offline RL。

## 目录

- `daily/`：每轮深挖报告，Markdown，可全文检索。
- 后续逐步增加 `papers/`、`code_resources/`、`experiments/` 索引。

## 追踪原则

优先近五年高水平机器人、医疗机器人、控制与接触丰富操作论文；跨机器人超声、抛光/磨削/擦洗等同构任务检索。所有论文尽量核验题目、作者、年份、载体、DOI/官方链接；区分正式论文、预印本、项目与代码仓库。重点记录可迁移算法、真实机器人部署条件和待验证假设。

## 日报索引

- 2026-09-23：`daily/2026-09-23_run03.md` — 宽工具接触位置/工具形状估计、力矩法向修正、未知曲面恒力与低频导纳部署。
- 2026-09-24：`daily/2026-09-24_run04.md` — 从 point contact 推进到等效 CoP/面接触可观测性；MLS/DeepFit 局部法向；RegHEC 标定-配准；离散/冲击导纳稳定性与 FR5 实验设计。
- 2026-09-25：`daily/2026-09-25_run05.md` — 滑动接触 friction bias 与 force+velocity 在线法向估计；软组织刚度/Hunt–Crossley 在线辨识；刚-软异质接触 EKF+MPC；提出状态相关的视觉/力觉融合权重。
- 2026-09-25：`daily/2026-09-25_run06.md` — 新增人体背部直接同构 baseline：F/T 接触点/法向估计 + 动态接触减速 + 力误差补偿 + bounded variable impedance；补齐 KWR75D payload/CoG/bias ROS2 标定资源；进一步固定状态相关 visual-force local contact frame 与双向刮痧状态机。
