# 刮痧机器人文献调研 · 零基础教学总览

> 本索引区分已归档内容与待审分支。main 当前已归档到 Run 11；Run 12/13 在开放 PR #3 中 pending；Run 14 在独立分支/PR 中待审。待审报告不能被当作 main 已归档事实。

## 一、先看系统全貌

主链路：RGB-D 曲面感知 → 人工标线 2D→3D → 手眼/TCP/刮板接触框架 → 视觉几何与六维 wrench 融合 → 动态接触点/法向估计 → 曲面恒力与姿态同步控制 → 前刮/转向/轻力回程 FSM → 独立安全监督。Residual/offline RL 放到安全 baseline 之后。

## 二、学习路线与历史轮次

| 轮次 | 教学模块 | 这一轮解决什么问题 | 当前状态 |
|---|---|---|---|
| Run 2 | [round02.md](round02.md) | 系统开篇：跨领域任务如何映射到刮痧机器人 | 历史文件 |
| Run 3 | [round03.md](round03.md) | 宽工具接触位置/工具形状估计 | 历史文件 |
| Run 4 | [round04.md](round04.md) | 面接触可观测性、局部法向、标定与离散导纳 | 历史文件 |
| Run 5 | [round05.md](round05.md) | 滑动摩擦、软组织刚度与视觉/力觉融合 | 历史文件 |
| Run 6 | [round06.md](round06.md) | 人体背部同构 baseline 与有界柔顺 | 历史文件 |
| Run 7 | [round07.md](round07.md) | 曲面 scraping 全链路与四层架构 | 历史文件 |
| Run 8 | [round08.md](round08.md) | 人工画线→3D、曲面参数化与误差预算 | 历史文件 |
| Run 9 | [round09.md](round09.md) | 人体按摩直接 baseline、曲面约束与安全层 | 历史文件 |
| Run 10 | [round10.md](round10.md) | 动态机器人超声、运动环相位滞后与软体形变 | 历史文件 |
| Run 11 | [round11.md](round11.md) | 动态接触框架与 FR5 频响/时延实验设计 | main 已归档 |
| Run 12 | [待审 Run 12](https://github.com/hoshizaki-akari/robot/blob/research/run12-2026-10-10/research/guasha_robot_literature/all_round/round12.md) | 接续 G0/G1 与基线审查 | PR #3 pending |
| Run 13 | [待审 Run 13](https://github.com/hoshizaki-akari/robot/blob/research/run12-2026-10-10/research/guasha_robot_literature/all_round/round13.md) | 动态 CoP/contact-frame 候选与可证伪实验 | PR #3 pending |
| Run 14 | [round14.md](round14.md) | FR5 伺服生命周期、KWR75D 低载荷尺度、统一时序与停止链 | 本分支待审 |

## 三、Run 14 的关键结论

1. **命令周期不是实际运动带宽。** 官方 FAQ 的调用间隔指导不等于 FR5 的真实频响；必须测命令到运动延迟、反馈年龄和抖动。
2. **伺服生命周期是 G0。** 官方 SDK 定义 ServoMoveStart/ServoMoveEnd；开放 issue #39 报告 FR5 上未正确启停的风险，但仍需在本项目版本中独立复测。
3. **KWR75D 规格需要低载荷校准。** 官方页面列出 500/700 N 量程与 18 N·m 力矩满量程；满量程百分比换算只是风险提示，不是实物低载荷误差。
4. **G0 未通过，G1–G7 不得越级。** 不开展接触闭环、双向刮痧 FSM 或 residual/offline RL。

## 四、怎么读每一轮

每个 roundNN.md 使用七节模板：1 本轮问题、2 前置知识、3 核心讲解、4 公式直觉、5 仍未解决的问题、6 记住的三件事、7 前后轮关系。详细证据、链接、元数据、baseline 矩阵和台架方案以对应 daily 报告为准。

## 五、项目路线与术语

- [项目路线图](../project_roadmap.md)：G0–G7、模块接口、baseline、实验与决策日志。
- [术语表](glossary.md)：坐标、wrench、CoP、接触框架、时序和控制术语。
- [README](../README.md)：当前 Gate 状态与目录说明。
- [Run 14 详细报告](../daily/2026-10-11_run14.md)：官方资料/项目清单、9 项高价值工作、结构化精读、实验与证据分级。

## 六、当前 Gate 与下一步

- **G0 阻塞**：现场版本、状态数据年龄、端到端延迟、KWR75D 低载荷校准与独立停止链尚未实测。
- **G1 未验收**：手眼/TCP/wrench 坐标与参考点变换要用刚性工装和已知力臂验证。
- **G2–G4 暂停**：几何轨迹、动态 CoP 与柔顺控制都依赖可信输入。
- **G5–G7 不推进**：状态机、创新验证和学习策略不得越级。
下一轮优先使用真实台架数据；若没有数据，先完善日志器、版本清单和时戳 schema，而不是重复泛搜导纳论文。

---
*本索引不把开放 PR、用户 issue 或其他机器人平台的性能数字当作本项目实测。*