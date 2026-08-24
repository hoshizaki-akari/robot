# 骨伤牵引机器人工作站（WSL 本地版）

平台2由两个部分组成：

- `gateway.py`：独立 REST/WebSocket 网关，读取统一状态服务。
- `090105.html`：浏览器界面，通过同源 `/ws` 接收实时状态。

界面保持最初原型不变。真实状态和恒力牵引计算在后台完成，当前不向 FR5 发送运动命令。

牵引力计算流程：

1. 夹爪不受额外外力时点击“开始牵引并记录”，后台自动记录零点。
2. 沿计划牵引方向轻拉一次，超过 3 N 后后台锁定该方向；手拉大小不作为目标力。
3. 后台只计算锁定方向上的牵引力，不把侧向力冒充牵引力。
4. 小于预设目标时，计算机械臂沿受力反方向增加牵引；大于目标时反向卸力。
5. 传感器每次更新都会重新比较，持续把牵引力调到预设值附近。

力控参数在 `platform_b/config.json`，不会在界面中增加额外项目。

启动：

```bash
cd /home/zhj/projects/fr5_platform_ws
source .venv/bin/activate
python platform_b/gateway.py
```

浏览器打开 `http://127.0.0.1:8080`。

检查力控计算：

```bash
cd /home/zhj/projects/fr5_platform_ws
source .venv/bin/activate
python scripts/check_force_control.py
```
