# FR5 双平台只读数据工作区

本目录是在 WSL 内运行的第一阶段实现：

```text
FR5 / KWR75D / AG95 / D435
              ↓
        state_service :8765
           ↙         ↘
 platform_a        platform_b/gateway :8080
 Tkinter             浏览器
```

## 目录

- `state_service/`：统一状态模型、回放源和真机只读源。
- `platform_a/`：平台1，支持离线仿真与真机只读切换。
- `platform_b/`：平台2，独立 REST/WebSocket 网关和网页。
- `scripts/`：启动、状态检查和冒烟测试脚本。

## 一次性准备

```bash
cd /home/zhj/projects/fr5_platform_ws
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
cp /home/zhj/projects/fr5_learning/.venv/lib/python3.10/site-packages/fr5_learning_fairino_sdk.pth \
   .venv/lib/python3.10/site-packages/
```

## 开发检查顺序

先启动统一状态服务：

```bash
cd /home/zhj/projects/fr5_platform_ws
source .venv/bin/activate
python scripts/start_state_service.py
```

输入 `1` 使用回放，输入 `2` 使用真机只读。

第二个 WSL 终端启动平台1：

```bash
cd /home/zhj/projects/fr5_platform_ws
source .venv/bin/activate
python platform_a/main.py
```

第三个 WSL 终端启动平台2：

```bash
cd /home/zhj/projects/fr5_platform_ws
source .venv/bin/activate
python platform_b/gateway.py
```

浏览器打开 `http://127.0.0.1:8080`。

## 自动冒烟测试

确保上述三个程序已退出，避免端口占用，然后执行：

```bash
cd /home/zhj/projects/fr5_platform_ws
source .venv/bin/activate
python scripts/smoke_test.py
```
# A、B平台一键启动

真机接线并打开控制箱后，在 WSL 中运行：

```bash
cd /home/zhj/projects/fr5_platform_ws
bash scripts/platforms.sh start
```

这条命令会依次启动 D435、共同数据服务、B平台和A平台，并打开B平台网页。
A、B只读取同一份共同数据，不会分别抢占机械臂连接。

常用命令：

```bash
# 查看是否都正常
bash scripts/platforms.sh status

# 全部停止（之后可使用法奥官方网页）
bash scripts/platforms.sh stop

# 停止A、B相关服务，并打开法奥官方网页
bash scripts/platforms.sh official

# 全部重新启动
bash scripts/platforms.sh restart
```

不要与平台同时运行 `scripts/02_robot_mode_and_enable.py`。一键启动发现它后会先将其停止。
