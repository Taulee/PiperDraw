# Piper 无头绘画

在 Linux 上使用 Docker 运行。Web 容器包含 ROS 2 Humble 和 MoveIt；轻量容器提供图像处理、标定与 SDK 诊断。启动、异机迁移和离线搬运见 [Docker 使用与迁移](docs/docker.md)。

```bash
mkdir -p output
sudo PIPER_UID="$(id -u)" PIPER_GID="$(id -g)" docker compose up -d
# 浏览器访问 http://127.0.0.1:8080
```

若当前用户已有 Docker 权限，省略 `sudo`。`./piper-draw` 默认使用轻量容器，加 `--ros` 使用完整容器；两个入口都调用同一套 CLI。下文的原生命令 `uv run piper-draw` 也可换成 `./piper-draw --ros`，路径使用项目内相对路径。
标定流程见 [TCP 标定与纸面定位](docs/calibration.md)，入口为 `./piper-draw calibrate`。
网页选图并明确线稿来源后，点「开始画画」。命令行使用同一流程和历史：

```bash
./piper-draw draw input/style_reference.png --from-line-art
./piper-draw history
```

四个页面标签和历史重画用法见 [Web 使用说明](docs/web.md)。
设备配置由 [config/device.yaml](config/device.yaml) 提供，显式命令行参数覆盖文件值。
同一 CAN 接口的 SDK 连接互斥；ROS 后端运行时通过 `robot status/enable/stop/gripper` 操作，
独立 `doctor/home/resume` 不抢占连接。详见 [技术说明](docs/project-guide.md#设备操作与后续-web-接口)。
源机的连接结果和首次回零中止记录见 [Orange Pi 真机连接记录](docs/hardware-connection.md)。

复用 `/root/TCB605_draw/cartoon_simple` 的提示词、参考图、线稿处理和绘画规划，适配标准六轴 Piper 与原装夹爪。运行环境为 Ubuntu 22.04、ROS 2 Humble、Python 3.10。无需服务器桌面或 GPU，Mac 可通过 SSH 操作和浏览器查看。

机器人模型、网格和 SRDF 来自 [Piper 官方 ROS 2 Humble](https://github.com/agilexrobotics/piper_ros/tree/humble)，CAN 驱动和夹爪控制来自 [官方 SDK](https://github.com/agilexrobotics/piper_sdk)。逆解沿用官方配置的 KDL，接近路径用 MoveIt/OMPL，绘制路径用 MoveIt 笛卡尔插值。源码提交锁在 `vendor.lock`；SDK 固定为该提交的 `0.7.0a1`，用于同时提供 V2/V3 协议接口。

## 安装

以下命令适用于 Ubuntu 22.04 原生环境。新机器上先安装 ROS 2 Humble 和 uv，再运行：

```bash
cd ~/piper-draw
bash scripts/install_ros.sh
bash scripts/setup.sh
source scripts/env.sh
```

每个操作终端都先进入项目并 `source scripts/env.sh`。默认 `ROS_DOMAIN_ID=106`、仅本机 ROS 通信，日志在 `output/ros_logs/`。

## 无头仿真

终端一启动 Piper、纸面、控制器和 MoveIt：

```bash
cd ~/piper-draw
source scripts/env.sh
uv run piper-draw bringup --mode mock
```

终端二使用人物照片生成任务；照片模式会调用 Seedream：

```bash
cd ~/piper-draw
source scripts/env.sh
uv run piper-draw prepare input/test3.jpg
```

已有自己的线稿时，可离线生成任务：

```bash
uv run piper-draw prepare /绝对路径/你的线稿.png --from-line-art
```

将命令打印的 `*_job.yaml` 路径传给 `plan` 做规划，或传给 `run` 规划并执行：

```bash
uv run piper-draw plan /绝对路径/任务_job.yaml
uv run piper-draw run /绝对路径/任务_job.yaml
```

`plan` 仅规划，`run` 规划并执行。mock 是理想关节反馈仿真，不模拟重力、笔压、摩擦和墨水。产物保存在 `output/`，含线稿、SVG、骨架、配置、任务和规划／执行报告。

照片模式保留原方案当前 `prompt.txt` 与 `input/style_reference.png`；密钥读取 `api_key.txt`，不存在时读取 `ARK_API_KEY`。服务配置在 `config/seedream.yaml`。

## 连接真机

接好服务器 USB-CAN、机械臂电源后，先检查接口：

```bash
ip -details link show
sudo bash scripts/can_up.sh can0
uv run piper-draw robot doctor --can can0
```

`doctor` 仅查询固件并读取状态，不使能、不复位、不运动。若接口名不同，替换 `can0`。同一条 CAN 总线上只运行一套控制程序。

官方版本对应关系：

| 固件 | `robot.dh_version` | 启动参数 |
| --- | --- | --- |
| 早于 S-V1.6-3 | `old` | `--protocol v2` |
| S-V1.6-3 至早于 V1.8-8 | `new` | `--protocol v2` |
| V1.8-8 及以后 | `new` | `--protocol v3` |

设备连接统一读取 `config/device.yaml`；本机配置为 `can0` / `v3` / `new`。读到不匹配的固件会拒绝使能；按诊断修改配置和协议后重启。固件阈值依据官方仓库文档，实际设备仍需联调核对。

复制实机配置并启动只读后端：

```bash
cp -n config/real.example.yaml config/real.yaml
uv run piper-draw bringup --mode real --config config/real.yaml
```

另一终端可以读状态和单独调夹爪：

```bash
source scripts/env.sh
uv run piper-draw robot status
# 总开口 15 mm，电机力矩上限 0.3 N·m；此命令会实际操作夹爪。
uv run piper-draw robot gripper --width-mm 15 --torque-nm 0.3
```

本项目夹爪指令限制为总开口 0–70 mm，与官方碰撞模型的每侧 35 mm 行程一致。力矩按 SDK 的电机 N·m 表达，不能直接当作笔杆夹持力。调节命令返回“已提交”，实际开口需查看 `status`，夹住笔时不要求到达空载目标开口。

首次夹笔需手动摆正位置、角度和伸出长度。每次重夹、换笔或换 3D 打印夹具后，重新测量法兰 `link6` 到笔尖的 TCP。原装夹爪保留在碰撞模型中；以后改夹具时需更新几何模型。

实机配置需要测量纸面三点、TCP 和笔姿。`config/real.example.yaml` 是未标定模板。通过 [标定命令](docs/calibration.md) 采集法兰位姿，依次求 TCP、定位纸面并选取 A5。求解成功后，程序生成独立 `calibration_id`，并自动设置 `calibrated: true`。以下 `--config` 指向最终实测配置，用于生成实机任务。

```bash
uv run piper-draw prepare /绝对路径/你的线稿.png --from-line-art --config config/real.yaml
# 用上一步打印的任务路径先规划：
uv run piper-draw plan /绝对路径/任务_job.yaml --mode real
# 先确认周边空间，再明确使能；保持当前反馈位置，不自动回零。
uv run piper-draw robot enable
# 将同一实测任务先改为短线或离纸空跑，再执行。
uv run piper-draw run /绝对路径/任务_job.yaml --mode real
```

`enable` 后如手动改变机械臂位置，应停止并重启后端以重新对齐控制器。实机首次测试应先用短直线、小方框和抬笔空跑，再测试整幅线稿。

软件停止命令：

```bash
uv run piper-draw robot stop
```

停止请求保留夹爪夹持，不主动失能电机。反馈断流、指令断流或跟踪异常会请求控制器快速停止并关闭本次会话的运动发送；软件请求不证明实体已经停住，现场仍需保留实体急停。

## 浏览器查看与验证

Mac 通过 SSH 转发服务器 8765 端口，在 Foxglove 连接 `ws://127.0.0.1:8765`。启动命令、布局和录制见 [浏览器仿真](docs/browser-simulation.md)。

原 GPU 验证命令和范围见 [验证记录](docs/verification.md)，坐标、接口和数据流见 [技术说明](docs/project-guide.md)。
源机真机连接和固件核对记录见 [Orange Pi 真机连接记录](docs/hardware-connection.md)。迁移后需在目标机重新检查 CAN、设备配置及标定。
