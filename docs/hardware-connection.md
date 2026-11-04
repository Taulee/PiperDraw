# Orange Pi 真机连接记录

本文记录迁移前的 Orange Pi 环境和联调结果。目标机的 Docker 用法见 [Docker 使用与迁移](docker.md)。
当时的项目目录为 `/home/robotdraw/piperdraw`，原始项目来自 GPU 的 `/root/piper-draw`。
源码、配置、输入与历史输出已复制；x86 虚拟环境和构建产物未复制，官方依赖提交已核对。

本机使用 USB-CAN 与 Piper SDK 连接，不启动 ROS、MoveIt 或仿真。
连接检查只读；另有显式执行的恢复与回零命令，见下文。
真机纸面与 TCP 尚未标定，`config/real.yaml` 保持 `calibrated: false`。

## 运行环境

- 宿主：openEuler 22.03 LTS SP3，ARM64，现有 Docker 18.09 服务。
- 连接镜像：`piperdraw/hardware:0.1`，Python 3.10 与锁定提交的官方 SDK `0.7.0a1`。
- 复用已缓存的 Python 3.10 Debian Bookworm 镜像，安装 SDK、PyYAML、CAN 工具及标定／图像处理依赖；它不是完整 ROS 运行环境。
- 容器共享宿主网络以访问 `can0`，使用调用用户的 UID/GID，运行时移除所有 capabilities。
- 源码以只读方式挂载，`output/` 可写；设备记录位于 `output/hardware/`，Web 数据位于 `output/web/`。密钥和历史图片不会打包进镜像。
- 复用本机 Docker 服务；不修改其他用户的镜像标签、容器或服务配置。

## 操作

所有命令在项目根目录运行；普通用户无 Docker 权限时，容器入口使用 `sudo docker`。

```bash
cd ~/piperdraw
# 首次安装，或 SDK / Dockerfile / 依赖改变后重建。
bash scripts/build_hardware.sh

# 每次开机或重插适配器后检查；本命令配置接口，不使能电机。
sudo bash scripts/can_up.sh can0

# 固件、六轴、夹爪与使能状态检查；只发送 SDK 初始化所需的参数/固件查询。
./piper-draw robot doctor

# 若固件可查询但没有周期关节帧，显式设置从臂/运动输出模式，再检查。
# 此命令配置 CAN 角色与默认消息 ID，不发送使能或运动目标。
./piper-draw robot feedback-on

# 被动监听 5 秒，不发送任何 CAN 帧。
./piper-draw robot listen --seconds 5

# 使用假传输测试诊断逻辑，不连接实体 CAN。
uv run pytest -q tests/test_hardware_*.py tests/test_device.py tests/test_robot_cli.py
```

本机已读到固件 `S-V1.8-9`，`config/device.yaml` 统一配置 `can0`、`v3` 协议和 `new` DH。
所有 SDK 操作及 ROS 真机启动共用这份文件；命令行显式参数可覆盖。
如果换接不同固件的设备，按诊断提示显式指定协议，例如：

```bash
./piper-draw robot doctor --protocol v2 --dh-version new
```

设备操作成功时在标准输出打印 JSON；进度、记录路径及错误写入标准错误。
每次 `doctor/feedback-on/home/resume` 在 `output/hardware/` 保存带时间戳的 JSON，
并原子更新 `latest.json` 为最近一次尝试的结果，失败也更新，`status: failed` 与 `error` 说明原因。
关节角为 rad、夹爪开口为 mm；`enabled` 是读到的六轴状态，`checked_at` 为成功检查时间。
`--report` 可指定本次记录路径，仍会更新默认目录的 `latest.json`。
检查后容器退出并释放连接锁。原有历史 JSON 保留原样，其成功快照语义不变。

CAN 接口必须为 1 Mbit/s 且处于 UP 状态。若反馈超时，先核对机械臂电源、USB-CAN 到机械臂的 CAN 线、
接口和总线错误计数。`ERROR-ACTIVE` 是 CAN 正常工作状态名称，需结合 `bus-off` 和错误计数判断。
SDK 在连接前取得 `output/hardware/locks/<接口>.lock` 的进程锁，直到连接关闭才释放。
容器与宿主映射同一目录，重复操作会明确报占用。锁文件保留，无需手动删除。
ROS bridge 持有同一把锁；运行时用 ROS 服务读取状态和提交操作，停止后端后再独立回零或恢复。
此锁约束本项目入口；厂商工具等外部控制程序仍需避免并行运行。

## 本机验证结果

2026-11-01 在本机完成：

- 构建 ARM64 镜像 `piperdraw/hardware:0.1`，兼容已有 Docker 18.09。
- USB-CAN 为 `can0` / `gs_usb`，1 Mbit/s；总线错误、丢包和 bus-off 计数均为零。
- 原先固件查询有响应，但没有周期关节反馈。按官方 SDK 说明显式发送一次
  `MasterSlaveConfig(0xFC, 0, 0, 0)`，随后读取通过。
- 固件 `S-V1.8-9`，六轴角度及夹爪开口可读，控制器 `arm_status=0`、`error_code=0`。
- 反馈显示六轴电机均为已使能。本次未发送使能、关节目标或夹爪指令；诊断报告该状态。
- 连续监听约 5 秒，收到 15,729 帧。三组关节帧、控制器状态及夹爪反馈均约 200 Hz，六个电机状态各约 40 Hz。
- 再次运行只读 `doctor` 通过，无需重复配置从臂模式。
- 5 项独立连接测试通过，覆盖只读边界、CAN 未启用、波特率错误、协议不匹配和显式反馈配置。

首次成功快照见 `output/hardware/first-connection.json`，
帧率统计为 `feedback-stability.json`，最初查询响应留在 `can-query-capture.json`。
本轮未验证运动、夹笔或绘画精度；CAN 配置未设为开机自动启用。

## 命令行回零与解除急停

项目根目录执行：

```bash
# 查看参数与说明；容器入口需要 Docker 权限。
./piper-draw robot home --help

# 只读状态。
./piper-draw robot doctor

# 六轴以 3% 速度回到关节零位，运行即使能并运动。
./piper-draw robot home --speed-percent 3
```

入口复用当前 Docker 环境和共享设备配置，回零与恢复目前仅支持 V3 协议、新 DH；其他组合在连接前拒绝。
回零速度范围为 1–5%。
目标固定为六轴关节角零度，不修改电机零点标定。执行后监测反馈、状态、超时和速度，
进入 0.5° 容差并保持 1 秒才报告到达；成功后保持使能。Ctrl+C 请求快速停止。
软件停止请求不能替代现场急停，也不能保证 CAN 断开时传达给设备。

急停状态不会被回零命令自动解除。显式恢复命令为：

```bash
# 恢复软件急停，运行即恢复，不自动使能或回零。
./piper-draw robot resume
```

恢复通过 SDK `ResetPiper` 实现，可能失能并使机械臂下落。已正常时不会重复 reset，
存在其他故障时拒绝恢复。恢复与回零分别执行，避免解除急停后立即自动运动。

2026-11-01 的首次 3% 回零试验在使能初期触发了软件速度监测并停止，尚未通过实机回零验收。
现场反馈没有明显突动或碰撞，启动时反馈变化仍待排查。
连接、回零、恢复逻辑使用假传输测试；离线测试不能代替实体回零验收。

新记录为 `output/hardware/<时间戳>_home.json` 和 `<时间戳>_resume.json`；
旧 `home-report.json`、`resume-report.json` 是历史记录，不再更新。


## 开发验证

完整 Python 开发环境使用 `uv sync --locked` 后运行上述 pytest 命令。
硬件镜像包含标定、图片处理和 Web 服务依赖，不包含 pytest 或 ROS。离线回归使用假传输或临时文件锁，
不连接实体 CAN；锁测试启动独立进程验证互斥及释放。

`robot home` 直接运动，`robot resume` 直接恢复，`run` 直接执行；`plan` 仅规划。
回零使用固件的 1–5% 速度设定。角度差分估算的峰值速度仅记录在成功报告中。
反馈断流、控制器故障、急停、超时和未向零位收敛仍会中止回零。
