# 技术说明

## 输入、输出与处理流程

输入是人物照片或已有黑白线稿、纸面标定、笔尖 TCP 和运动参数。输出是毫米笔画、SVG、米／弧度制机器人任务、MoveIt 关节轨迹和执行报告。照片模式先调用 Seedream；已有线稿模式全程离线。

1. `src/piper_draw/seedream.py` 复用原方案的提示词与参考图调用图像服务，保存请求快照。
2. `vectorize.py`、`simple_lines.py` 等将线稿骨架转换成纸面毫米笔画。
3. `geometry.py` 用纸面三点构建正交坐标系；`job.py` 将二维笔画变成抬笔、空移、落笔、绘制段。工具姿态从纸面坐标转换到基坐标，减去旋转后的 TCP 偏移，生成法兰目标。
4. `launch/model.py` 读取官方带夹爪 URDF，按固件选择新旧 DH，加载官方 SRDF 和 KDL 配置。六轴规划组为 `arm`，基坐标为 `base_link`，法兰为 `link6`。不改官方关节位置限位；项目提供较低速度和加速度上限。
5. `planner.cpp` 复用原方案完整路径筛选：KDL 首点逆解、后续全路径碰撞和连续性检查、OMPL 接近、时间参数化与笔尖速度限制。不能完成全路径时不执行部分笔画。
6. mock 使用 ros2_control `GenericSystem`；real 使用 `PiperHardware`，向官方 SDK 桥接节点提交六轴位置。MoveIt 统一调用 `arm_controller/follow_joint_trajectory`。
7. `visualization.cpp` 用关节反馈和同一官方模型求笔尖正解，生成纸面与独立笔迹；显示细节见 [反馈笔迹](actual-ink.md)。

## 坐标和单位

配置的纸面三点、TCP、纸张尺寸使用 mm；工具四元数使用 ROS `xyzw`。任务位姿和关节目标使用 m / rad，SDK 六轴指令使用 0.001°，夹爪指令使用 0.001 mm 和 0.001 N·m。

例如 SDK 夹爪反馈 10000 表示总开口 10 mm；模型 `joint7=0.005 m`、`joint8=-0.005 m`。夹爪反馈单独更新，六轴运动不会自动开合夹爪。

默认纸面 A5、仿真 TCP 190 mm、笔姿绕纸面 Y 轴旋转 180°，均是可规划的仿真参数。实机必须测量，不能直接沿用。夹笔时允许笔杆接触 `link6`、夹爪底座和两指，其余机器人和纸面碰撞仍参与检查。

## 真机接口

| 接口 | 类型与用途 |
| --- | --- |
| `/piper/raw_joint_states` | `JointState`，官方 SDK 的六轴与两指反馈，仅收到新鲜的各组 CAN 反馈才发布 |
| `/piper/joint_commands` | `JointState`，ros2_control 六轴位置目标，桥接默认忽略运动发送 |
| `/piper/status` | JSON `String`，固件、DH 版本、使能、六轴角度、开口和诊断 |
| `/piper/check_feedback` | `Trigger`，核对反馈和固件，用于不使能的真机规划 |
| `/piper/preflight` | `Trigger`，额外要求本次会话已使能，用于执行预检 |
| `/piper/enable` | `SetBool`，true 对齐并使能；false 请求快速停止 |
| `/piper/stop` | `Trigger`，请求快速停止 |
| `/piper/hold` | `Trigger`，以新鲜反馈锁定位置并持续发送保持目标 |
| `/piper/resume_commands` | `Trigger`，原目标取消且控制器对齐后恢复轨迹目标 |
| `/piper/set_gripper` | `SetGripper`，总开口 mm、力矩 N·m，返回提交结果 |

SDK 封装只负责官方接口调用和单位转换，不实现 CAN 编解码或逆解。启动只查询参数、读取反馈，不自动复位、使能、回零或设置夹爪零点。夹爪动作必须由明确命令提交。

反馈看门狗分别监测三组关节帧、控制器状态、六个电机状态以及夹爪帧，不能让一组仍在刷新的帧掩盖另一组断流。反馈超过 500 ms、伺服指令超过 250 ms、单次目标跳变超过 0.1 rad 或跟踪偏差超过 0.15 rad 时，拒绝继续跟随原轨迹。实机控制使用位置速度模式，不使用 MIT 力矩模式。

上述故障处理现优先尝试新鲜反馈位置保持；反馈、驱动或控制器本身异常导致保持不可用时，才退回快速停止。保持期间忽略旧控制器轨迹，仍检查反馈和设备状态。应用层收到轨迹故障或取消后取消原 Action，完整规划离纸和返回待机位，再解除目标锁执行；返程失败不重试，不自动关闭后端。正常完成同样返程并保持。`robot stop` 现请求位置保持，底层 `/piper/stop` 仍保留为无法保持时的停止接口。

启动入口优先读取工作区中的 Python launch 文件，使设备 YAML 参数传递和模型修改直接生效。C++ 源码修改仍需要重新构建 ROS 包／镜像；新增的 `lift_velocity_scale` 与 `lift_acceleration_scale` 在旧 C++ 执行器上尚不生效，当前已实跑第一档仍使用原有 `travel_*_scale` 约束抬笔。

模型与后端模式不一致、模型参数不一致、固件和 DH／协议不匹配、未标定任务都会被拒绝。后台进程退出时关闭本套后端。

停止消息通过同一 CAN 链路发送；CAN 物理断开时无法保证到达设备。实体保护与实际停止状态必须现场核对。

## 复用范围

图像和路径算法来自 `cartoon_simple`，Python 包与命令改名为 `piper_draw` / `piper-draw`，ROS 包为 `piper_drawing`。原项目未修改。依赖源码保持官方提交，未把旧项目的厂家驱动或厂家预检服务带入本项目。

`vendor/piper_ros` 只构建 `piper_description`；官方 KDL、SRDF 配置复制到绘画包中，运行时调整求解超时和绘画限速。官方 SDK 负责 CAN 驱动。用薄桥接代替直接启动通用遥操作节点，以保留默认只读、逐组反馈新鲜度和独立夹爪控制语义。


## 设备操作与后续 Web 接口

设备操作输入是 `device.load_device_config()` 返回的不可变 `DeviceConfig`，
以及操作参数，例如回零速度百分比。`robot.doctor()`、`robot.home()` 和 `robot.resume()`
返回包含反馈和结果状态的字典，失败直接抛出异常。它们不解析命令行、不打印、不安装信号处理器。
`home()` 的可选 `progress(feedback, seconds)` 回调提供运行进度；不传回调时无进度输出。

```python
from piper_draw.device import load_device_config
from piper_draw import robot

config = load_device_config("config/device.yaml")
result = robot.doctor(config)  # 查询反馈；不使能、不运动
```

配置解析顺序为显式函数参数覆盖文件值，不读取连接参数环境变量，也不回退其他配置路径。
`lock_dir` 相对设备配置文件解析；CLI、容器和 ROS 必须使用同一宿主目录。
SDK 连接先取得对应 CAN 接口的进程锁，再初始化官方 SDK，关闭时释放锁；初始化失败也释放。
ROS 真机启动从同一设备配置解析连接参数，并检查绘画配置的 DH 与设备一致，再构建模型。
绘画任务中的 DH 是任务快照，仍参与既有模型一致性检查。

调用顺序为 CLI 参数解析、配置加载、业务函数、SDK 连接与反馈检查、实际操作、结果返回。
`hardware_cli.py` 负责 JSON 输出、SIGTERM 转为中断、进度显示和运行记录。
回零循环在中断或故障时仍请求快速停止；SDK 关闭后才保存记录。
`homing.py`、`recovery.py` 实现设备动作，`piper_backend.py` 保留单位转换、固件校验和会话保护。

后续 Web 后端直接调用上述业务函数，HTTP 层把返回值转成 JSON，把异常转换成明确失败响应。
回零等长操作放在后端工作任务中执行，进度通过回调传给 UI；任务结果以完成后的反馈为准。
当 ROS bridge 持有连接时，通过 `ros_client.command(action, ...)` 访问已有 ROS 服务，
其中 `action` 为 `status`、`enable`、`stop` 或 `gripper`；独立 SDK 操作会报 `DeviceBusyError`，不绕过控制权检查。
当前 `robot stop` 面向 ROS bridge；独立回零通过运行它的 CLI 接收 Ctrl+C 或 SIGTERM 请求停止。
Web 接入长操作时需明确任务取消与停止通道，避免把 HTTP 断开当作机械臂已经停止。

本地 Web 控制台已接入，入口为 `piper-draw web`，使用标准库 HTTP 服务和静态 HTML/JS。
HTTP 层只构造允许的 CLI 参数，以独立进程复用已有操作及中断处理；一个前台任务与一个常驻 ROS 后端分别管理。
上传、工作配置和日志保存到 `output/web/`，操作与部署见 [Web 使用说明](web.md)。

标定使用同样的业务／CLI 分工，接口与流程见 [TCP 标定与纸面定位](calibration.md)。
`calibration.py` 提供只读采点、TCP 最小二乘和纸面四角定位，在实测四边形内选取完整 A5；`calibration_cli.py` 负责参数与 YAML 写入。
`paper.drawing_area` 可选，包含纸面内的二维 `origin_mm`、`width_mm`、`height_mm`；
未设置时仍使用整个纸面，设置后线稿缩放与留白作用于选区，笔画坐标仍以完整纸面原点表达。
纸面场景、碰撞和既有任务格式保持使用完整纸面，TCP 与姿态仍由现有任务生成器转换。
