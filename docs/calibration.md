# TCP 标定与纸面定位

标定入口是 `piper-draw calibrate`。本机使用 `./piper-draw`；原生完整环境使用
`uv run piper-draw`，参数相同。轻量硬件镜像需要重新运行
`bash scripts/build_hardware.sh`，安装与主项目相同的 NumPy、SciPy。
采点命令只读取反馈；手动定位、接触判断由操作者完成，不使能或移动机械臂。

## 1. TCP 标定

固定笔和夹具，选一个不会移动的接触点。让笔尖以 4–6 个不同姿态轻触同一点，
绕不同方向改变姿态。每次停止移动、保持轻触后运行一次：

```bash
./piper-draw calibrate capture \
  --config config/real.yaml \
  --samples output/hardware/calibration/tcp_samples.yaml
```

默认通过独立 SDK 读取真实反馈，使用 `config/device.yaml` 的连接参数与既有 CAN 连接锁。
`--device-config`、`--can`、`--protocol` 可显式覆盖连接配置；DH 必须与绘画配置一致。
已有 ROS bridge 持有连接时，在 ROS 环境中使用 `--source ros`，读取 `/piper/status`。
两种来源均按绘画使用的官方 URDF，从实际关节角计算 `base_link` 中的 `link6` 位姿。
SDK 自带 DH 与 URDF 的几何参数存在舍入差异，因此标定不混用 SDK 的末端欧拉角反馈。

同一个 `--samples` 文件会追加样本。TCP 和纸面使用不同文件；重新采整组时使用新文件名。
错误样本可以从 YAML 的 `samples` 列表中删除。完成后求解：

```bash
./piper-draw calibrate tcp \
  --config config/real.yaml \
  --samples output/hardware/calibration/tcp_samples.yaml \
  --output output/hardware/calibration/tcp.yaml
```

输出完整绘画配置，更新 `tool.tcp_offset_mm`，记录拟合误差并生成新的 `calibration_id`。
输出保留原有图像、机器人和运动参数，`calibrated` 为 `false`，旧纸面标定结果被撤销。
原输入配置保持原样，除非显式指定同一个输出路径。

求解器将各次 `法兰位置 + 法兰旋转 × TCP偏移` 约束到同一个未知点，使用最小二乘。
至少需要 4 个样本；拒绝秩不足、姿态变化太小或病态方程。
默认要求各接触点到拟合中心的最大残差不超过 1 mm，可通过 `--max-error-mm` 指定。
残差检查衡量采样一致性，不能代替实体落笔精度验证。

笔和夹具不变时可以复用 TCP。换笔、重新夹持或笔杆滑动后重新标定。

## 2. 纸面定位与 A5 选区

固定平整纸张，标记可画范围。从纸面上方看，依次轻触 **左下、右下、右上、左上** 四个角点。
四点只需按顺序围成不交叉的四边形，不要求直角或对边平行；凹四边形也可使用。
每个点保持静止后运行一次以下命令，共四次：

```bash
./piper-draw calibrate capture \
  --config output/hardware/calibration/tcp.yaml \
  --samples output/hardware/calibration/paper_samples.yaml
```

四个点可以使用不同笔姿。第一个点的法兰姿态将成为绘画姿态，应先在左下角摆好笔。
程序用已标定 TCP 将每次法兰位姿转换成真实笔尖位置，用全部四点拟合平面，
以第一条边在平面内的方向作为横向，并将四角投影到该平面。
默认最大平面残差为 1 mm，可用 `--max-plane-error-mm` 指定；不再检查是否为直角。
法向必须朝向法兰所在的一侧，否则拒绝生成抬笔方向。四角采样仍无法检测纸张中间的翘曲。
旧版三点样本需清空后按新顺序重新采集四点；已保存的标定配置仍可直接使用。

求解纸面位置和 A5 选区。

```bash
./piper-draw calibrate paper \
  --config output/hardware/calibration/tcp.yaml \
  --samples output/hardware/calibration/paper_samples.yaml \
  --output output/hardware/calibration/real.yaml
```

TCP 和纸面标定成功后，配置自动写入 `calibrated: true`，可用于生成实机任务。

默认在实测四边形内部放入 `148 × 210 mm` 的 A5；竖向放不下时检查横向。
选区沿纸面横向或纵向摆放，尽量接近范围中心；居中会越界时自动平移到可容纳的位置。
选区的整个矩形必须在四边形内，仅包围盒足够大并不代表能够放下 A5。
可用 `--orientation portrait` 或 `--orientation landscape` 指定方向。
放不下时明确报错，不缩小 A5。

`paper.origin_mm/x_point_mm/y_point_mm/width_mm/height_mm` 描述四边形在纸面坐标中的包围矩形。
`calibration.paper.measured_corners_mm` 保留原始四角，`corners_in_paper_mm` 保存投影后的二维边界；
`plane_rms_error_mm/plane_max_error_mm` 保存平面拟合残差。
`paper.drawing_area` 保存 A5 的二维纸面原点及宽高；`paper.margin_mm` 是 A5 内的留白。
线稿与调参共享同一个选区映射；预览、碰撞场景使用完整包围矩形。
底板超出实测范围的部分仍需通过现有 `obstacles` 配置建模。
纸张、底板或机器人底座移动后重新定位纸面。

## 3. 接入现有绘画流程

在完整 ROS 环境中使用生成的配置启动后端并生成新任务：

```bash
source scripts/env.sh
uv run piper-draw bringup --mode real --config output/hardware/calibration/real.yaml
```

另一终端：

```bash
source scripts/env.sh
uv run piper-draw prepare /绝对路径/线稿.png --from-line-art \
  --config output/hardware/calibration/real.yaml
# 用 prepare 打印的任务文件执行 plan；先验证短线、小方框和离纸空跑。
uv run piper-draw plan /绝对路径/任务_job.yaml --mode real
```

纸面定位后的 `tool.quaternion_in_paper` 来自首个纸面采点的姿态，
已有 `job.make_job()` 仍负责转换笔尖目标、扣除 TCP、生成法兰目标。
标定不会执行使能或绘画；运动继续通过原有 `robot enable`、`run`、`robot stop` 接口。

## Python / 后续 Web 接口

`calibration.capture_pose(config, source="sdk", device=device_config)` 返回一个样本字典，
含 `position_mm`、`quaternion_xyzw`、实际关节角、时间和采样来源。ROS 来源不需要 `device`。
输入绘画配置已解析为字典；`device_config` 来自 `device.load_device_config()`。

```python
from piper_draw.calibration import calibrate_tcp, calibrate_paper, robot_identity

tcp_observations = {"robot": robot_identity(config), "samples": tcp_samples}
tcp_config = calibrate_tcp(config, tcp_observations)
paper_observations = {"robot": robot_identity(tcp_config), "samples": paper_samples}
real_config = calibrate_paper(tcp_config, paper_observations)
```

业务函数返回新配置，不修改输入、不打印、不写文件；失败抛出异常。
`solve_tcp()` 与 `locate_paper()` 可直接用于显示拟合结果。
CLI 负责原子写入 YAML，失败不会覆盖现有输出文件。
`calibration.tcp` 保存本次 TCP 求解结果；纸面定位会核对它与 `tool.tcp_offset_mm`，
拒绝使用被手动改写的 TCP。样本中的基坐标、法兰坐标和 DH 必须与目标配置一致。
