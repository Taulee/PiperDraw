# 绘画控制台

Docker Compose 启动后，打开 `http://127.0.0.1:8080`。远程访问可用 SSH 转发。启动方式见 [Docker 使用与迁移](docker.md)。

- **当前任务**。上传或选择图片，明确选择「使用已有线稿」或「从照片生成线稿」，然后点「开始画画」。页面会显示进度、预计时间和停止按钮。照片模式调用已配置的图像服务。
- **标定与姿态**。管理 TCP、纸面定位和待机姿态。绘画直接复用当前工作标定。采点方式见 [标定说明](calibration.md)。
- **历史任务**。预览、选择重画方式或勾选删除。重画可以复用保存的线稿，也可以用原照片重新生成。只保存了路径的旧任务可在标定仍匹配时复用路径。删除记录会同时删除这次任务的生成文件。
- **配置管理**。管理绘画参数、设备和图像服务。设备与图像服务的网页覆盖配置保存在 `output/web/`，命令行绘画也会读取。

Web 每次绘画调用一次和命令行相同的 `draw`。脚本准备线稿、连接机械臂、规划整张图，然后使能并执行。执行入口会按当前反馈重新校验和规划，避免沿用过期起点。遇到机械臂故障或软件停止状态需要在现场处理后重新发起。绘画开始后刷新或关闭页面，任务继续执行。

正常完成、取消绘画和反馈仍有效的轨迹故障，都采用“保持当前位置、取消原轨迹、沿纸面法向离纸、规划并返回待机位、持续位置保持”的收尾方式。返程同时检查纸面净空和场景碰撞；规划失败不会盲目移动，也不会自动关闭已经使能的后端。失联、失能、驱动故障或实体急停时不能保证位置保持和返程，页面会报告未确认保持或返回失败，不自动复位恢复故障。

设备 YAML 的 `return_to_idle` 控制三种情形的返程，当前 `config/device.yaml` 已启用。目标默认使用“标定与姿态”中保存的 `output/hardware/parking.json`；也可在 YAML 明确给出六轴 `positions_rad`。待机目标与当前设备、纸面净空在绘画开始前检查。返程结束保留控制后端，下一次绘画对齐后解除保持；取消不等于关闭驱动。

```yaml
return_to_idle:
  enabled: true
  velocity_scale: 0.2
  acceleration_scale: 0.4
  min_tip_height_mm: 4.0
  timeout_s: 120.0
  # positions_rad: [q1, q2, q3, q4, q5, q6]  # 可选，单位 rad
```

绘画参数以 `output/web/drawing.yaml` 为当前工作配置；`motion` 中配置空移／绘画／落笔／抬笔速度、抬笔高度和关节缩放，`robot` 中配置绝对关节速度／加速度上限。设备配置中的 `firmware_speed_percent` 是独立的固件比例。新任务保存解析后的速度；历史路径会保留原任务的速度，修改工作配置后需要重新生成任务才能应用新的速度。

当前实机默认速度为空移／绘画／落笔／抬笔 `300／10／5／30 mm/s`，基础关节速度／加速度缩放为 `0.4／0.8`，空移与抬笔缩放为 `1.0／1.0`，抬笔高度为 `8 mm`。实机默认模板为 `config/real.yaml` 与 `config/real.example.yaml`；当前工作配置沿用已有标定。

```bash
./piper-draw draw input/style_reference.png --from-line-art
./piper-draw draw input/test3.jpg --from-photo
./piper-draw draw input/style_reference.png --from-line-art --mode mock
./piper-draw history
./piper-draw draw <历史ID> --history --from-line-art
./piper-draw history --delete <历史ID>
./piper-draw stop
```

新任务保存在 `output/draws/`。历史还会读取此前保存在 `output/` 与 `output/web/jobs/` 下的任务。工作标定和采样存放在 `output/web/`。与绘画无关的手动诊断仍可在高级工具和原有 CLI 子命令中使用。
