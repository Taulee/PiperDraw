# 浏览器仿真

使用无头 MoveIt/mock 后端、Foxglove Bridge 和本机浏览器查看机械臂、纸面与反馈笔迹。安装及任务生成见 [README](../README.md)，显示规则见 [反馈笔迹](actual-ink.md)。
以下命令在服务器项目根目录执行，相关终端使用同一 ROS 域。

## 实时查看

终端一，启动后端；无需桌面显示环境：

```bash
source scripts/env.sh
export ROS_DOMAIN_ID=106 ROS_LOCALHOST_ONLY=1
uv run piper-draw bringup --mode mock --config config/sim.yaml
```

终端二，启动 Bridge：

```bash
export ROS_DOMAIN_ID=106
bash scripts/foxglove.sh
```

脚本默认使用域 106，接受显式 `ROS_DOMAIN_ID`，并设置本机 ROS 通信。Bridge 只监听服务器 `127.0.0.1:8765`。

在 VS Code 远程窗口的 **Ports** 面板转发远端端口 `8765`。本机 Foxglove 选择 **Foxglove WebSocket**，连接 `ws://127.0.0.1:8765`；若 Ports 分配了其他本地端口，使用实际端口。保持远程会话打开。

将服务器的 `config/foxglove-layout.json` 下载到本机并导入。手动设置 3D 面板时：

- Fixed frame / Display frame：`base_link`。
- URDF 自定义层：Topic `/robot_description`，Control mode 为 Transforms，Mesh up axis 为 Z。
- 启用 `/drawing_markers`，显示 `drawing_paper` 和 `drawing_ink`。

后端启动即显示配置纸面；任务开始后采用任务纸面并清空旧笔迹。`run` 才会运动和生成笔迹。
任务退出后，后端仍保留最后画面，刷新或重连可见；导入文件不会自动更新账户中已有布局。

## 录制与回放

后端启动后、执行任务前，另开终端录制；目录须尚不存在：

```bash
export ROS_DOMAIN_ID=106
bash scripts/record_sim.sh record output/recordings/drawing-001
```

任务结束后按 Ctrl+C 保存 `.mcap` 与 `metadata.yaml`。省略目录时自动创建 `output/recordings/日期-时间`。
录制包含 `/tf`、`/tf_static`、`/joint_states`、`/robot_description`、`/drawing_markers`；QoS 配置允许晚启动时取得模型、静态 TF 和当前画面，但完整笔迹增长过程须提前录制。

回放前停止实时后端与实时 Bridge，避免同域发布冲突。分别在两个终端运行：

```bash
# 终端一：使用录制时钟的 Bridge。
export ROS_DOMAIN_ID=106
bash scripts/foxglove.sh play
```

```bash
# 终端二：单次回放，可追加 --loop 或 --rate 0.5。
export ROS_DOMAIN_ID=106
bash scripts/record_sim.sh play output/recordings/drawing-001
```

本机复用原端口转发、地址和布局。回放无需 MoveIt 或控制器，服务器仍须保留厂家模型包，Bridge 用它提供 STL 网格。
播放器发布 `/clock`，回放 Bridge 使用 `use_sim_time=true`。空格暂停/继续，上下键调速，暂停时右键逐条播放，Ctrl+C 停止。

当前经 Bridge 回放是实时数据连接，没有网页历史时间轴拖动。直接打开 MCAP 不会自动附带 `package://` 模型网格。
旧录制若没有 `/drawing_markers`，不能自动补出纸面与笔迹；SVG 只表示目标路径。

## 排障

| 现象 | 检查 |
|---|---|
| WebSocket 连不上 | Bridge 是否运行、8765 是否转发、Ports 的实际本地端口是否一致 |
| 已连接，无机械臂 | 后端是否运行，ROS 域是否一致，是否有 `/robot_description`、`/tf`、`/joint_states` |
| 有机械臂，无纸面 | `drawing_scene` 是否运行，3D 面板是否启用 `/drawing_markers` |
| 没有运动或新增笔迹 | 是否使用 `run` 执行，而非 `plan`，报告是否成功，当前是否处于绘制段 |
| 回放缺模型 | 厂家模型包是否保留，是否使用回放 Bridge 提供网格 |
| 重启后没有旧画面 | 使用保存的 MCAP 回放；常驻场景只保留本次后端生命周期的画面 |

历史验收与检查入口统一见 [验证记录](verification.md)。
