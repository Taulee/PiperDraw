# 新平滑路径首次完整实机运行

日期：2026-11-04，Asia/Shanghai。按用户要求，使用默认运动配置和修改后的路径生成流程，重新绘制最后一个成功人物任务。

## 结果

- 源人物任务：`output/draws/20261103_232337_135430`，使用其保存的线稿重新生成路径。
- 本次任务：`output/draws/20261104_210510_752801`。
- 路径实现版本：`d5a18c8`，标签 `path-smoothing-v1`。
- 完整规划通过，123 笔；执行报告为 `executed`，491／491 个运动段、123／123 笔完成。
- 自动返程报告为 `returned`，`holding: true`；工作流为 `done`、`idle_returned: true`。
- 计划运动时间 788.789 秒；从首次 Execute 接受至最后一次 Execute 成功，实际执行跨度 835.270 秒，约 13 分 55 秒。此跨度含段间调度，不含此前准备／规划和之后的待机返程。
- 上述执行时间为北京时间 21:05:45.911 至 21:19:41.181。

这是一次完整运行验证，没有通过相同条件的旧／新路径重复对照量化纸面晃动，也没有从照片测量墨迹误差；不能把执行成功等同于画质已达到某个精度。

## 配置与标定

沿用当前 Web 工作配置中今天重新测量的纸面，标定 ID 为 `paper-2bc04aa6482a4e03af2fadd7a6048aa1`。没有使用旧人物任务的纸面坐标。TCP 与当前笔姿沿用当前标定；声明笔宽仍为 0.1 mm。

将工作配置中的几何项更新到新模板默认值：`simplify_mm: 0.15`、`step_mm: 1.0`、`curve_chord_error_mm: 0.03`，移除废弃的 `smoothing_span_mm`。更新前的完整配置已单独备份；工作配置保留这些新值供之后重新准备任务使用。

默认运动参数保持：绘画 10 mm/s，空移 300 mm/s，落笔 5 mm/s，抬笔 30 mm/s；抬笔高度 8 mm；基础关节速度／加速度缩放 0.4／0.8，空移缩放 1.0／1.0。设备为 can0、V3 协议、new DH，固件速度参数为 100%。

执行通过项目的 `draw --history --from-line-art --mode real` 工作流完成，先重新取线、规划，再使能和执行。既有反馈、跟踪、停止和返程保护保持生效。旧任务未覆盖。

## 保存记录

运行汇总、配置备份与反馈数据位于 `output/path-smoothing/real-20261104_210449/`：

- `manifest.json`：源任务、配置、执行时间、完成计数和返程状态。
- `working-config-before.yaml`：更新平滑参数前的工作配置。
- `working-config-used.yaml`、`device.yaml`：本次使用的配置快照。
- `feedback/`：已正常结束并写出 metadata 的 MCAP 录制，时长约 890.293 秒，共 329,780 条消息。

录制包含 `/piper/joint_commands`、`/piper/raw_joint_states`、`/piper/status`、`/joint_states`、控制器期望／实际状态与反馈笔迹话题。录制结束只停止本次 recorder；实机后端留在返程后的位置保持状态。

本次任务目录另保存新的任务 YAML、SVG、逐段平滑诊断、规划报告 `plan.yaml`、执行报告 `execute.yaml` 和工作流报告 `workflow.yaml`。

此前 [离线实施记录](path-smoothing-results.md) 中的 ROS／Docker 不可用，属于初次代码实施时的环境状态。本次已可通过现有容器完成上述实机全流程；独立 mock、新旧墨迹对照和机械晃动定量诊断仍待进行。
