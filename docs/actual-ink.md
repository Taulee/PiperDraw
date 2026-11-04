# 反馈笔迹

浏览器显示来自执行关节反馈的笔尖几何轨迹。目标 SVG 与反馈笔迹用途不同；mock 提供理想关节反馈，不模拟接触力、摩擦或墨水。
连接、布局和录制步骤见 [浏览器仿真](browser-simulation.md)。

## 输入、输出与处理

输入是任务纸面、TCP、当前运动段与控制器 `/joint_states`。输出为标准 `visualization_msgs/MarkerArray`，包含薄纸和独立墨线。

1. `bringup` 启动常驻 `drawing_scene`，从配置三点与尺寸发布纸面，无需先运行任务。
2. 执行器用同一 MoveIt 模型对关节反馈求正解，转到任务基坐标系，再施加法兰 TCP 偏移，得到笔尖位置。
3. 只在实际进入 `draw` 段且笔尖距纸面不超过 0.5 mm 时采样。反馈须包含完整关节、有限位置值和有效时间；非零时间戳还须足够新鲜且不倒退。
4. 每个绘制段建立独立线条；离纸或反馈超过 250 ms 未更新时断线，恢复后不跨过未观测的移动。
5. 点投影到纸面并沿法向抬高 0.1 mm，避免闪烁；完整快照经内部 `/drawing_task_markers` 交给场景节点，统一发布 `/drawing_markers`。

| 显示参数 | 值 |
|---|---|
| 可视纸厚 | 0.2 mm，上表面与任务纸面重合 |
| 常规采样上限 | 约 100 Hz |
| 常规保留点间距 | 至少约 0.2 mm，段端点可更密 |
| 墨线宽度 | 固定 0.7 mm |
| 快照频率 | 约 5 Hz，段结束立即发送 |

落笔、抬笔、空移和只规划不产生墨线。离纸空跑若超出接触阈值，也不会显示笔迹。
每个快照先 `DELETEALL` 再加入纸面和全部墨线，新任务清空旧画面。

## 保留与实现位置

场景输出使用 reliable / transient-local、深度 1；后端存活时，晚连接可取得最新完整画面，即使任务已退出。
后端停止或重启后，历史画面须用含 `/drawing_markers` 的 MCAP 回放。内部快照主题不需开放给 Foxglove 或录制。

| 文件 | 职责 |
|---|---|
| [visualization.cpp](../ros2_ws/src/piper_drawing/src/visualization.cpp) | 反馈校验、笔尖正解、采样与任务快照 |
| [scene.cpp](../ros2_ws/src/piper_drawing/src/scene.cpp) | 初始化纸面、转发最新画面 |
| [paper.hpp](../ros2_ws/src/piper_drawing/include/piper_drawing/paper.hpp) | 共用薄纸几何 |
| [main.cpp](../ros2_ws/src/piper_drawing/src/main.cpp) | 在段边界开始、结束采样 |
| [foxglove.yaml](../config/foxglove.yaml)、[recording-qos.yaml](../config/recording-qos.yaml) | 显示话题与录制 QoS |

可视化持有独立 `RobotState`，不改规划状态或发送控制命令；显示异常记录警告。
额外笔杆与夹具显示、网页可拖动历史时间轴尚未实现。实机接触与跟踪精度需单独验证。
场景保留和 MCAP 几何检查见 [验证记录](verification.md)。
