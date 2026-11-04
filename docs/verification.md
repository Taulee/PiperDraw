# 验证记录

2026-11-01 的 Orange Pi Docker / CAN / SDK 连接验证见 [本机真机连接](hardware-connection.md)。
以下为原 GPU 项目在 2026-10-29 的历史验证记录。

2026-10-29，在 `/root/piper-draw` 的最终路径构建和验证。未调用真实图像 API，未连接或驱动实体 Piper。原 `cartoon_simple` 未修改；提示词与参考图已逐字节核对。

- 构建：`piper_description`、`piper_drawing` 通过，官方依赖提交与 `vendor.lock` 一致。
- Python：284 项通过；Ruff 与 shell 语法检查通过。
- C++ 服务回归：4 个用例通过，零失败；colcon 汇总连同测试入口计为 5 项。
- 无头规划：7 笔回归任务重复 5 次通过；不可达尾部被拒绝；首笔四段完成模拟执行。
- 完整人像：缓存线稿 64 笔、1246.9 mm，完整规划 253 段通过；未执行整幅人像。
- 命令入口：通过实际 CLI 和 launch 完成规划，生成 `planned` 报告。
- 假驱动：首帧目标等于真实反馈；反馈断流后停止发布目标并请求快速停止。不创建 SDK、不连接 CAN。
- 纸面场景：倾斜三点坐标、尺寸、晚连接、任务退出保留、新任务清场通过。

以下为清理前的 Linux 服务器测量，不推算 Mac 性能：

| 项目 | 测量 |
| --- | --- |
| 线稿取线耗时 | 2.423 s |
| 线稿取线峰值 RSS | 113.5 MiB |
| 无头后端空闲 RSS 进程总和 | 283.7 MiB |
| 后端空闲 CPU | 单核占用约 10.6% |
| 完整人像规划耗时 | 3.531 s |
| 规划执行器峰值 RSS | 171.4 MiB |

RSS 按进程相加时可能重复计入共享页。测量未包含 Foxglove、浏览器和真实 CAN 驱动负载。

临时线稿、示例任务、人像性能检查脚本及本轮测试输出已清理。保留必要回归测试；下面的复查命令会重新生成检查产物。

复查命令：

```bash
cd ~/piper-draw
source scripts/env.sh
uv run pytest -q
uv run ruff check src tests scripts ros2_ws/src/piper_drawing/launch
uv run python tests/check_planner.py --output output/verification/planner
uv run python tests/check_hardware.py
ROS_DOMAIN_ID=118 uv run python tests/check_scene.py
cd ros2_ws
colcon test --packages-select piper_drawing --ctest-args -R test_service --output-on-failure
colcon test-result --verbose
```

各 ROS 检查使用独立本机域；检查结束关闭其后端。

真机准备：`config/real.yaml` 已创建为未标定模板，保持 `calibrated: false`。当前服务器仅有 `eth0`、`lo`，尚无 `can0`。真机 CAN、固件、使能、实际夹笔、纸面与 TCP 标定、接触和轨迹跟踪均待连接后联调。mock 与假驱动结果不能作为实机绘画精度证明。
