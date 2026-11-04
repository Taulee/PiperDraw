# 固定工作流与联合调参

输入是人物照片或已有线稿、本版提示词与参考图、纸面和取线参数。输出为 API PNG、请求快照、毫米笔画、SVG、骨架和 Piper 任务。照片作为图1，`input/style_reference.png` 作为图2；参考图指导画法，人物内容来自原照。

本项目原样复用 `cartoon_simple` 当前的 `prompt.txt`、参考图和取线参数。本次适配只验证已有缓存线稿，没有重新调用 Seedream。

```bash
# 照片模式会调用 API。
uv run piper-draw prepare input/test3.jpg
# 已有线稿全程离线。
uv run piper-draw prepare /绝对路径/你的线稿.png --from-line-art
uv run piper-draw tune /绝对路径/你的线稿.png
```

入口读取 `config/app.yaml` 的提示词和参考图，图像服务配置在 `config/seedream.yaml`。默认一次照片请求，生成结果和路径保存到同一时间戳目录。

`tune` 复用原方案的评价规则：固定 1024 像素参考骨架、0.5 mm 距离容差、24 个画布分块；同时比较覆盖率、偏离、局部覆盖和名义绘画时间。门槛见 `config/tuning.yaml`，这些指标不表示人像相似度或实机接触质量。

```bash
# 调用次数由照片数、重复数和工作流阶段数决定，运行前打印。
uv run piper-draw regress input/test3.jpg input/test4.jpg
uv run piper-draw regress input/test3.jpg input/test4.jpg --repeats 3
```

`config/regression.yaml` 默认只有单次提示词工作流。配置中的提示词路径相对工作流文件；参考图来自 app 配置。所有本版产物保存在 `output/`，不依赖原方案的历史输出目录。
