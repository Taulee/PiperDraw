"""加载当前版本的输入类型、提示词和有序参考图。"""

from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class AppConfig:
    source: str
    prompt: Path
    references: tuple[Path, ...]


def load_app_config(path: Path) -> AppConfig:
    values = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(values, dict):
        raise TypeError("应用配置必须是 YAML 映射")
    source = values["source"]
    if source not in ("photo", "line_art"):
        raise ValueError("应用配置 source 必须是 photo 或 line_art")
    prompt = values["prompt"]
    if not isinstance(prompt, str) or not prompt.strip():
        raise ValueError("应用配置 prompt 必须是非空路径字符串")
    references = values["references"]
    if not isinstance(references, list) or any(
        not isinstance(reference, str) or not reference.strip() for reference in references
    ):
        raise ValueError("应用配置 references 必须是路径字符串列表")
    if len(references) > 9:
        raise ValueError("应用配置 references 最多允许 9 张参考图")
    # 路径只依赖版本内部布局，整个版本移动后仍按新位置解析。
    return AppConfig(
        source,
        (path.parent / prompt).resolve(),
        tuple((path.parent / reference).resolve() for reference in references),
    )
