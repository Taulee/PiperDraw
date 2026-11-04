"""按固定提示词链重复调用 API，再在全部末稿上联合调参。"""

import shutil
from dataclasses import asdict
from pathlib import Path

import yaml

from .artifacts import Artifacts
from .seedream import generate_line_art, load_seedream_config
from .tuning import file_digest, load_tuning_config, save_json, tune_paths


def load_workflows(path):
    values = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(values, dict) or values.keys() != {"workflows"}:
        raise ValueError("工作流配置必须只包含 workflows")
    workflows = values["workflows"]
    if not isinstance(workflows, dict) or not workflows:
        raise ValueError("workflows 必须是非空的命名提示词链映射")
    loaded = {}
    for name, prompts in workflows.items():
        if not isinstance(name, str) or not name:
            raise ValueError("工作流名称必须是非空字符串")
        if not isinstance(prompts, list) or not prompts:
            raise ValueError(f"工作流 {name} 必须至少有一份提示词")
        loaded[name] = []
        for prompt in prompts:
            if not isinstance(prompt, str) or not prompt:
                raise ValueError(f"工作流 {name} 的提示词路径必须是非空字符串")
            prompt_path = (path.parent / prompt).resolve(strict=True)
            text = prompt_path.read_text(encoding="utf-8")
            if not text.strip():
                raise ValueError(f"提示词不能为空：{prompt_path}")
            loaded[name].append((prompt_path, text))
    return loaded


def regress(
    images,
    workflow_path,
    seedream_path,
    config_path,
    tuning_path,
    repeats=1,
    *,
    reference_images=(),
):
    if isinstance(repeats, bool) or not isinstance(repeats, int) or repeats < 1:
        raise ValueError("回归重复次数必须是正整数")
    workflows = load_workflows(workflow_path)
    generation_config = load_seedream_config(seedream_path)
    load_tuning_config(tuning_path)
    config_path.resolve(strict=True)
    images = [Path(image).resolve(strict=True) for image in images]
    reference_images = [Path(reference).resolve(strict=True) for reference in reference_images]
    if len(reference_images) > 9:
        raise ValueError("回归最多允许 9 张参考图")
    if not images:
        raise ValueError("回归至少需要一张照片")
    artifacts = Artifacts.create()
    artifacts.path("workflow", ".yaml").write_text(workflow_path.read_text(encoding="utf-8"))
    manifest = {
        "expected_api_calls": repeats * len(images) * sum(map(len, workflows.values())),
        "status": "generating",
        "runs": [],
        "samples": [],
    }
    manifest_path = artifacts.path("regression", ".json")
    save_json(manifest_path, manifest)
    print(
        f"固定回归：预计 {manifest['expected_api_calls']} 次 API 调用；目录：{artifacts.directory}",
        flush=True,
    )
    final_images = []
    for workflow_index, (name, prompts) in enumerate(workflows.items(), start=1):
        for repeat in range(1, repeats + 1):
            for image_index, image in enumerate(images, start=1):
                directory = (
                    artifacts.directory
                    / f"workflow_{workflow_index:02d}"
                    / f"repeat_{repeat:02d}"
                    / f"image_{image_index:02d}"
                )
                directory.mkdir(parents=True)
                original = directory / f"original{image.suffix.lower()}"
                shutil.copyfile(image, original)
                current = original
                for stage, (prompt_path, prompt) in enumerate(prompts, start=1):
                    stage_directory = directory / f"stage_{stage:02d}"
                    stage_directory.mkdir()
                    stage_artifacts = Artifacts(stage_directory, f"stage_{stage:02d}")
                    source = stage_artifacts.path("source", current.suffix.lower())
                    shutil.copyfile(current, source)
                    stage_artifacts.path("prompt", ".txt").write_text(prompt, encoding="utf-8")
                    stage_artifacts.path("seedream", ".yaml").write_text(
                        yaml.safe_dump(
                            asdict(generation_config), sort_keys=False, allow_unicode=True
                        ),
                        encoding="utf-8",
                    )
                    output = stage_artifacts.path("line_art", ".png")
                    reference_paths = []
                    reference_records = []
                    for index, reference in enumerate(reference_images, start=1):
                        snapshot = stage_artifacts.path(
                            f"reference_{index}", reference.suffix.lower()
                        )
                        shutil.copyfile(reference, snapshot)
                        reference_paths.append(snapshot)
                        reference_records.append(
                            {
                                "source": str(reference),
                                "snapshot": str(snapshot),
                                "sha256": file_digest(snapshot),
                            }
                        )
                    record = {
                        "workflow": name,
                        "repeat": repeat,
                        "image": str(image),
                        "stage": stage,
                        "input": str(source),
                        "input_sha256": file_digest(source),
                        "prompt": str(prompt_path),
                        "prompt_sha256": file_digest(stage_artifacts.path("prompt", ".txt")),
                        "references": reference_records,
                        "output": str(output),
                        "status": "requesting",
                    }
                    manifest["runs"].append(record)
                    save_json(manifest_path, manifest)
                    print(
                        f"API {len(manifest['runs'])}/{manifest['expected_api_calls']}：{name} · {image.name} · 第 {repeat} 次 · 阶段 {stage}",
                        flush=True,
                    )
                    try:
                        generation_options = (
                            {"reference_images": reference_paths} if reference_paths else {}
                        )
                        current = generate_line_art(
                            source, prompt, generation_config, output, **generation_options
                        )
                    except (OSError, ValueError, TypeError, RuntimeError) as error:
                        record.update(status="failed", error=str(error))
                        manifest["status"] = "failed"
                        save_json(manifest_path, manifest)
                        raise
                    record.update(status="generated", output_sha256=file_digest(current))
                    save_json(manifest_path, manifest)
                final_images.append(current)
                manifest["samples"].append(
                    {
                        "workflow": name,
                        "repeat": repeat,
                        "image": str(image),
                        "line_art": str(current),
                        "tuning_input_index": len(final_images) - 1,
                    }
                )
    manifest["status"] = "tuning"
    save_json(manifest_path, manifest)
    tuning_directory = artifacts.directory / "tuning"
    tuning_directory.mkdir()
    try:
        report, selected_path = tune_paths(
            final_images, config_path, tuning_path, Artifacts(tuning_directory, "tuning")
        )
    except (OSError, ValueError, TypeError, RuntimeError) as error:
        manifest.update(status="failed", error=str(error))
        save_json(manifest_path, manifest)
        raise
    manifest.update(status="complete", selected_config=str(selected_path))
    selected_rows = [
        row for row in report["rows"] if row["profile"] == report["selected"]["profile"]
    ]
    for sample, metrics in zip(manifest["samples"], selected_rows):
        sample["metrics"] = metrics
    save_json(manifest_path, manifest)
    print(f"回归完成：{manifest_path}", flush=True)
    return manifest
