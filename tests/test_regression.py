import json
from pathlib import Path

import pytest
import yaml

from piper_draw import regression
from piper_draw.seedream import SeedreamAPIError, SeedreamConfig


@pytest.fixture
def environment(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    image = Path("photo.jpg")
    image.write_bytes(b"photo")
    Path("drawing.yaml").write_text("image: {}\n")
    Path("first.txt").write_text("照片转线稿", encoding="utf-8")
    Path("hair.txt").write_text("清理头发", encoding="utf-8")
    Path("details.txt").write_text("清理眉毛", encoding="utf-8")
    workflow = Path("workflow.yaml")
    workflow.write_text(
        yaml.safe_dump({"workflows": {"fixed": ["first.txt", "hair.txt", "details.txt"]}})
    )
    monkeypatch.setattr(regression, "load_seedream_config", lambda path: SeedreamConfig())
    monkeypatch.setattr(regression, "load_tuning_config", lambda path: {})
    return image, workflow


def test_fixed_stage_chain_repeats_from_photo_then_jointly_tunes(monkeypatch, environment):
    image, workflow = environment
    calls = []
    tuning_calls = []

    def generate(source, prompt, config, output):
        calls.append((source.read_bytes(), prompt))
        output.write_bytes(prompt.encode())
        return output

    def tune(images, config, tuning, artifacts):
        tuning_calls.append(images)
        rows = [{"profile": "selected", "input_index": index} for index in range(len(images))]
        return {"selected": {"profile": "selected"}, "rows": rows}, artifacts.path(
            "config", ".yaml"
        )

    monkeypatch.setattr(regression, "generate_line_art", generate)
    monkeypatch.setattr(regression, "tune_paths", tune)
    report = regression.regress(
        [image], workflow, Path("api.yaml"), Path("drawing.yaml"), Path("tuning.yaml"), repeats=2
    )
    assert (
        calls
        == [
            (b"photo", "照片转线稿"),
            ("照片转线稿".encode(), "清理头发"),
            ("清理头发".encode(), "清理眉毛"),
        ]
        * 2
    )
    assert len(tuning_calls) == 1 and len(tuning_calls[0]) == 2
    assert report["expected_api_calls"] == 6
    assert report["status"] == "complete"
    assert all(run["status"] == "generated" for run in report["runs"])


def test_api_failure_keeps_failed_stage_and_stops_without_retry(monkeypatch, environment):
    image, workflow = environment
    calls = []

    def generate(source, prompt, config, output):
        calls.append(prompt)
        if len(calls) == 2:
            raise SeedreamAPIError("HTTP 403：无权限")
        output.write_bytes(b"first line art")
        return output

    monkeypatch.setattr(regression, "generate_line_art", generate)
    with pytest.raises(SeedreamAPIError, match="无权限"):
        regression.regress(
            [image], workflow, Path("api.yaml"), Path("drawing.yaml"), Path("tuning.yaml")
        )
    assert len(calls) == 2
    manifest_path = next(Path("output").glob("*/*_regression.json"))
    manifest = json.loads(manifest_path.read_text())
    assert manifest["status"] == "failed"
    assert [run["status"] for run in manifest["runs"]] == ["generated", "failed"]
    assert Path(manifest["runs"][0]["output"]).read_bytes() == b"first line art"
    assert not Path(manifest["runs"][1]["output"]).exists()


def test_empty_prompt_fails_before_any_api_call(monkeypatch, environment):
    image, workflow = environment
    Path("hair.txt").write_text(" \n")
    with pytest.raises(ValueError, match="提示词不能为空"):
        regression.regress(
            [image], workflow, Path("api.yaml"), Path("drawing.yaml"), Path("tuning.yaml")
        )
    assert not Path("output").exists()


@pytest.mark.parametrize("fail_stage", [None, 2])
def test_each_stage_snapshots_and_sends_ordered_references(monkeypatch, environment, fail_stage):
    image, workflow = environment
    first = Path("first.PNG")
    first.write_bytes(b"first reference")
    second = Path("second.jpg")
    second.write_bytes(b"second reference")
    calls = []

    def generate(source, prompt, config, output, *, reference_images):
        calls.append(reference_images)
        assert [path.read_bytes() for path in reference_images] == [
            b"second reference",
            b"first reference",
        ]
        assert all(path.parent == source.parent for path in reference_images)
        if len(calls) == fail_stage:
            raise SeedreamAPIError("HTTP 403：参考图被拒绝")
        output.write_bytes(prompt.encode())
        return output

    def tune(images, config, tuning, artifacts):
        return {
            "selected": {"profile": "selected"},
            "rows": [{"profile": "selected"}],
        }, artifacts.path("config", ".yaml")

    monkeypatch.setattr(regression, "generate_line_art", generate)
    monkeypatch.setattr(regression, "tune_paths", tune)
    arguments = ([image], workflow, Path("api.yaml"), Path("drawing.yaml"), Path("tuning.yaml"))
    if fail_stage is not None:
        with pytest.raises(SeedreamAPIError, match="参考图被拒绝"):
            regression.regress(*arguments, reference_images=[second, first])
    else:
        regression.regress(*arguments, reference_images=[second, first])

    manifest = json.loads(next(Path("output").glob("*/*_regression.json")).read_text())
    assert manifest["status"] == ("complete" if fail_stage is None else "failed")
    assert len(calls) == (3 if fail_stage is None else 2)
    assert len({path for references in calls for path in references}) == len(calls) * 2
    for run, references in zip(manifest["runs"], calls):
        assert [record["source"] for record in run["references"]] == [
            str(second.resolve()),
            str(first.resolve()),
        ]
        assert [Path(record["snapshot"]) for record in run["references"]] == references
        assert [record["sha256"] for record in run["references"]] == [
            regression.file_digest(path) for path in references
        ]
    if fail_stage is not None:
        assert manifest["runs"][-1]["status"] == "failed"
        assert "参考图被拒绝" in manifest["runs"][-1]["error"]


def test_missing_reference_fails_before_any_api_call(monkeypatch, environment):
    image, workflow = environment
    with pytest.raises(FileNotFoundError):
        regression.regress(
            [image],
            workflow,
            Path("api.yaml"),
            Path("drawing.yaml"),
            Path("tuning.yaml"),
            reference_images=[Path("missing.png")],
        )
    assert not Path("output").exists()
