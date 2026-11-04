import base64
import io
import json
import shutil
import sys
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
import yaml

from piper_draw import cli, job, regression, seedream
from piper_draw.seedream import SeedreamAPIError

real_load_seedream_config = seedream.load_seedream_config
real_generate_line_art = seedream.generate_line_art


def test_launch_prefers_workspace_python_sources(monkeypatch):
    calls = []
    monkeypatch.setattr(cli.subprocess, "Popen", lambda arguments, **kwargs:
                        calls.append(arguments) or SimpleNamespace(wait=lambda: 0))
    cli.launch("bringup.launch.py", mode="real")
    expected = Path(cli.__file__).resolve().parents[2] / "ros2_ws/src/piper_drawing/launch/bringup.launch.py"
    assert calls == [["ros2", "launch", str(expected), "mode:=real"]]


def test_launch_cancels_and_waits_for_executor_before_returning(monkeypatch):
    events = []

    def wait(**kwargs):
        if not kwargs:
            raise KeyboardInterrupt
        events.append("executor_exited")
        return -2

    process = SimpleNamespace(pid=98765, wait=wait, poll=lambda: None)
    monkeypatch.setattr(cli.subprocess, "Popen", lambda *args, **kwargs: process)
    monkeypatch.setattr(cli.os, "killpg", lambda *args: events.append("cancel_executor"))
    with pytest.raises(KeyboardInterrupt):
        cli.launch("draw.launch.py")
    assert events == ["cancel_executor", "executor_exited"]


def invoke(monkeypatch, *arguments):
    monkeypatch.setattr(sys, "argv", ["piper-draw", *map(str, arguments)])
    cli.main()


@pytest.fixture
def environment(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    image = Path("photo.PNG")
    image.write_bytes(b"original photo")
    Path("config").mkdir()
    Path("config/app.yaml").write_text(
        "source: photo\nprompt: ../prompt.txt\nreferences: []\n", encoding="utf-8"
    )
    Path("config/sim.yaml").write_text("calibrated: false\n", encoding="utf-8")
    seedream_text = "model: seedream-test\napi_key_env: TEST_API_KEY\n"
    Path("config/seedream.yaml").write_text(seedream_text, encoding="utf-8")
    prompt = "将原图转换为白底黑色线稿，保持主体轮廓。"
    Path("prompt.txt").write_text(prompt, encoding="utf-8")
    state = SimpleNamespace(
        image=image,
        prompt=prompt,
        seedream_text=seedream_text,
        api_config=object(),
        events=[],
        generations=[],
        preparations=[],
        loaded_configs=[],
    )

    def load_seedream_config(path):
        state.loaded_configs.append(path)
        return state.api_config

    def generate_line_art(image_path, prompt, config, output_path):
        state.events.append("generate")
        state.generations.append((image_path, prompt, config, output_path))
        output_path.write_bytes(b"generated line art")
        return output_path

    def prepare(image_path, config_path, artifacts=None):
        artifacts = artifacts if artifacts is not None else cli.Artifacts.create()
        state.events.append("prepare")
        state.preparations.append((image_path, config_path, artifacts))
        artifacts.path("job", ".yaml").write_text("segments: []\n", encoding="utf-8")
        artifacts.path("preview", ".svg").write_text("<svg/>\n", encoding="utf-8")
        artifacts.path("skeleton", ".png").write_bytes(b"skeleton")

    monkeypatch.setattr(seedream, "load_seedream_config", load_seedream_config)
    monkeypatch.setattr(seedream, "generate_line_art", generate_line_art)
    monkeypatch.setattr(job, "prepare", prepare)
    return state


def assert_generation_snapshots(directory, environment):
    prefix = directory.name
    source = directory / f"{prefix}_source.png"
    assert source.read_bytes() == environment.image.read_bytes()
    assert (directory / f"{prefix}_prompt.txt").read_text(encoding="utf-8") == environment.prompt
    assert (directory / f"{prefix}_seedream.yaml").read_text(
        encoding="utf-8"
    ) == environment.seedream_text
    assert all(path.name.startswith(f"{prefix}_") for path in directory.iterdir())
    return source


def test_prepare_from_line_art_is_offline(monkeypatch, environment, tmp_path):
    invoke(monkeypatch, "prepare", environment.image, "--from-line-art")

    assert environment.events == ["prepare"]
    image, config, artifacts = environment.preparations[0]
    assert image == environment.image
    assert config == Path("config/sim.yaml")
    assert artifacts.directory.resolve().parent == tmp_path / "output"
    assert artifacts.timestamp == artifacts.directory.name
    assert artifacts.path("job", ".yaml").is_file()
    assert environment.loaded_configs == []


@pytest.mark.parametrize("source_options", [(), ("--from-photo",)])
def test_prepare_photo_shares_timestamp_and_directory(monkeypatch, environment, source_options):
    invoke(monkeypatch, "prepare", environment.image, *source_options)

    assert environment.events == ["generate", "prepare"]
    source, prompt, config, line_art = environment.generations[0]
    prepared_image, drawing_config, artifacts = environment.preparations[0]
    directory = artifacts.directory
    assert source == assert_generation_snapshots(directory, environment)
    assert prompt == environment.prompt
    assert config is environment.api_config
    assert environment.loaded_configs == [Path("config/seedream.yaml")]
    assert line_art == artifacts.path("line_art", ".png")
    assert line_art.read_bytes() == b"generated line art"
    assert prepared_image == line_art
    assert drawing_config == Path("config/sim.yaml")
    for name, suffix in (("job", ".yaml"), ("preview", ".svg"), ("skeleton", ".png")):
        assert artifacts.path(name, suffix).is_file()
    assert list(Path("output").iterdir()) == [directory]


def test_line_art_only_generates_image(monkeypatch, environment):
    invoke(monkeypatch, "line-art", environment.image)

    assert environment.events == ["generate"]
    assert environment.preparations == []
    source, _, _, line_art = environment.generations[0]
    directory = line_art.parent
    assert source == assert_generation_snapshots(directory, environment)
    assert line_art.name == f"{directory.name}_line_art.png"
    assert line_art.read_bytes() == b"generated line art"
    assert not list(directory.glob("*_job.yaml"))


@pytest.mark.parametrize("command", ["line-art", "prepare"])
def test_photo_commands_snapshot_reference_images_in_order(monkeypatch, environment, command):
    first_reference = Path("first.PNG")
    first_reference.write_bytes(b"first reference")
    second_reference = Path("second.jpg")
    second_reference.write_bytes(b"second reference")
    received_references = []

    def generate_with_references(image_path, prompt, config, output_path, *, reference_images):
        environment.events.append("generate")
        environment.generations.append((image_path, prompt, config, output_path))
        received_references.extend(reference_images)
        output_path.write_bytes(b"generated line art")
        return output_path

    monkeypatch.setattr(seedream, "generate_line_art", generate_with_references)
    arguments = [
        command,
        environment.image,
        "--reference",
        first_reference,
        "--reference",
        second_reference,
    ]
    if command == "prepare":
        arguments.append("--from-photo")

    invoke(monkeypatch, *arguments)

    source, _, _, line_art = environment.generations[0]
    directory = line_art.parent
    assert source == assert_generation_snapshots(directory, environment)
    assert received_references == [
        directory / f"{directory.name}_reference_1.png",
        directory / f"{directory.name}_reference_2.jpg",
    ]
    for snapshot, reference in zip(received_references, (first_reference, second_reference)):
        assert snapshot.read_bytes() == reference.read_bytes()
        assert snapshot != reference
    assert environment.image.read_bytes() == b"original photo"
    assert first_reference.read_bytes() == b"first reference"
    assert second_reference.read_bytes() == b"second reference"
    assert environment.events == (["generate", "prepare"] if command == "prepare" else ["generate"])


def test_prepare_rejects_reference_images_without_photo_generation(monkeypatch, environment):
    with pytest.raises(SystemExit) as error:
        invoke(
            monkeypatch,
            "prepare",
            environment.image,
            "--from-line-art",
            "--reference",
            environment.image,
        )

    assert error.value.code == 2
    assert environment.events == []
    assert environment.loaded_configs == []
    assert not Path("output").exists()


def test_prepare_source_configuration_and_explicit_photo_override(monkeypatch, environment):
    Path("config/app.yaml").write_text(
        "source: line_art\nprompt: ../prompt.txt\nreferences: []\n", encoding="utf-8"
    )

    invoke(monkeypatch, "prepare", environment.image)
    assert environment.events == ["prepare"]
    invoke(monkeypatch, "prepare", environment.image, "--from-photo")
    assert environment.events == ["prepare", "generate", "prepare"]


@pytest.mark.parametrize("command", ["line-art", "prepare"])
@pytest.mark.parametrize("override", [False, True])
def test_configured_prompt_and_ordered_references_support_explicit_overrides(
    monkeypatch, environment, command, override
):
    Path("input").mkdir()
    first_reference = Path("input/first.PNG")
    first_reference.write_bytes(b"first reference")
    second_reference = Path("input/second.jpg")
    second_reference.write_bytes(b"second reference")
    Path("config/style.txt").write_text("版本提示词", encoding="utf-8")
    Path("config/app.yaml").write_text(
        yaml.safe_dump(
            {
                "source": "photo",
                "prompt": "style.txt",
                "references": ["../input/second.jpg", "../input/first.PNG"],
            }
        ),
        encoding="utf-8",
    )
    received = []

    def generate(image_path, prompt, config, output_path, *, reference_images):
        received.append((prompt, [reference.read_bytes() for reference in reference_images]))
        output_path.write_bytes(b"generated line art")
        return output_path

    monkeypatch.setattr(seedream, "generate_line_art", generate)
    arguments = [command, environment.image]
    if override:
        arguments.extend(["--prompt", "prompt.txt", "--reference", first_reference])
    invoke(monkeypatch, *arguments)

    assert received == [
        (environment.prompt, [b"first reference"])
        if override
        else ("版本提示词", [b"second reference", b"first reference"])
    ]


def test_offline_override_ignores_default_references(monkeypatch, environment):
    Path("config/app.yaml").write_text(
        "source: photo\nprompt: ../prompt.txt\nreferences: [../missing.png]\n",
        encoding="utf-8",
    )
    invoke(monkeypatch, "prepare", environment.image, "--from-line-art")
    assert environment.events == ["prepare"]
    assert environment.loaded_configs == []


def test_default_photo_entrypoint_remains_usable_after_directory_move(
    monkeypatch, environment, tmp_path
):
    project = tmp_path / "moved_project"
    version = project / "original_lineart"
    version.mkdir(parents=True)
    for source in (Path("config"), Path("prompt.txt"), environment.image):
        destination = version / source.name
        if source.is_dir():
            shutil.copytree(source, destination)
        else:
            shutil.copyfile(source, destination)
    monkeypatch.chdir(version)
    invoke(monkeypatch, "prepare", environment.image)
    for source in list(version.iterdir()):
        shutil.move(str(source), str(project / source.name))
    version.rmdir()
    monkeypatch.chdir(project)
    invoke(monkeypatch, "prepare", environment.image)

    assert environment.events == ["generate", "prepare", "generate", "prepare"]
    assert [row[1] for row in environment.generations] == [environment.prompt] * 2
    assert len(list(Path("output").iterdir())) == 2


@pytest.mark.parametrize("override", [False, True])
def test_regress_uses_configured_reference_list_or_explicit_override(
    monkeypatch, environment, override
):
    Path("config/app.yaml").write_text(
        "source: photo\nprompt: ../prompt.txt\nreferences: [../configured.png]\n",
        encoding="utf-8",
    )
    calls = []

    def regress(*arguments, **options):
        calls.append((arguments, options))

    monkeypatch.setattr(regression, "regress", regress)
    arguments = ["regress", environment.image]
    if override:
        arguments.extend(["--reference", "explicit.jpg"])
    invoke(monkeypatch, *arguments)

    assert calls[0][0][0] == [environment.image]
    assert calls[0][1]["reference_images"] == (
        [Path("explicit.jpg")] if override else (Path("configured.png").resolve(),)
    )


def test_missing_reference_image_fails_before_generation(monkeypatch, environment):
    with pytest.raises(FileNotFoundError):
        invoke(monkeypatch, "line-art", environment.image, "--reference", "missing.png")

    assert environment.generations == []
    assert environment.preparations == []
    assert environment.image.read_bytes() == b"original photo"


@pytest.mark.parametrize("command", ["line-art", "prepare"])
def test_photo_commands_use_local_key_without_copying_credentials(
    monkeypatch, environment, capsys, command
):
    image = np.full((32, 24, 3), 255, dtype=np.uint8)
    cv2.line(image, (5, 5), (18, 26), (0, 0, 0), 2)
    success, encoded = cv2.imencode(".png", image)
    assert success
    image_bytes = encoded.tobytes()
    environment.image.write_bytes(image_bytes)
    environment.seedream_text = (
        "model: seedream-test\napi_key_file: api_key.txt\napi_key_env: TEST_API_KEY\n"
    )
    Path("config/seedream.yaml").write_text(environment.seedream_text, encoding="utf-8")
    secret = "private-local-cli-key"
    key_path = Path("api_key.txt")
    key_path.write_text(secret + "\n", encoding="utf-8")
    monkeypatch.delenv("TEST_API_KEY", raising=False)
    monkeypatch.setattr(seedream, "load_seedream_config", real_load_seedream_config)
    monkeypatch.setattr(seedream, "generate_line_art", real_generate_line_art)
    requests = []

    def respond(request, *, timeout):
        requests.append(request)
        payload = {"data": [{"b64_json": base64.b64encode(image_bytes).decode("ascii")}]}
        return io.BytesIO(json.dumps(payload).encode("utf-8"))

    monkeypatch.setattr(seedream, "urlopen", respond)
    arguments = [command, environment.image]
    if command == "prepare":
        arguments.append("--from-photo")

    invoke(monkeypatch, *arguments)

    assert len(requests) == 1
    assert requests[0].get_header("Authorization") == f"Bearer {secret}"
    directories = list(Path("output").iterdir())
    assert len(directories) == 1
    directory = directories[0]
    assert_generation_snapshots(directory, environment)
    assert (directory / f"{directory.name}_line_art.png").read_bytes() == image_bytes
    assert bool(list(directory.glob("*_job.yaml"))) is (command == "prepare")
    assert not list(directory.glob("*api_key*"))
    assert all(secret.encode("utf-8") not in path.read_bytes() for path in directory.iterdir())
    captured = capsys.readouterr()
    assert secret not in captured.out + captured.err
    assert key_path.read_text(encoding="utf-8") == secret + "\n"


def test_photo_generation_failure_keeps_inputs_and_skips_prepare(monkeypatch, environment):
    def fail_generation(*arguments):
        raise RuntimeError("Seedream 请求失败")

    monkeypatch.setattr(seedream, "generate_line_art", fail_generation)
    with pytest.raises(RuntimeError, match="Seedream 请求失败"):
        invoke(monkeypatch, "prepare", environment.image, "--from-photo")

    assert environment.preparations == []
    directories = list(Path("output").iterdir())
    assert len(directories) == 1
    assert_generation_snapshots(directories[0], environment)
    assert not list(directories[0].glob("*_line_art.png"))
    assert not list(directories[0].glob("*_job.yaml"))


@pytest.mark.parametrize("command", ["prepare", "line-art"])
def test_api_rejection_exits_with_diagnostic_and_keeps_snapshots(monkeypatch, environment, command):
    config_path = Path("config/custom-seedream.yaml")
    config_path.write_text(environment.seedream_text, encoding="utf-8")
    diagnostic = (
        "Seedream HTTP 404：InvalidEndpointOrModel.NotFound。"
        "Request id: model-access-request；请检查 API Key 权限。"
    )
    calls = []

    def fail_generation(*arguments):
        calls.append(arguments)
        raise SeedreamAPIError(diagnostic)

    monkeypatch.setattr(seedream, "generate_line_art", fail_generation)
    arguments = [command, environment.image, "--seedream-config", config_path]
    if command == "prepare":
        arguments.append("--from-photo")

    with pytest.raises(SystemExit) as error:
        invoke(monkeypatch, *arguments)

    assert error.value.code == f"{diagnostic}\n生成配置：{config_path}"
    assert error.value.__suppress_context__ is True
    assert len(calls) == 1
    assert environment.loaded_configs == [config_path]
    assert environment.preparations == []
    directories = list(Path("output").iterdir())
    assert len(directories) == 1
    assert_generation_snapshots(directories[0], environment)
    assert not list(directories[0].glob("*_line_art.png"))
    assert not list(directories[0].glob("*_job.yaml"))


def test_missing_drawing_config_is_rejected_before_photo_generation(monkeypatch, environment):
    Path("config/sim.yaml").unlink()
    with pytest.raises(FileNotFoundError):
        invoke(monkeypatch, "prepare", environment.image, "--from-photo")
    assert environment.generations == []
    assert environment.preparations == []


def test_prepare_rejects_obsolete_output_option(monkeypatch, environment):
    with pytest.raises(SystemExit) as error:
        invoke(monkeypatch, "prepare", environment.image, "--output", "elsewhere")
    assert error.value.code == 2
    assert environment.events == []


@pytest.mark.parametrize(
    ("command", "execute", "mode", "report_kind"),
    [("plan", False, "mock", "plan_report"), ("run", True, "real", "execute_report")],
)
def test_run_defaults_to_absolute_report_beside_job(
    tmp_path, monkeypatch, command, execute, mode, report_kind
):
    monkeypatch.chdir(tmp_path)
    Path("config").mkdir()
    Path("config/device.yaml").write_text("can: can0\nprotocol: v3\ndh_version: new\nlock_dir: locks\n")
    directory = Path("output/prepared")
    directory.mkdir(parents=True)
    job = directory / "drawing_job.yaml"
    job.write_text("segments: []\n", encoding="utf-8")
    calls = []

    def launch(file, **parameters):
        calls.append((file, parameters))
        report = parameters["report"]
        assert isinstance(report, Path) and report.is_absolute()
        assert report.parent == job.resolve().parent
        assert report.name.endswith(f"_{report_kind}.yaml")
        assert report.read_text(encoding="utf-8") == ""
        report.write_text(
            "status: executed\n" if execute else "status: planned\n", encoding="utf-8"
        )

    monkeypatch.setattr(cli, "launch", launch)
    invoke(monkeypatch, command, job, "--mode", mode)

    assert len(calls) == 1
    file, parameters = calls[0]
    assert file == "draw.launch.py"
    assert parameters["job"] == job.resolve()
    assert parameters["mode"] == mode
    assert parameters["execute"] is execute


def test_run_failed_report_exits_with_error(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    job = Path("drawing_job.yaml")
    job.write_text("segments: []\n", encoding="utf-8")

    def launch(file, **parameters):
        parameters["report"].write_text("status: failed\nerror: 轨迹规划失败\n", encoding="utf-8")

    monkeypatch.setattr(cli, "launch", launch)
    with pytest.raises(SystemExit, match="轨迹规划失败"):
        invoke(monkeypatch, "run", job)
    assert len(list(tmp_path.glob("*_execute_report.yaml"))) == 1


def test_repeated_runs_preserve_both_reports(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    job = Path("drawing_job.yaml")
    job.write_text("segments: []\n", encoding="utf-8")
    reports = []

    def launch(file, **parameters):
        report = parameters["report"]
        assert report.read_text(encoding="utf-8") == ""
        reports.append(report)
        report.write_text(
            yaml.safe_dump({"status": "planned", "run": len(reports)}), encoding="utf-8"
        )

    monkeypatch.setattr(cli, "launch", launch)
    invoke(monkeypatch, "run", job)
    invoke(monkeypatch, "run", job)

    assert len(set(reports)) == 2
    assert [yaml.safe_load(report.read_text(encoding="utf-8"))["run"] for report in reports] == [
        1,
        2,
    ]
    assert all(report.parent == job.resolve().parent for report in reports)


def test_run_honors_explicit_report_path(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    job = Path("drawing_job.yaml")
    job.write_text("segments: []\n", encoding="utf-8")
    report = Path("reports/custom.yaml")

    def launch(file, **parameters):
        assert parameters["report"] == report.resolve()
        assert report.parent.is_dir()
        parameters["report"].write_text("status: planned\n", encoding="utf-8")

    monkeypatch.setattr(cli, "launch", launch)
    invoke(monkeypatch, "run", job, "--report", report)
    assert report.is_file()


def test_report_timestamp_collision_preserves_existing_report(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    job = Path("drawing_job.yaml")
    job.write_text("segments: []\n", encoding="utf-8")
    report = tmp_path / "fixed_plan_report.yaml"
    report.write_text("status: planned\nrun: previous\n", encoding="utf-8")
    monkeypatch.setattr(cli, "timestamped_path", lambda *arguments: report)

    def launch(*arguments, **parameters):
        pytest.fail("时间戳碰撞时不能启动机器人任务")

    monkeypatch.setattr(cli, "launch", launch)
    with pytest.raises(FileExistsError):
        invoke(monkeypatch, "run", job)
    assert report.read_text(encoding="utf-8") == "status: planned\nrun: previous\n"


@pytest.mark.parametrize("report_text", ["", "status: success\n", "[]\n"])
def test_run_rejects_empty_or_invalid_backend_report(tmp_path, monkeypatch, report_text):
    monkeypatch.chdir(tmp_path)
    job = Path("drawing_job.yaml")
    job.write_text("segments: []\n", encoding="utf-8")

    def launch(file, **parameters):
        parameters["report"].write_text(report_text, encoding="utf-8")

    monkeypatch.setattr(cli, "launch", launch)
    with pytest.raises(RuntimeError, match="绘画后端未生成有效报告"):
        invoke(monkeypatch, "run", job)


def test_bringup_preserves_mode_config_and_can_protocol(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    config = Path("custom.yaml")
    config.write_text("calibrated: true\n", encoding="utf-8")
    calls = []

    def launch(file, **parameters):
        calls.append((file, parameters))

    monkeypatch.setattr(cli, "launch", launch)
    invoke(monkeypatch, "bringup", "--mode", "real", "--config", config, "--can", "can_piper", "--protocol", "v3")
    assert calls == [
        (
            "bringup.launch.py",
            {"mode": "real", "config": config.resolve(),
             "device_config": (tmp_path / "config/device.yaml").resolve(),
             "can_port": "can_piper", "protocol": "v3"},
        )
    ]
