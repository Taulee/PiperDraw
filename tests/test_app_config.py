import shutil

import pytest
import yaml

from piper_draw.app_config import load_app_config


@pytest.mark.parametrize(
    "values",
    [
        [],
        {"source": "photo", "prompt": "prompt.txt"},
        {"source": "unexpected", "prompt": "prompt.txt", "references": []},
        {"source": False, "prompt": "prompt.txt", "references": []},
        {"source": "photo", "prompt": None, "references": []},
        {"source": "photo", "prompt": "  ", "references": []},
        {"source": "photo", "prompt": "prompt.txt", "references": "image.png"},
        {"source": "photo", "prompt": "prompt.txt", "references": [1]},
        {"source": "photo", "prompt": "prompt.txt", "references": [""]},
        {"source": "photo", "prompt": "prompt.txt", "references": ["image.png"] * 10},
    ],
)
def test_invalid_app_configuration_is_rejected(tmp_path, values):
    path = tmp_path / "app.yaml"
    path.write_text(yaml.safe_dump(values), encoding="utf-8")
    with pytest.raises((TypeError, ValueError, KeyError)):
        load_app_config(path)


def test_relative_resources_follow_promoted_version_and_keep_reference_order(tmp_path):
    version = tmp_path / "cartoon_refined"
    (version / "config").mkdir(parents=True)
    (version / "input").mkdir()
    (version / "prompt.txt").write_text("漫画提示词", encoding="utf-8")
    (version / "input/example.png").write_bytes(b"first")
    (version / "input/second.jpg").write_bytes(b"second")
    (version / "config/app.yaml").write_text(
        "source: photo\nprompt: ../prompt.txt\n"
        "references: [../input/second.jpg, ../input/example.png]\n",
        encoding="utf-8",
    )
    before = load_app_config(version / "config/app.yaml")
    assert before.prompt.read_text(encoding="utf-8") == "漫画提示词"
    assert [path.read_bytes() for path in before.references] == [b"second", b"first"]

    for source in version.iterdir():
        shutil.move(str(source), str(tmp_path / source.name))
    version.rmdir()
    after = load_app_config(tmp_path / "config/app.yaml")

    assert after.source == before.source == "photo"
    assert after.prompt == tmp_path / "prompt.txt"
    assert after.references == (tmp_path / "input/second.jpg", tmp_path / "input/example.png")
    assert [path.read_bytes() for path in after.references] == [b"second", b"first"]


def test_additional_app_metadata_is_ignored(tmp_path):
    path = tmp_path / "app.yaml"
    path.write_text("source: photo\nprompt: prompt.txt\nreferences: []\nnotes: local experiment\n")
    assert load_app_config(path).source == "photo"
