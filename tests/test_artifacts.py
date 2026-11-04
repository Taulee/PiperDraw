from datetime import datetime

import pytest

from piper_draw import artifacts


def test_timestamp_uses_beijing_time(monkeypatch):
    class FixedDatetime(datetime):
        @classmethod
        def now(cls, timezone):
            assert timezone.key == "Asia/Shanghai"
            return cls(2026, 10, 8, 15, 30, 0, 123456, tzinfo=timezone)

    monkeypatch.setattr(artifacts, "datetime", FixedDatetime)
    assert artifacts.timestamp() == "20261008_153000_123456"


def test_same_timestamp_cannot_overwrite_existing_artifacts(tmp_path, monkeypatch):
    monkeypatch.setattr(artifacts, "timestamp", lambda: "20261008_153000_123456")
    files = artifacts.Artifacts.create(tmp_path)
    original = files.path("job", ".yaml")
    original.write_text("原有任务", encoding="utf-8")

    with pytest.raises(FileExistsError):
        artifacts.Artifacts.create(tmp_path)

    assert original.read_text(encoding="utf-8") == "原有任务"
