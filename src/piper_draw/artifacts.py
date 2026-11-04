from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo


def timestamp() -> str:
    return datetime.now(ZoneInfo("Asia/Shanghai")).strftime("%Y%m%d_%H%M%S_%f")


def timestamped_path(directory: Path, name: str, suffix: str) -> Path:
    return directory / f"{timestamp()}_{name}{suffix}"


@dataclass(frozen=True)
class Artifacts:
    """一次处理共用目录和时间戳，便于关联照片、线稿与任务。"""

    directory: Path
    timestamp: str

    @classmethod
    def create(cls, root: Path = Path("output")) -> "Artifacts":
        stamp = timestamp()
        directory = root / stamp
        directory.mkdir(parents=True, exist_ok=False)
        return cls(directory, stamp)

    def path(self, name: str, suffix: str) -> Path:
        return self.directory / f"{self.timestamp}_{name}{suffix}"
