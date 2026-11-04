"""下载锁定的官方源码；已有目录必须处于指定提交。"""
import json
import subprocess
from pathlib import Path

root = Path(__file__).resolve().parents[1]
for name, spec in json.loads((root / "vendor.lock").read_text()).items():
    path = root / "vendor" / name
    if not path.exists():
        subprocess.run(["git", "clone", spec["repository"], str(path)], check=True)
        subprocess.run(["git", "-C", str(path), "checkout", "--detach", spec["commit"]], check=True)
    actual = subprocess.check_output(["git", "-C", str(path), "rev-parse", "HEAD"], text=True).strip()
    if actual != spec["commit"]:
        raise RuntimeError(f"{path} 提交不符：{actual}，需要 {spec['commit']}")
    if subprocess.check_output(["git", "-C", str(path), "status", "--porcelain"], text=True).strip():
        raise RuntimeError(f"官方源码有本地修改：{path}")
