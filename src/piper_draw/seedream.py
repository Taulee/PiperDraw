"""独立调用 Seedream 图生图接口，以 PNG 文件衔接后续处理。"""

import base64
import binascii
import json
import math
import os
import re
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen

import cv2
import numpy as np
import yaml


class SeedreamAPIError(RuntimeError):
    """图像服务拒绝请求；保留服务端诊断供命令行显示。"""


@dataclass(frozen=True)
class SeedreamConfig:
    model: str = "doubao-seedream-5-0-flash-260915"
    base_url: str = "https://ark.cn-beijing.volces.com/api/v3"
    size: str = "1696x2400"
    timeout: float = 180.0
    api_key_env: str = "ARK_API_KEY"
    api_key_file: str = "api_key.txt"

    def __post_init__(self):
        for name in ("model", "base_url", "size", "api_key_file", "api_key_env"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"Seedream 配置 {name} 必须是非空字符串")
        address = urlsplit(self.base_url)
        if address.scheme not in ("http", "https") or not address.netloc:
            raise ValueError("Seedream base_url 必须是 HTTP 或 HTTPS 地址")
        if address.query or address.fragment:
            raise ValueError("Seedream base_url 不能包含查询参数或片段")
        if self.size not in ("1K", "1.5K", "2K", "3K", "4K") and not re.fullmatch(
            r"[1-9]\d*x[1-9]\d*", self.size
        ):
            raise ValueError("Seedream size 必须是分辨率档位或宽x高，例如 1696x2400")
        if (
            isinstance(self.timeout, bool)
            or not isinstance(self.timeout, (int, float))
            or not math.isfinite(self.timeout)
            or self.timeout <= 0
        ):
            raise ValueError("Seedream timeout 必须是有限的正数，单位为秒")


def load_seedream_config(path: Path) -> SeedreamConfig:
    values = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(values, dict):
        raise TypeError("Seedream 配置必须是 YAML 映射")
    return SeedreamConfig(**values)


def _load_api_key(config: SeedreamConfig) -> str:
    path = Path(config.api_key_file)
    source = f"API 密钥文件 {path}"
    try:
        api_key = path.read_text(encoding="utf-8").strip()
    except FileNotFoundError:
        api_key = os.environ.get(config.api_key_env, "").strip()
        source = f"环境变量 {config.api_key_env}"
        if not api_key:
            raise ValueError(
                f"请将 API 密钥写入 {path}，或设置环境变量 {config.api_key_env}"
            ) from None
    if not api_key:
        raise ValueError(f"{source} 为空，请填写 API 密钥")
    if "\n" in api_key or "\r" in api_key:
        raise ValueError(f"{source} 必须只包含一行密钥")
    return api_key


def _image_data_url(path: Path) -> str:
    formats = {
        ".png": "png",
        ".jpg": "jpeg",
        ".jpeg": "jpeg",
        ".webp": "webp",
        ".bmp": "bmp",
        ".tif": "tiff",
        ".tiff": "tiff",
    }
    image_format = formats.get(path.suffix.lower())
    if image_format is None:
        raise ValueError(f"不支持的输入图像格式：{path.suffix}")
    if path.stat().st_size > 30 * 1024 * 1024:
        raise ValueError("Seedream 输入图像不能超过 30 MB")
    data = path.read_bytes()
    signatures = {
        "png": data.startswith(b"\x89PNG\r\n\x1a\n"),
        "jpeg": data.startswith(b"\xff\xd8\xff"),
        "webp": data[:4] == b"RIFF" and data[8:12] == b"WEBP",
        "bmp": data.startswith(b"BM"),
        "tiff": data.startswith((b"II*\x00", b"MM\x00*", b"II+\x00", b"MM\x00+")),
    }
    if not signatures[image_format]:
        raise ValueError(f"输入图像内容与文件后缀不符：{path}")
    image = (
        cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_UNCHANGED) if data else None
    )
    if image is None:
        raise ValueError(f"无法读取输入图像：{path}")
    height, width = image.shape[:2]
    if (
        min(width, height) <= 14
        or width * height > 36_000_000
        or not 1 / 16 <= width / height <= 16
    ):
        raise ValueError(
            "Seedream 输入图像两边须大于 14 像素、总像素不超过 3600 万、宽高比在 1/16 到 16 之间"
        )
    encoded = base64.b64encode(data).decode("ascii")
    return f"data:image/{image_format};base64,{encoded}"


def _error_message(error) -> str:
    if isinstance(error, dict):
        parts = [str(error[name]) for name in ("code", "message") if error.get(name)]
        return ": ".join(parts) if parts else str(error)
    return str(error)


def _http_error_message(error: HTTPError, config: SeedreamConfig) -> str:
    detail = error.read().decode("utf-8", errors="replace")
    message = f"Seedream HTTP {error.code}："
    try:
        payload = json.loads(detail)
    except json.JSONDecodeError:
        return message + (detail or str(error.reason))
    api_error = payload.get("error", payload) if isinstance(payload, dict) else payload
    message += _error_message(api_error)
    if isinstance(api_error, dict) and api_error.get("code") in (
        "InvalidEndpointOrModel.NotFound",
        "ModelNotOpen",
        "InvalidEndpointOrModel.ModelIDAccessDisabled",
    ):
        message += (
            f"\n模型或接入点：{config.model}\nAPI 地址：{config.base_url}"
            "\n请核对生成配置中的 model、base_url，使用账号已开通且仍可用的模型服务，"
            "并确认当前 API Key 的账号与地域。"
            "账号限制 Model ID 调用时，可将 model 改为有访问权限的推理接入点 ID（ep-...）。"
        )
    return message


def generate_line_art(
    image_path: Path,
    prompt: str,
    config: SeedreamConfig,
    output_path: Path,
    *,
    reference_images: Sequence[Path] = (),
) -> Path:
    """发送主图与有序参考图，验证唯一返回图像后保存 PNG；失败不重试。"""
    if not isinstance(prompt, str) or not prompt.strip():
        raise ValueError("Seedream 提示词不能为空")
    if len(reference_images) > 9:
        raise ValueError("Seedream 主图与参考图合计不能超过 10 张")
    api_key = _load_api_key(config)
    images = [_image_data_url(image_path)]
    images.extend(_image_data_url(path) for path in reference_images)
    body = {
        "model": config.model,
        "prompt": prompt,
        "image": images if reference_images else images[0],
        "size": config.size,
        "response_format": "b64_json",
        "output_format": "png",
        "watermark": False,
    }
    # 仅发送三种 5.0 模型的共同参数；pro/flash 不支持组图控制参数。
    request = Request(
        config.base_url.rstrip("/") + "/images/generations",
        data=json.dumps(body, ensure_ascii=False).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    # 鉴权只交给配置的 API 地址，重定向时不转发密钥。
    request.add_unredirected_header("Authorization", f"Bearer {api_key}")
    try:
        with urlopen(request, timeout=config.timeout) as response:
            result = json.load(response)
    except HTTPError as error:
        raise SeedreamAPIError(_http_error_message(error, config)) from error
    except URLError as error:
        raise RuntimeError(f"Seedream 请求失败：{error.reason}") from error
    except TimeoutError as error:
        raise RuntimeError(f"Seedream 请求超时（{config.timeout} 秒）") from error
    except (json.JSONDecodeError, UnicodeDecodeError) as error:
        raise ValueError("Seedream 响应不是有效 JSON") from error
    if not isinstance(result, dict):
        raise TypeError("Seedream 响应必须是 JSON 对象")
    if "error" in result:
        raise SeedreamAPIError(f"Seedream 生成失败：{_error_message(result['error'])}")
    data = result.get("data")
    if not isinstance(data, list) or len(data) != 1 or not isinstance(data[0], dict):
        raise ValueError("Seedream 响应必须包含唯一图像")
    if "error" in data[0]:
        raise SeedreamAPIError(f"Seedream 图像生成失败：{_error_message(data[0]['error'])}")
    encoded = data[0].get("b64_json")
    if not isinstance(encoded, str) or not encoded:
        raise ValueError("Seedream 响应缺少 b64_json 图像数据")
    try:
        image_bytes = base64.b64decode(encoded, validate=True)
    except (binascii.Error, ValueError) as error:
        raise ValueError("Seedream 返回的图像不是有效 Base64") from error
    image = (
        cv2.imdecode(np.frombuffer(image_bytes, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
        if image_bytes
        else None
    )
    if image is None:
        raise ValueError("Seedream 返回的内容不是有效图像")
    saved, png = cv2.imencode(".png", image)
    if not saved:
        raise OSError("无法将 Seedream 返回图像编码为 PNG")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("xb") as output:
        output.write(png.tobytes())
    return output_path
