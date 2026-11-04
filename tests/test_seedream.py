import base64
import io
import json
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import HTTPRedirectHandler

import cv2
import numpy as np
import pytest
import yaml

from piper_draw import seedream
from piper_draw.seedream import (
    SeedreamAPIError,
    SeedreamConfig,
    generate_line_art,
    load_seedream_config,
)


@pytest.fixture(autouse=True)
def isolated_working_directory(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)


@pytest.fixture
def image_bytes():
    image = np.full((32, 24, 3), 255, dtype=np.uint8)
    cv2.line(image, (5, 5), (18, 26), (0, 0, 0), 2)
    success, encoded = cv2.imencode(".png", image)
    assert success
    return encoded.tobytes()


@pytest.fixture
def image_path(tmp_path, image_bytes):
    path = tmp_path / "photo.png"
    path.write_bytes(image_bytes)
    return path


@pytest.fixture
def response_payload(image_bytes):
    return {"data": [{"b64_json": base64.b64encode(image_bytes).decode("ascii")}]}


@pytest.fixture
def mock_http(monkeypatch, response_payload):
    calls = []

    def respond(request, *, timeout):
        calls.append((request, timeout))
        return io.BytesIO(json.dumps(response_payload).encode("utf-8"))

    monkeypatch.setattr(seedream, "urlopen", respond)
    monkeypatch.setenv("ARK_API_KEY", "test-api-key")
    return calls


def test_request_uploads_photo_and_saves_valid_png(tmp_path, image_path, mock_http):
    output = tmp_path / "results" / "line_art.png"
    config = SeedreamConfig()

    result = generate_line_art(image_path, "转换为简洁黑白线稿", config, output)

    assert result == output
    image = cv2.imread(str(result), cv2.IMREAD_UNCHANGED)
    assert image is not None and image.shape[:2] == (32, 24)
    assert output.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    assert len(mock_http) == 1
    request, timeout = mock_http[0]
    assert request.full_url == "https://ark.cn-beijing.volces.com/api/v3/images/generations"
    assert request.method == "POST"
    assert request.get_header("Authorization") == "Bearer test-api-key"
    assert request.get_header("Content-type") == "application/json"
    assert timeout == 180.0
    body = json.loads(request.data)
    assert body["model"] == "doubao-seedream-5-0-flash-260915"
    assert body["prompt"] == "转换为简洁黑白线稿"
    assert body["size"] == "1696x2400"
    assert body["response_format"] == "b64_json"
    assert body["output_format"] == "png"
    assert body["watermark"] is False
    media_type, encoded = body["image"].split(",", maxsplit=1)
    assert media_type == "data:image/png;base64"
    uploaded = cv2.imdecode(np.frombuffer(base64.b64decode(encoded), np.uint8), -1)
    assert uploaded is not None and uploaded.shape[:2] == (32, 24)


def test_jpeg_api_image_is_saved_as_png(tmp_path, image_path, monkeypatch):
    image = np.full((32, 24, 3), 255, dtype=np.uint8)
    success, encoded = cv2.imencode(".jpg", image)
    assert success
    payload = {"data": [{"b64_json": base64.b64encode(encoded).decode("ascii")}]}
    monkeypatch.setenv("ARK_API_KEY", "test-key")
    monkeypatch.setattr(
        seedream,
        "urlopen",
        lambda request, *, timeout: io.BytesIO(json.dumps(payload).encode("utf-8")),
    )
    output = tmp_path / "line.png"

    generate_line_art(image_path, "线稿", SeedreamConfig(), output)

    assert output.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    assert cv2.imread(str(output)).shape[:2] == (32, 24)


def test_reference_images_are_uploaded_after_main_image_in_order(
    tmp_path, image_path, mock_http
):
    first_reference = tmp_path / "first.png"
    assert cv2.imwrite(str(first_reference), np.full((26, 30, 3), 192, dtype=np.uint8))
    second_reference = tmp_path / "second.jpg"
    image = np.full((28, 36, 3), 255, dtype=np.uint8)
    assert cv2.imwrite(str(second_reference), image)

    generate_line_art(
        image_path,
        "参考图一保留神态，简化主图",
        SeedreamConfig(),
        tmp_path / "line.png",
        reference_images=(first_reference, second_reference),
    )

    assert len(mock_http) == 1
    body = json.loads(mock_http[0][0].data)
    assert isinstance(body["image"], list)
    assert len(body["image"]) == 3
    for uploaded, path in zip(body["image"], (image_path, first_reference, second_reference)):
        encoded = uploaded.split(",", maxsplit=1)[1]
        assert base64.b64decode(encoded) == path.read_bytes()


def test_ten_total_images_are_accepted(tmp_path, image_path, mock_http):
    generate_line_art(
        image_path,
        "线稿",
        SeedreamConfig(),
        tmp_path / "line.png",
        reference_images=[image_path] * 9,
    )

    assert len(mock_http) == 1
    assert len(json.loads(mock_http[0][0].data)["image"]) == 10


def test_too_many_reference_images_fail_before_http(tmp_path, image_path, mock_http):
    output = tmp_path / "line.png"

    with pytest.raises(ValueError, match="合计不能超过 10 张"):
        generate_line_art(
            image_path,
            "线稿",
            SeedreamConfig(),
            output,
            reference_images=[image_path] * 10,
        )

    assert mock_http == []
    assert not output.exists()


@pytest.mark.parametrize("reference_kind", ["missing", "broken", "wrong_suffix"])
def test_invalid_reference_image_fails_before_http(
    tmp_path, image_path, image_bytes, mock_http, reference_kind
):
    reference = tmp_path / "reference.png"
    if reference_kind == "broken":
        reference.write_bytes(b"invalid image")
    elif reference_kind == "wrong_suffix":
        reference = reference.with_suffix(".jpg")
        reference.write_bytes(image_bytes)
    output = tmp_path / "line.png"

    with pytest.raises((FileNotFoundError, ValueError)):
        generate_line_art(
            image_path, "线稿", SeedreamConfig(), output, reference_images=(reference,)
        )

    assert mock_http == []
    assert not output.exists()


@pytest.mark.parametrize(
    "model",
    [
        "doubao-seedream-5-0-lite-260128",
        "doubao-seedream-5-0-260128",
        "doubao-seedream-5-0-pro-260628",
        "doubao-seedream-5-0-flash-260915",
    ],
)
def test_model_config_uses_common_single_image_parameters(tmp_path, image_path, mock_http, model):
    generate_line_art(image_path, "线稿", SeedreamConfig(model=model), tmp_path / "line.png")

    body = json.loads(mock_http[0][0].data)
    assert body["model"] == model
    assert body["size"] == "1696x2400"
    assert "sequential_image_generation" not in body
    assert "stream" not in body
    assert "seed" not in body


def test_custom_base_url_timeout_and_key_environment(tmp_path, image_path, mock_http, monkeypatch):
    monkeypatch.setenv("DRAWING_API_KEY", "custom-key")
    config = SeedreamConfig(
        base_url="https://example.com/custom/v3/", timeout=42.0, api_key_env="DRAWING_API_KEY"
    )

    generate_line_art(image_path, "线稿", config, tmp_path / "line.png")

    request, timeout = mock_http[0]
    assert request.full_url == "https://example.com/custom/v3/images/generations"
    assert request.get_header("Authorization") == "Bearer custom-key"
    assert timeout == 42.0


@pytest.mark.parametrize("environment_key", [None, "environment-api-key"])
def test_local_api_key_is_trimmed_and_preferred(
    tmp_path, image_path, mock_http, monkeypatch, environment_key
):
    Path("api_key.txt").write_text(" \n\t local-api-key \r\n", encoding="utf-8")
    if environment_key is None:
        monkeypatch.delenv("ARK_API_KEY", raising=False)
    else:
        monkeypatch.setenv("ARK_API_KEY", environment_key)

    generate_line_art(image_path, "线稿", SeedreamConfig(), tmp_path / "line.png")

    assert len(mock_http) == 1
    assert mock_http[0][0].get_header("Authorization") == "Bearer local-api-key"


@pytest.mark.parametrize("absolute", [False, True])
def test_custom_api_key_file_path(tmp_path, image_path, mock_http, absolute):
    path = Path("credentials") / "drawing.txt"
    path.parent.mkdir()
    path.write_text("custom-file-key\n", encoding="utf-8")
    configured_path = str(path.resolve() if absolute else path)
    config = SeedreamConfig(api_key_file=configured_path)

    generate_line_art(image_path, "线稿", config, tmp_path / "line.png")

    assert mock_http[0][0].get_header("Authorization") == "Bearer custom-file-key"


def test_missing_api_key_file_falls_back_to_environment(
    tmp_path, image_path, mock_http, monkeypatch
):
    monkeypatch.setenv("DRAWING_API_KEY", " \t fallback-api-key \n")
    config = SeedreamConfig(api_key_file="missing-key.txt", api_key_env="DRAWING_API_KEY")

    generate_line_art(image_path, "线稿", config, tmp_path / "line.png")

    assert len(mock_http) == 1
    assert mock_http[0][0].get_header("Authorization") == "Bearer fallback-api-key"


@pytest.mark.parametrize(
    "content",
    [
        "",
        " \n\t",
        "private-key-first\nprivate-key-second",
        "private-key-first\r\nprivate-key-second",
    ],
)
def test_invalid_local_api_key_does_not_fall_back_or_leak(
    tmp_path, image_path, mock_http, capsys, content
):
    Path("api_key.txt").write_text(content, encoding="utf-8")
    output = tmp_path / "line.png"

    with pytest.raises(ValueError) as error:
        generate_line_art(image_path, "线稿", SeedreamConfig(), output)

    captured = capsys.readouterr()
    diagnostic = str(error.value) + captured.out + captured.err
    assert "private-key-first" not in diagnostic
    assert "private-key-second" not in diagnostic
    assert "test-api-key" not in diagnostic
    assert mock_http == []
    assert not output.exists()


def test_api_key_directory_does_not_fall_back(tmp_path, image_path, mock_http):
    Path("api_key.txt").mkdir()
    output = tmp_path / "line.png"

    with pytest.raises(IsADirectoryError):
        generate_line_art(image_path, "线稿", SeedreamConfig(), output)

    assert mock_http == []
    assert not output.exists()


@pytest.mark.parametrize("error", [PermissionError("密钥文件不可读"), OSError("密钥文件读取失败")])
def test_api_key_read_error_does_not_fall_back(tmp_path, image_path, mock_http, monkeypatch, error):
    key_path = Path("api_key.txt")
    key_path.write_text("private-file-key", encoding="utf-8")
    read_text = Path.read_text

    def fail_key_read(path, *arguments, **options):
        if path == key_path:
            raise error
        return read_text(path, *arguments, **options)

    monkeypatch.setattr(Path, "read_text", fail_key_read)
    output = tmp_path / "line.png"

    with pytest.raises(type(error)) as raised:
        generate_line_art(image_path, "线稿", SeedreamConfig(), output)

    assert "private-file-key" not in str(raised.value)
    assert "test-api-key" not in str(raised.value)
    assert mock_http == []
    assert not output.exists()


def test_http_redirect_does_not_forward_api_key(tmp_path, image_path, mock_http):
    generate_line_art(image_path, "线稿", SeedreamConfig(), tmp_path / "line.png")
    request = mock_http[0][0]

    redirected = HTTPRedirectHandler().redirect_request(
        request, None, 302, "Found", {}, "https://other.example.com/images/generations"
    )

    assert request.get_header("Authorization") == "Bearer test-api-key"
    assert redirected.get_header("Authorization") is None


def test_yaml_config_loads_all_fields(tmp_path):
    path = tmp_path / "seedream.yaml"
    settings = {
        "model": "doubao-seedream-5-0-flash-260915",
        "base_url": "https://example.com/api/v3",
        "size": "2K",
        "timeout": 75.0,
        "api_key_file": "credentials/ark.txt",
        "api_key_env": "CUSTOM_KEY",
    }
    path.write_text(yaml.safe_dump(settings), encoding="utf-8")

    config = load_seedream_config(path)

    assert config == SeedreamConfig(**settings)


def test_yaml_config_defaults_to_recommended_pro_model(tmp_path):
    path = tmp_path / "seedream.yaml"
    path.write_text("{}\n", encoding="utf-8")

    config = load_seedream_config(path)

    assert config.model == "doubao-seedream-5-0-flash-260915"
    assert config.api_key_file == "api_key.txt"


@pytest.mark.parametrize("api_key_file", [None, False, 123, [], {}, "", " \t"])
def test_invalid_api_key_file_config_is_rejected(tmp_path, api_key_file):
    path = tmp_path / "seedream.yaml"
    path.write_text(yaml.safe_dump({"api_key_file": api_key_file}), encoding="utf-8")

    with pytest.raises(ValueError, match="api_key_file"):
        load_seedream_config(path)


@pytest.mark.parametrize("content", ["[]", "null", "not-a-mapping", "unknown_option: 1", "[:"])
def test_invalid_yaml_config_is_rejected(tmp_path, content):
    path = tmp_path / "invalid.yaml"
    path.write_text(content, encoding="utf-8")

    with pytest.raises((TypeError, ValueError, yaml.YAMLError)):
        load_seedream_config(path)


@pytest.mark.parametrize("prompt", ["", " \n\t"])
def test_empty_prompt_fails_before_http(tmp_path, image_path, mock_http, prompt):
    output = tmp_path / "line.png"
    with pytest.raises(ValueError):
        generate_line_art(image_path, prompt, SeedreamConfig(), output)
    assert mock_http == []
    assert not output.exists()


def test_missing_api_key_fails_before_http(tmp_path, image_path, mock_http, monkeypatch):
    monkeypatch.delenv("ARK_API_KEY", raising=False)
    output = tmp_path / "line.png"

    with pytest.raises(ValueError, match="ARK_API_KEY"):
        generate_line_art(image_path, "线稿", SeedreamConfig(), output)

    assert mock_http == []
    assert not output.exists()


@pytest.mark.parametrize("key", ["", "  "])
def test_empty_api_key_fails_before_http(tmp_path, image_path, mock_http, monkeypatch, key):
    monkeypatch.setenv("ARK_API_KEY", key)
    output = tmp_path / "line.png"
    with pytest.raises(ValueError):
        generate_line_art(image_path, "线稿", SeedreamConfig(), output)
    assert mock_http == []
    assert not output.exists()


def test_missing_input_fails_before_http(tmp_path, mock_http):
    with pytest.raises((FileNotFoundError, ValueError)):
        generate_line_art(tmp_path / "missing.png", "线稿", SeedreamConfig(), tmp_path / "line.png")
    assert mock_http == []


def test_invalid_input_image_fails_before_http(tmp_path, mock_http):
    path = tmp_path / "broken.png"
    path.write_bytes(b"invalid image")
    output = tmp_path / "line.png"
    with pytest.raises(ValueError):
        generate_line_art(path, "线稿", SeedreamConfig(), output)
    assert mock_http == []
    assert not output.exists()


def test_unsupported_input_format_fails_before_http(tmp_path, image_bytes, mock_http):
    path = tmp_path / "photo.txt"
    path.write_bytes(image_bytes)
    with pytest.raises(ValueError):
        generate_line_art(path, "线稿", SeedreamConfig(), tmp_path / "line.png")
    assert mock_http == []


def test_mismatched_image_suffix_fails_before_http(tmp_path, image_bytes, mock_http):
    path = tmp_path / "photo.jpg"
    path.write_bytes(image_bytes)
    output = tmp_path / "line.png"

    with pytest.raises(ValueError, match="文件后缀不符"):
        generate_line_art(path, "线稿", SeedreamConfig(), output)

    assert mock_http == []
    assert not output.exists()


@pytest.mark.parametrize("shape", [(14, 20), (20, 14), (15, 300)])
def test_input_image_dimensions_fail_before_http(tmp_path, mock_http, shape):
    image = np.full(shape, 255, dtype=np.uint8)
    path = tmp_path / "small.png"
    assert cv2.imwrite(str(path), image)
    with pytest.raises(ValueError):
        generate_line_art(path, "线稿", SeedreamConfig(), tmp_path / "line.png")
    assert mock_http == []


@pytest.mark.parametrize(
    "payload",
    [
        {"error": {"code": "InvalidParameter", "message": "尺寸无效"}},
        {"data": [{"error": {"code": "ContentFilter", "message": "审核失败"}}]},
        {"data": []},
        {"data": [{"b64_json": "a"}, {"b64_json": "b"}]},
        {"data": {}},
        {"data": [None]},
        {"data": [{}]},
        {"data": [{"url": "https://example.com/image.png"}]},
        {"data": [{"b64_json": "%%%"}]},
        {"data": [{"b64_json": ""}]},
        {"data": [{"b64_json": 123}]},
        {"data": [{"b64_json": base64.b64encode(b"invalid image").decode("ascii")}]},
        [],
    ],
)
def test_invalid_api_response_fails_without_output(tmp_path, image_path, monkeypatch, payload):
    calls = []

    def respond(request, *, timeout):
        calls.append(request)
        return io.BytesIO(json.dumps(payload).encode("utf-8"))

    monkeypatch.setattr(seedream, "urlopen", respond)
    monkeypatch.setenv("ARK_API_KEY", "test-key")
    output = tmp_path / "line.png"

    with pytest.raises((RuntimeError, TypeError, ValueError)):
        generate_line_art(image_path, "线稿", SeedreamConfig(), output)

    assert len(calls) == 1
    assert not output.exists()


@pytest.mark.parametrize("content", [b"not JSON", b"\xff"])
def test_invalid_json_response_has_diagnostic(tmp_path, image_path, monkeypatch, content):
    monkeypatch.setenv("ARK_API_KEY", "test-key")
    monkeypatch.setattr(seedream, "urlopen", lambda request, *, timeout: io.BytesIO(content))
    output = tmp_path / "line.png"
    with pytest.raises((RuntimeError, ValueError)):
        generate_line_art(image_path, "线稿", SeedreamConfig(), output)
    assert not output.exists()


@pytest.mark.parametrize(
    "error",
    [
        HTTPError(
            "https://example.com/api/v3/images/generations",
            429,
            "Too Many Requests",
            {},
            io.BytesIO(b'{"error":{"code":"RateLimitExceeded","message":"request limit"}}'),
        ),
        URLError("network unavailable"),
        URLError(TimeoutError("timed out")),
        TimeoutError("timed out"),
    ],
)
def test_http_and_timeout_errors_do_not_retry(tmp_path, image_path, monkeypatch, error):
    calls = []

    def fail(request, *, timeout):
        calls.append(request)
        raise error

    monkeypatch.setattr(seedream, "urlopen", fail)
    monkeypatch.setenv("ARK_API_KEY", "test-key")
    output = tmp_path / "line.png"

    with pytest.raises((RuntimeError, OSError)):
        generate_line_art(image_path, "线稿", SeedreamConfig(), output)

    assert len(calls) == 1
    assert not output.exists()


@pytest.mark.parametrize(
    ("code", "message"),
    [
        (
            "InvalidEndpointOrModel.NotFound",
            (
                "The model or endpoint doubao-seedream-5-0-lite-260128 doesnot exist or you do "
                "not have access to it. Request id: "
                "0217914484144777d6713dc852f73f0cf900d2fbe3d448869cbb3"
            ),
        ),
        (
            "ModelNotOpen",
            (
                "Your account test-account has not activated the model "
                "doubao-seedream-5-0-lite-260128. Please activate the model service in the Ark "
                "Console. Request id: model-not-open-request"
            ),
        ),
        (
            "InvalidEndpointOrModel.ModelIDAccessDisabled",
            (
                "Accessing the model via Model ID is not allowed for your account. Please use "
                "a custom endpoint ID instead. Request id: model-id-access-disabled-request"
            ),
        ),
    ],
)
def test_model_access_errors_preserve_details_and_show_configuration(
    tmp_path, image_path, monkeypatch, code, message
):
    config = SeedreamConfig(
        model="doubao-seedream-5-0-lite-260128",
        base_url="https://example.com/custom/v3",
    )
    body = json.dumps(
        {"error": {"code": code, "message": message, "param": "", "type": "Not Found"}}
    )
    calls = []

    def fail(request, *, timeout):
        calls.append(request)
        raise HTTPError(request.full_url, 404, "Not Found", {}, io.BytesIO(body.encode("utf-8")))

    monkeypatch.setattr(seedream, "urlopen", fail)
    monkeypatch.setenv("ARK_API_KEY", "test-key")
    output = tmp_path / "line.png"

    with pytest.raises(SeedreamAPIError) as error:
        generate_line_art(image_path, "线稿", config, output)

    diagnostic = str(error.value)
    assert "Seedream HTTP 404" in diagnostic
    assert code in diagnostic
    assert message in diagnostic
    for expected in (config.model, config.base_url, "Model ID", "ep-", "开通", "API Key", "地域"):
        assert expected in diagnostic
    assert len(calls) == 1
    assert json.loads(calls[0].data)["model"] == config.model
    assert not output.exists()


@pytest.mark.parametrize("body", [b"<html>upstream service unavailable</html>", b"\xffgateway failed"])
def test_non_json_http_error_preserves_body(tmp_path, image_path, monkeypatch, body):
    calls = []

    def fail(request, *, timeout):
        calls.append(request)
        raise HTTPError(request.full_url, 502, "Bad Gateway", {}, io.BytesIO(body))

    monkeypatch.setattr(seedream, "urlopen", fail)
    monkeypatch.setenv("ARK_API_KEY", "test-key")
    output = tmp_path / "line.png"

    with pytest.raises(SeedreamAPIError) as error:
        generate_line_art(image_path, "线稿", SeedreamConfig(), output)

    assert "Seedream HTTP 502" in str(error.value)
    assert body.decode("utf-8", errors="replace") in str(error.value)
    assert len(calls) == 1
    assert not output.exists()


@pytest.mark.parametrize(
    "payload",
    [
        {"error": {"code": "InvalidParameter", "message": "尺寸无效"}},
        {"data": [{"error": {"code": "ContentFilter", "message": "审核失败"}}]},
    ],
)
def test_api_rejection_uses_typed_error(tmp_path, image_path, monkeypatch, payload):
    monkeypatch.setenv("ARK_API_KEY", "test-key")
    monkeypatch.setattr(
        seedream,
        "urlopen",
        lambda request, *, timeout: io.BytesIO(json.dumps(payload).encode("utf-8")),
    )
    output = tmp_path / "line.png"

    with pytest.raises(SeedreamAPIError):
        generate_line_art(image_path, "线稿", SeedreamConfig(), output)

    assert not output.exists()


def test_response_failure_preserves_existing_output(tmp_path, image_path, monkeypatch):
    monkeypatch.setenv("ARK_API_KEY", "test-key")
    monkeypatch.setattr(
        seedream, "urlopen", lambda request, *, timeout: io.BytesIO(b'{"data":[]}')
    )
    output = tmp_path / "line.png"
    output.write_bytes(b"previous result")

    with pytest.raises((RuntimeError, ValueError)):
        generate_line_art(image_path, "线稿", SeedreamConfig(), output)

    assert output.read_bytes() == b"previous result"
