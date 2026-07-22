#!/usr/bin/env python3
"""Camera -> Yandex AI Studio VLM -> validated GFS-X commands.

The default mode is deliberately a dry-run: it calls the VLM and prints the
validated commands, but it does not activate another robot.
"""

from __future__ import annotations

import argparse
import base64
import json
import os
import re
import sys
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

from camera_snapshot import (
    DEFAULT_CAMERA_URL,
    CameraSnapshotError,
    capture_jpeg,
)


DEFAULT_AI_URL = "https://ai.api.cloud.yandex.net/v1/responses"
DEFAULT_VISION_MODEL = "qwen3.6-35b-a3b"
DEFAULT_COMMAND_MODEL = "yandexgpt-5.1"
DEFAULT_YANDEX_ENV_FILE = "/src/rover_m2m/yandex-cloud.env"
CLASS_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")


class DispatcherError(RuntimeError):
    """Expected, user-facing connector error."""


def load_env_file(path: str | None) -> None:
    """Load a small KEY=VALUE environment file without third-party packages."""
    if not path:
        return
    env_path = Path(path).expanduser()
    if not env_path.exists():
        raise DispatcherError(f"Файл конфигурации не найден: {env_path}")
    for line_number, raw_line in enumerate(env_path.read_text().splitlines(), 1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            raise DispatcherError(
                f"Некорректная строка {line_number} в {env_path}: ожидается KEY=VALUE"
            )
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip().strip("'\"")
        if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", key):
            raise DispatcherError(f"Некорректное имя переменной в строке {line_number}")
        os.environ.setdefault(key, value)


def capture_frame(camera_url: str, jpeg_quality: int = 85) -> bytes:
    """Read one frame from MediaMTX and return JPEG bytes."""
    try:
        return capture_jpeg(camera_url, jpeg_quality)
    except CameraSnapshotError as exc:
        raise DispatcherError(str(exc)) from exc


def read_image(path: str) -> bytes:
    image_path = Path(path).expanduser()
    try:
        data = image_path.read_bytes()
    except OSError as exc:
        raise DispatcherError(f"Не удалось прочитать изображение {image_path}: {exc}") from exc
    if not data:
        raise DispatcherError(f"Изображение пустое: {image_path}")
    return data


def build_prompt(allowed_classes: set[str]) -> str:
    allowed = ""
    if allowed_classes:
        allowed = (
            " class_name ОБЯЗАН быть одним из: "
            + ", ".join(sorted(allowed_classes))
            + "."
        )
    return (
        "Ты визуальный диспетчер M2M-эстафеты. Определи один главный предмет "
        "перед камерой ровера и верни только JSON-массив ровно из двух объектов, "
        "без Markdown и пояснений. Формат: "
        '[{"target":"gfsx_yolo","action":"set_target_class",'
        '"class_name":"object_class"},'
        '{"target":"gfsx_robot","action":"activate","state":"start"}].'
        " class_name — короткое имя класса латиницей, допустимы A-Z, a-z, 0-9, "
        "точка, дефис и подчёркивание."
        + allowed
        + " Если предмет нельзя уверенно определить, верни class_name unknown."
    )


def post_json(
    url: str,
    payload: dict[str, Any],
    headers: dict[str, str],
    timeout: float,
) -> tuple[int, Any]:
    request = urllib.request.Request(
        url,
        data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
        headers={"Content-Type": "application/json", **headers},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            raw = response.read().decode("utf-8", errors="replace")
            return response.status, json.loads(raw) if raw else None
    except urllib.error.HTTPError as exc:
        body = exc.read().decode("utf-8", errors="replace")[:2000]
        raise DispatcherError(f"HTTP {exc.code} от {url}: {body}") from exc
    except urllib.error.URLError as exc:
        raise DispatcherError(f"Сетевая ошибка при обращении к {url}: {exc.reason}") from exc
    except json.JSONDecodeError as exc:
        raise DispatcherError(f"Сервис {url} вернул не-JSON ответ") from exc


def extract_output_text(response: dict[str, Any]) -> str:
    direct = response.get("output_text")
    if isinstance(direct, str) and direct.strip():
        return direct.strip()
    for item in response.get("output", []):
        if not isinstance(item, dict):
            continue
        content_items = item.get("content")
        if not isinstance(content_items, list):
            continue
        for content in content_items:
            if isinstance(content, dict) and content.get("type") == "output_text":
                text = content.get("text")
                if isinstance(text, str) and text.strip():
                    return text.strip()
    output_summary = []
    for item in response.get("output", []):
        if not isinstance(item, dict):
            continue
        content = item.get("content")
        content_types = (
            [part.get("type") for part in content if isinstance(part, dict)]
            if isinstance(content, list)
            else None
        )
        output_summary.append(
            {"type": item.get("type"), "status": item.get("status"), "content": content_types}
        )
    diagnostic = {
        "status": response.get("status"),
        "error": response.get("error"),
        "incomplete_details": response.get("incomplete_details"),
        "output": output_summary,
    }
    raise DispatcherError(
        "Yandex AI Studio не вернул output_text: "
        + json.dumps(diagnostic, ensure_ascii=False)
    )


def parse_and_validate_commands(text: str, allowed_classes: set[str]) -> list[dict[str, str]]:
    cleaned = text.strip()
    if cleaned.startswith("```") and cleaned.endswith("```"):
        lines = cleaned.splitlines()
        cleaned = "\n".join(lines[1:-1]).strip()
    try:
        commands = json.loads(cleaned)
    except json.JSONDecodeError as exc:
        raise DispatcherError(f"Ответ модели не является корректным JSON: {exc.msg}") from exc

    if not isinstance(commands, list) or len(commands) != 2:
        raise DispatcherError("Модель должна вернуть массив ровно из двух команд")
    first, second = commands
    if not isinstance(first, dict) or set(first) != {"target", "action", "class_name"}:
        raise DispatcherError("Первая команда имеет недопустимую структуру")
    if first.get("target") != "gfsx_yolo" or first.get("action") != "set_target_class":
        raise DispatcherError("Первая команда не является безопасной set_target_class")
    class_name = first.get("class_name")
    if not isinstance(class_name, str) or not CLASS_RE.fullmatch(class_name):
        raise DispatcherError("class_name имеет недопустимый формат")
    if allowed_classes and class_name not in allowed_classes:
        raise DispatcherError(
            f"Класс {class_name!r} отсутствует в GFSX_ALLOWED_CLASSES"
        )
    expected_second = {"target": "gfsx_robot", "action": "activate", "state": "start"}
    if second != expected_second:
        raise DispatcherError("Вторая команда не является разрешённой командой activate/start")
    return [dict(first), dict(second)]


def query_yandex(
    image_bytes: bytes,
    folder_id: str,
    api_key: str,
    model: str,
    command_model: str,
    api_url: str,
    allowed_classes: set[str],
    timeout: float,
) -> list[dict[str, str]]:
    image_b64 = base64.b64encode(image_bytes).decode("ascii")
    payload = {
        "model": f"gpt://{folder_id}/{model}",
        "input": [
            {
                "role": "user",
                "content": [
                    {"type": "input_text", "text": build_prompt(allowed_classes)},
                    {
                        "type": "input_image",
                        "image_url": f"data:image/jpeg;base64,{image_b64}",
                        "detail": "auto",
                    },
                ],
            }
        ],
    }
    _, response = post_json(
        api_url,
        payload,
        {"Authorization": f"Api-Key {api_key}", "OpenAI-Project": folder_id},
        timeout,
    )
    if not isinstance(response, dict):
        raise DispatcherError("Yandex AI Studio вернул неожиданный ответ")
    vision_commands = parse_and_validate_commands(
        extract_output_text(response), allowed_classes
    )
    if not command_model:
        return vision_commands

    command_prompt = (
        "Ты диспетчер робототехнической M2M-эстафеты. Визуальная модель уже "
        "определила объект и предложила команды ниже. Верни только JSON-массив "
        "ровно из двух команд без Markdown и пояснений. Не меняй class_name. "
        "Разрешены только set_target_class для gfsx_yolo и activate/start для "
        "gfsx_robot. Результат визуального анализа: "
        + json.dumps(vision_commands, ensure_ascii=False)
    )
    command_payload = {
        "model": f"gpt://{folder_id}/{command_model}",
        "input": [
            {
                "role": "user",
                "content": [{"type": "input_text", "text": command_prompt}],
            }
        ],
    }
    _, command_response = post_json(
        api_url,
        command_payload,
        {"Authorization": f"Api-Key {api_key}", "OpenAI-Project": folder_id},
        timeout,
    )
    if not isinstance(command_response, dict):
        raise DispatcherError("YandexGPT вернул неожиданный ответ")
    final_commands = parse_and_validate_commands(
        extract_output_text(command_response), allowed_classes
    )
    if final_commands[0]["class_name"] != vision_commands[0]["class_name"]:
        raise DispatcherError("YandexGPT изменил class_name визуальной модели")
    return final_commands


def send_to_unity(
    commands: list[dict[str, str]], unity_base_url: str, timeout: float
) -> None:
    base = unity_base_url.rstrip("/")
    for endpoint, command in (("set_target", commands[0]), ("activate", commands[1])):
        status, _ = post_json(f"{base}/{endpoint}", command, {}, timeout)
        if status < 200 or status >= 300:
            raise DispatcherError(f"Unity /{endpoint} вернул HTTP {status}")


def allowed_classes_from_env() -> set[str]:
    raw = os.getenv("GFSX_ALLOWED_CLASSES", "")
    values = {item.strip() for item in raw.split(",") if item.strip()}
    invalid = sorted(value for value in values if not CLASS_RE.fullmatch(value))
    if invalid:
        raise DispatcherError(f"Недопустимые классы в GFSX_ALLOWED_CLASSES: {invalid}")
    return values


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    configured_env = os.getenv("YANDEX_CLOUD_ENV_FILE")
    default_env = configured_env or (
        DEFAULT_YANDEX_ENV_FILE if Path(DEFAULT_YANDEX_ENV_FILE).exists() else None
    )
    parser.add_argument(
        "--env-file",
        default=default_env,
        help="Файл конфигурации Yandex Cloud в формате KEY=VALUE",
    )
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--image", help="Использовать локальный JPEG/PNG вместо камеры")
    source.add_argument("--camera-url", help="Переопределить URL камеры")
    parser.add_argument("--send", action="store_true", help="Отправить команды в Unity")
    parser.add_argument("--timeout", type=float, default=60.0)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    load_env_file(args.env_file)
    folder_id = os.getenv("YANDEX_FOLDER_ID", "")
    api_key = os.getenv("YANDEX_API_KEY", "")
    model = os.getenv(
        "YANDEX_VISION_MODEL", os.getenv("YANDEX_MODEL", DEFAULT_VISION_MODEL)
    )
    command_model = os.getenv("YANDEX_COMMAND_MODEL", DEFAULT_COMMAND_MODEL)
    api_url = os.getenv("YANDEX_API_URL", DEFAULT_AI_URL)
    allowed_classes = allowed_classes_from_env()
    if not folder_id or not api_key:
        raise DispatcherError("Нужны YANDEX_FOLDER_ID и YANDEX_API_KEY")

    print("1/3 Получение изображения...", flush=True)
    image = read_image(args.image) if args.image else capture_frame(
        args.camera_url or os.getenv("ROVER_CAMERA_URL", DEFAULT_CAMERA_URL)
    )
    print(
        f"2/3 Анализ в Yandex AI Studio ({model} -> {command_model})...",
        flush=True,
    )
    commands = query_yandex(
        image,
        folder_id,
        api_key,
        model,
        command_model,
        api_url,
        allowed_classes,
        args.timeout,
    )
    print(json.dumps(commands, ensure_ascii=False, indent=2))

    if not args.send:
        print("3/3 DRY-RUN: команды в Unity не отправлялись.")
        return 0
    unity_url = os.getenv("UNITY_API_URL", "")
    if not unity_url:
        raise DispatcherError("Для --send нужна переменная UNITY_API_URL")
    if not allowed_classes:
        raise DispatcherError("Для --send обязателен непустой GFSX_ALLOWED_CLASSES")
    print(f"3/3 Отправка команд в {unity_url}...", flush=True)
    send_to_unity(commands, unity_url, args.timeout)
    print("M2M-цепочка завершена.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except DispatcherError as exc:
        print(f"Ошибка: {exc}", file=sys.stderr)
        raise SystemExit(2)
