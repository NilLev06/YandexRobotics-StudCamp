#!/usr/bin/env python3
"""Хостовый сборщик телеметрии ровера.

Пишет в каталог сессии два JSONL-файла (каждая строка — с UTC-таймстампом):
  system.jsonl      — метрики Raspberry Pi: CPU, память, температура,
                      троттлинг, диск, Wi-Fi, статистика контейнеров
  yolo_health.jsonl — снимки /health YOLO-сайдкара: детекции, fps, ошибки

Плюс сторож диска: при нехватке свободного места останавливает контейнеры
записи (rosbag/видео), при превышении локального капа удаляет самые старые
завершённые сессии. Работает только со стандартной библиотекой.
"""

from __future__ import annotations

import argparse
import json
import shutil
import signal
import subprocess
import sys
import time
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

RECORDER_CONTAINERS = ("z-boys-bag", "z-boys-video", "z-boys-yolo-video")
SYSTEM_INTERVAL_S = 1.0
YOLO_INTERVAL_S = 1.0
DOCKER_STATS_INTERVAL_S = 15.0
DISK_GUARD_INTERVAL_S = 30.0

GIB = 1024 ** 3


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def append_jsonl(path: Path, record: dict) -> None:
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")))
        handle.write("\n")


def read_first_line(path: str) -> str | None:
    try:
        with open(path, "r", encoding="ascii") as handle:
            return handle.readline().strip()
    except OSError:
        return None


def read_meminfo() -> dict:
    values = {}
    try:
        with open("/proc/meminfo", "r", encoding="ascii") as handle:
            for line in handle:
                key, _, rest = line.partition(":")
                values[key] = int(rest.strip().split()[0])  # kB
    except (OSError, ValueError, IndexError):
        return {}
    return {
        "mem_total_mb": values.get("MemTotal", 0) // 1024,
        "mem_available_mb": values.get("MemAvailable", 0) // 1024,
        "swap_free_mb": values.get("SwapFree", 0) // 1024,
    }


def read_cpu_temp_c() -> float | None:
    raw = read_first_line("/sys/class/thermal/thermal_zone0/temp")
    if raw is None:
        return None
    try:
        return round(int(raw) / 1000.0, 1)
    except ValueError:
        return None


def read_loadavg() -> list[float] | None:
    raw = read_first_line("/proc/loadavg")
    if raw is None:
        return None
    try:
        return [float(part) for part in raw.split()[:3]]
    except (ValueError, IndexError):
        return None


def read_throttled() -> str | None:
    """vcgencmd get_throttled: битовая маска undervoltage/троттлинга Pi."""
    try:
        out = subprocess.run(
            ["vcgencmd", "get_throttled"],
            capture_output=True, text=True, timeout=2.0,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    if out.returncode != 0:
        return None
    _, _, value = out.stdout.strip().partition("=")
    return value or None


def read_clock_synced() -> bool | None:
    """timedatectl: was the clock set from NTP or is the Pi still on its
    power-on default (no RTC battery -> timestamps are unreliable until
    synced)."""
    try:
        out = subprocess.run(
            ["timedatectl", "show", "-p", "NTPSynchronized", "--value"],
            capture_output=True, text=True, timeout=2.0,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    if out.returncode != 0:
        return None
    value = out.stdout.strip()
    if value in ("yes", "no"):
        return value == "yes"
    return None


def read_wifi() -> dict | None:
    try:
        with open("/proc/net/wireless", "r", encoding="ascii") as handle:
            lines = handle.readlines()
    except OSError:
        return None
    for line in lines[2:]:
        parts = line.split()
        if len(parts) >= 4:
            try:
                return {
                    "iface": parts[0].rstrip(":"),
                    "link_quality": float(parts[2].rstrip(".")),
                    "signal_dbm": float(parts[3].rstrip(".")),
                }
            except ValueError:
                return None
    return None


def read_docker_stats() -> list[dict] | None:
    try:
        out = subprocess.run(
            ["docker", "stats", "--no-stream", "--format",
             "{{.Name}}\t{{.CPUPerc}}\t{{.MemUsage}}"],
            capture_output=True, text=True, timeout=10.0,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    if out.returncode != 0:
        return None
    stats = []
    for line in out.stdout.strip().splitlines():
        parts = line.split("\t")
        if len(parts) == 3:
            stats.append({"name": parts[0], "cpu": parts[1], "mem": parts[2]})
    return stats or None


def fetch_yolo_health(url: str) -> dict | None:
    request = urllib.request.Request(url, headers={"Cache-Control": "no-store"})
    try:
        with urllib.request.urlopen(request, timeout=1.5) as response:
            body = response.read(256 * 1024)
    except urllib.error.HTTPError as exc:
        body = exc.read(256 * 1024)
    except (OSError, urllib.error.URLError):
        return None
    try:
        payload = json.loads(body)
    except (json.JSONDecodeError, UnicodeDecodeError):
        return None
    return payload if isinstance(payload, dict) else None


def stop_recorders(reason: str, events_path: Path) -> None:
    append_jsonl(events_path, {"ts": utc_now(), "event": "recorders_stopped",
                               "reason": reason})
    for name in RECORDER_CONTAINERS:
        subprocess.run(["docker", "stop", "--time", "15", name],
                       capture_output=True, timeout=30.0)


def dir_size_bytes(path: Path) -> int:
    total = 0
    for item in path.rglob("*"):
        try:
            if item.is_file():
                total += item.stat().st_size
        except OSError:
            continue
    return total


def disk_guard(sessions_dir: Path, current_session: Path, events_path: Path,
               cap_gb: float, min_free_gb: float, recorders_stopped: bool) -> bool:
    free_gb = shutil.disk_usage(sessions_dir).free / GIB
    if free_gb < min_free_gb and not recorders_stopped:
        stop_recorders(f"free disk {free_gb:.2f} GiB < {min_free_gb} GiB",
                       events_path)
        return True

    # Кап на общий объём: удаляем самые старые ЗАВЕРШЁННЫЕ сессии
    # (без маркера .active), пока не влезем.
    sessions = sorted(
        item for item in sessions_dir.iterdir()
        if item.is_dir() and item != current_session
        and not (item / ".active").exists()
    )
    total_gb = sum(dir_size_bytes(item) for item in sessions_dir.iterdir()
                   if item.is_dir()) / GIB
    while total_gb > cap_gb and sessions:
        victim = sessions.pop(0)
        victim_gb = dir_size_bytes(victim) / GIB
        shutil.rmtree(victim, ignore_errors=True)
        total_gb -= victim_gb
        append_jsonl(events_path, {"ts": utc_now(), "event": "session_pruned",
                                   "session": victim.name,
                                   "freed_gb": round(victim_gb, 2)})
    return recorders_stopped


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-dir", required=True)
    parser.add_argument("--yolo-url", default="http://127.0.0.1:8092/health")
    parser.add_argument("--cap-gb", type=float, default=10.0)
    parser.add_argument("--min-free-gb", type=float, default=2.0)
    args = parser.parse_args()

    session_dir = Path(args.session_dir)
    sessions_dir = session_dir.parent
    session_dir.mkdir(parents=True, exist_ok=True)
    system_path = session_dir / "system.jsonl"
    yolo_path = session_dir / "yolo_health.jsonl"
    events_path = session_dir / "events.jsonl"

    stop = {"requested": False}

    def handle_signal(_signum, _frame):
        stop["requested"] = True

    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)

    append_jsonl(events_path, {"ts": utc_now(), "event": "collector_started"})

    last_yolo = 0.0
    last_docker = 0.0
    last_guard = 0.0
    last_yolo_seq = None
    recorders_stopped = False
    last_clock_synced = None

    while not stop["requested"]:
        cycle_started = time.monotonic()

        clock_synced = read_clock_synced()
        if clock_synced is not None and clock_synced != last_clock_synced:
            # The Pi has no RTC battery: timestamps before the first sync
            # are unreliable. Log the transition so recordings can be
            # flagged/corrected instead of silently trusted.
            append_jsonl(events_path, {"ts": utc_now(), "event": "clock_sync_changed",
                                       "clock_synced": clock_synced})
            last_clock_synced = clock_synced

        record = {
            "ts": utc_now(),
            "loadavg": read_loadavg(),
            "cpu_temp_c": read_cpu_temp_c(),
            "throttled": read_throttled(),
            "disk_free_gb": round(shutil.disk_usage(sessions_dir).free / GIB, 2),
            "wifi": read_wifi(),
            "clock_synced": clock_synced,
        }
        record.update(read_meminfo())
        if cycle_started - last_docker >= DOCKER_STATS_INTERVAL_S:
            record["docker"] = read_docker_stats()
            last_docker = cycle_started
        append_jsonl(system_path, record)

        if cycle_started - last_yolo >= YOLO_INTERVAL_S:
            payload = fetch_yolo_health(args.yolo_url)
            last_yolo = cycle_started
            if payload is not None:
                seq = payload.get("detection_seq")
                # Пишем только новые кадры инференса, чтобы не дублировать
                if seq != last_yolo_seq:
                    last_yolo_seq = seq
                    append_jsonl(yolo_path, {"ts": utc_now(), **payload})

        if cycle_started - last_guard >= DISK_GUARD_INTERVAL_S:
            recorders_stopped = disk_guard(
                sessions_dir, session_dir, events_path,
                args.cap_gb, args.min_free_gb, recorders_stopped,
            )
            last_guard = cycle_started

        elapsed = time.monotonic() - cycle_started
        time.sleep(max(0.0, SYSTEM_INTERVAL_S - elapsed))

    append_jsonl(events_path, {"ts": utc_now(), "event": "collector_stopped"})
    return 0


if __name__ == "__main__":
    sys.exit(main())
