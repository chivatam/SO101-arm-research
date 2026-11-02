"""Save one 640x480 frame from each icspring camera.

Uses ffmpeg's AVFoundation device list. Does not open FaceTime or the iPhone
continuity camera. Exits after a few seconds if macOS has not granted camera
access.
"""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "runs" / "camera_frames"


def icspring_indexes() -> tuple[int, ...]:
    proc = subprocess.run(
        ["ffmpeg", "-hide_banner", "-f", "avfoundation", "-list_devices", "true", "-i", ""],
        capture_output=True,
        text=True,
        check=False,
    )
    indexes: list[int] = []
    in_video = False
    for line in proc.stderr.splitlines():
        if "AVFoundation video devices" in line:
            in_video = True
            continue
        if "AVFoundation audio devices" in line:
            break
        if not in_video:
            continue
        match = re.search(r"\[(\d+)\] (.+)$", line)
        if match and "icspring" in match.group(2).lower():
            indexes.append(int(match.group(1)))
    if len(indexes) < 2:
        raise SystemExit(f"Need two icspring cameras, found {indexes or 'none'}.")
    return tuple(indexes[:2])


def grab(index: int) -> Path:
    destination = OUT / f"avfoundation_{index}.jpg"
    command = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "error",
        "-y",
        "-f",
        "avfoundation",
        "-framerate",
        "30",
        "-pixel_format",
        "uyvy422",
        "-video_size",
        "640x480",
        "-i",
        f"{index}:none",
        "-ss",
        "0.4",
        "-frames:v",
        "1",
        str(destination),
    ]
    try:
        subprocess.run(command, check=True, timeout=12)
    except subprocess.TimeoutExpired as exc:
        raise SystemExit(
            f"Camera {index} did not return a frame. Allow Camera access for Cursor, then rerun this script."
        ) from exc
    return destination


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    for index in icspring_indexes():
        path = grab(index)
        print(path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
