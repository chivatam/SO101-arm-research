"""Save one 640x480 frame from each icspring camera.

Uses ffmpeg's AVFoundation device list. Does not open FaceTime or the iPhone
continuity camera. Exits after a few seconds if macOS has not granted camera
access.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "bringup" / "camera_frames"
DEVICES = (0, 1)


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
    for index in DEVICES:
        path = grab(index)
        print(path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
