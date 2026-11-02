#!/usr/bin/env python3
"""Prompt page for this SO-101 follower and the LeRobot MolmoAct2 checkpoint.

Run from the repo root:

    .venv/bin/python operator/server.py

Open http://127.0.0.1:8771. The arm stays still until Act is pressed, and that
button stays disabled until the power-switch box is checked. Stop drops torque.
"""

from __future__ import annotations

import json
import os
import threading
import time
import traceback
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

os.environ.setdefault("PYTORCH_ENABLE_MPS_FALLBACK", "1")

import cv2
import numpy as np
import torch
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.motors.motors_bus import SerialMotorsBus
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.molmoact2.configuration_molmoact2 import MolmoAct2Config
from lerobot.policies.molmoact2.modeling_molmoact2 import MolmoAct2Policy
from lerobot.policies.pretrained import PreTrainedPolicy
from lerobot.policies.utils import prepare_observation_for_inference
from lerobot.processor.converters import policy_output_to_transition, transition_to_policy_action
from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
from lerobot.robots.so_follower.so_follower import SOFollower

ROOT = Path(__file__).resolve().parents[1]


def load_bench() -> dict:
    path = ROOT / "config" / "bench.json"
    if not path.is_file():
        raise SystemExit(f"Missing {path}. Set the serial port in config/bench.json.")
    with path.open() as handle:
        return json.load(handle)


BENCH = load_bench()
PORT_PATH = BENCH["port"]
ROBOT_ID = BENCH.get("robot_id", "so101_follower")
POLICY_ID = BENCH.get("policy_id", "lerobot/MolmoAct2-SO100_101-LeRobot")
RUN_DIR = ROOT / "runs" / "operator_runs"
HOST = str(BENCH.get("operator", {}).get("host", "127.0.0.1"))
PORT = int(BENCH.get("operator", {}).get("port", 8771))
NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
READY = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -30.0,
    "elbow_flex": 35.0,
    "wrist_flex": 58.0,
    "gripper": 40.0,
}
LIMITS = {
    "shoulder_lift": (-110.0, 60.0),
    "elbow_flex": (-70.0, 110.0),
    "wrist_flex": (-20.0, 100.0),
}

arm_lock = threading.Lock()
preview_lock = threading.Lock()
stop_event = threading.Event()
run_thread: threading.Thread | None = None
robot: SOFollower | None = None
policy = None
preprocessor = None
postprocessor = None
log_lines: list[str] = []
latest_jpeg = {"table": b"", "close": b""}
status = {
    "running": False,
    "phase": "idle",
    "inference_s": None,
    "error": None,
    "task": "",
    "joints": {},
}


def note(message: str) -> None:
    stamp = time.strftime("%H:%M:%S")
    log_lines.append(f"{stamp}  {message}")
    del log_lines[:-80]
    print(f"{stamp}  {message}", flush=True)


def _load_cached_backbone(cls, pretrained_name_or_path, *, config=None, **kwargs):
    """Use the allenai weights already loaded in MolmoAct2.__init__.

    The LeRobot repo also publishes a second full model.safetensors. Loading it
    again duplicates the 5B weights and stalls on an unfinished download.
    """
    del pretrained_name_or_path, kwargs
    if config is None:
        raise RuntimeError("MolmoAct2 config was not prepared before load.")
    instance = cls(config)
    instance.to(config.device or "cpu")
    instance.eval()
    return instance


def ensure_policy() -> None:
    global policy, preprocessor, postprocessor
    if policy is not None:
        return
    note("loading MolmoAct2. The arm stays still.")
    config = MolmoAct2Config.from_pretrained(POLICY_ID)
    config.train_mode_vlm = "fft"
    config.enable_inference_cuda_graph = False
    config.device = "mps"
    original = PreTrainedPolicy.from_pretrained
    PreTrainedPolicy.from_pretrained = classmethod(_load_cached_backbone)
    try:
        policy = MolmoAct2Policy.from_pretrained(POLICY_ID, config=config)
    finally:
        PreTrainedPolicy.from_pretrained = original
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=policy.config,
        pretrained_path=POLICY_ID,
        preprocessor_overrides={
            "device_processor": {"device": "mps"},
            "rename_observations_processor": {"rename_map": {}},
        },
    )
    note("model ready")


def make_robot(table_index: int, close_index: int) -> SOFollower:
    SerialMotorsBus.default_timeout = 250
    config = SOFollowerRobotConfig(
        port=PORT_PATH,
        id=ROBOT_ID,
        max_relative_target=8.0,
        cameras={
            "table": OpenCVCameraConfig(index_or_path=table_index, fps=30, width=640, height=480),
            "close": OpenCVCameraConfig(index_or_path=close_index, fps=30, width=640, height=480),
        },
    )
    return SOFollower(config)


def connect_robot(follower: SOFollower) -> None:
    """Connect without the official handshake or a calibration rewrite."""
    follower.bus.connect(handshake=False)
    try:
        if not follower.is_calibrated:
            raise RuntimeError("Calibration on the arm does not match the saved file. Nothing was recalibrated.")
        for cam in follower.cameras.values():
            cam.connect()
        follower.configure()
    except Exception:
        if follower.bus.is_connected:
            follower.bus.disconnect(disable_torque=True)
        raise


def release(follower: SOFollower | None) -> None:
    if follower is None:
        return
    with arm_lock:
        try:
            if follower.is_connected:
                follower.disconnect()
            elif follower.bus.is_connected:
                follower.bus.disconnect(disable_torque=True)
        except Exception as exc:
            note(f"torque-off failed: {exc}")


def read_joints(follower: SOFollower) -> dict[str, float]:
    with arm_lock:
        present = follower.bus.sync_read("Present_Position", normalize=True, num_retry=2)
    return {name: float(present[name]) for name in NAMES}


def remember_views(raw: dict) -> None:
    for key, slot in (("table", "table"), ("close", "close")):
        frame = raw.get(key)
        if frame is None:
            continue
        bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        ok, encoded = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if ok:
            latest_jpeg[slot] = encoded.tobytes()


def preview_jpeg(index: int) -> bytes:
    capture = cv2.VideoCapture(index)
    try:
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if not capture.isOpened():
            raise RuntimeError(f"OpenCV camera {index} did not open.")
        frame = None
        for _ in range(8):
            ok, frame = capture.read()
            if ok and frame is not None and frame.size > 1000:
                break
        if frame is None:
            raise RuntimeError(f"OpenCV camera {index} returned an empty frame.")
        ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            raise RuntimeError(f"Could not encode camera {index}.")
        return encoded.tobytes()
    finally:
        capture.release()


def outside_limits(joints: dict[str, float]) -> str | None:
    for name, (low, high) in LIMITS.items():
        value = joints[name]
        if value < low or value > high:
            return f"{name} at {value:.1f}° is outside {low:.0f}° to {high:.0f}°."
    return None


def send_goal(follower: SOFollower, goal: dict[str, float]) -> dict[str, float]:
    goal = dict(goal)
    goal["gripper"] = float(np.clip(goal["gripper"], 0.0, 100.0))
    blocked = outside_limits(goal)
    if blocked:
        raise RuntimeError(blocked + " Nothing further was sent.")
    with arm_lock:
        sent = follower.send_action({f"{name}.pos": float(goal[name]) for name in NAMES})
    return {name: float(sent[f"{name}.pos"]) for name in NAMES}


def move_to_ready(follower: SOFollower) -> None:
    present = read_joints(follower)
    wrist_start = present["wrist_roll"]
    target = dict(READY)
    target["wrist_roll"] = float(np.clip(wrist_start, -12.0, 12.0))
    note(
        "moving to the training pose: "
        f"shoulder {target['shoulder_lift']:.0f}, elbow {target['elbow_flex']:.0f}, gripper {target['gripper']:.0f}"
    )
    deadline = time.perf_counter() + 28.0
    while time.perf_counter() < deadline:
        if stop_event.is_set():
            raise RuntimeError("Stopped during the move to the training pose.")
        present = read_joints(follower)
        if abs(present["wrist_roll"] - wrist_start) > 25.0:
            raise RuntimeError("Wrist roll moved more than 25° during the training-pose move.")
        error = max(abs(present[name] - target[name]) for name in NAMES)
        if error < 6.0:
            note(f"training pose reached, largest error {error:.1f}°")
            return
        send_goal(follower, target)
        time.sleep(0.1)
    present = read_joints(follower)
    error = max(abs(present[name] - target[name]) for name in NAMES)
    raise RuntimeError(f"Training pose was not reached. Largest error {error:.1f}°.")


def policy_goal(follower: SOFollower, task: str) -> tuple[dict[str, float], float]:
    with arm_lock:
        raw = follower.get_observation()
    remember_views(raw)
    state = np.array([raw[f"{name}.pos"] for name in NAMES], dtype=np.float32)
    frame = {
        "observation.state": state,
        "observation.images.cam0": np.ascontiguousarray(raw["table"]),
        "observation.images.cam1": np.ascontiguousarray(raw["close"]),
    }
    started = time.perf_counter()
    prepared = prepare_observation_for_inference(frame, torch.device("mps"), task, robot_type="so101")
    batch = preprocessor(prepared)
    output = policy.select_action(batch)
    action = postprocessor(transition_to_policy_action(policy_output_to_transition(output)))
    elapsed = time.perf_counter() - started
    values = action.detach().float().cpu().numpy().reshape(-1)
    if values.shape[0] < 6 or not np.isfinite(values[:6]).all():
        raise RuntimeError(f"model returned {values.shape}")
    goal = {name: float(values[i]) for i, name in enumerate(NAMES)}
    return goal, elapsed


def record_pair(writers: dict[str, cv2.VideoWriter]) -> None:
    for slot, writer in writers.items():
        payload = latest_jpeg[slot]
        if not payload:
            continue
        frame = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
        if frame is not None:
            writer.write(frame)


def run_policy(body: dict) -> None:
    global robot
    status["running"] = True
    status["error"] = None
    status["phase"] = "loading"
    stop_event.clear()
    follower = None
    writers: dict[str, cv2.VideoWriter] = {}
    try:
        task = str(body.get("task") or "").strip()
        if not task:
            raise RuntimeError("Write a prompt first.")
        if not body.get("confirmed"):
            raise RuntimeError("Confirm you are at the power switch before the arm moves.")
        duration = min(45.0, max(1.0, float(body.get("duration", 20))))
        table_index = int(body.get("table_index", 0))
        close_index = int(body.get("close_index", 1))
        if table_index == close_index:
            raise RuntimeError("The table camera and the gripper camera have to be different.")
        status["task"] = task
        ensure_policy()
        if stop_event.is_set():
            note("stopped before the arm was connected")
            status["phase"] = "idle"
            return
        policy.reset()
        status["phase"] = "connecting"
        follower = make_robot(table_index, close_index)
        robot = follower
        connect_robot(follower)
        note(f"connected. prompt: {task}")
        if body.get("ready", True):
            status["phase"] = "training pose"
            move_to_ready(follower)
        run_path = RUN_DIR / datetime.now().strftime("%Y%m%d-%H%M%S")
        run_path.mkdir(parents=True, exist_ok=True)
        (run_path / "prompt.txt").write_text(task + "\n")
        for slot in ("table", "close"):
            writers[slot] = cv2.VideoWriter(
                str(run_path / f"{slot}.mp4"),
                cv2.VideoWriter_fourcc(*"mp4v"),
                10.0,
                (640, 480),
            )
        status["phase"] = "acting"
        started = time.perf_counter()
        wrist_origin = None
        ticks = 0
        while not stop_event.is_set() and (time.perf_counter() - started) < duration:
            present = read_joints(follower)
            status["joints"] = {name: round(value, 1) for name, value in present.items()}
            if wrist_origin is None:
                wrist_origin = present["wrist_roll"]
            elif abs(present["wrist_roll"] - wrist_origin) > 90.0:
                raise RuntimeError("Wrist roll moved more than 90° from the start of this prompt.")
            blocked = outside_limits(present)
            if blocked:
                raise RuntimeError(blocked)
            goal, elapsed = policy_goal(follower, task)
            status["inference_s"] = round(elapsed, 2)
            if elapsed > 0.4:
                note(
                    f"think {elapsed:.1f}s, shoulder {goal['shoulder_lift']:.1f}, "
                    f"elbow {goal['elbow_flex']:.1f}, gripper {goal['gripper']:.1f}"
                )
            if stop_event.is_set():
                break
            send_goal(follower, goal)
            record_pair(writers)
            ticks += 1
            time.sleep(0.05)
        note(f"prompt finished after {ticks} steps. video: {run_path}")
        status["phase"] = "idle"
    except Exception as exc:
        status["error"] = str(exc)
        status["phase"] = "error"
        note(f"error: {exc}")
        traceback.print_exc()
    finally:
        for writer in writers.values():
            writer.release()
        release(follower)
        robot = None
        status["running"] = False
        if status["phase"] != "error":
            status["phase"] = "idle"
        note("torque off")


PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>MolmoAct2 on SO-101</title>
<style>
  :root { color-scheme: light; }
  body { margin: 0; font: 16px/1.45 "Iowan Old Style", Palatino, Georgia, serif; background: #f4f0e8; color: #1c1915; }
  header { padding: 22px 28px 4px; }
  h1 { font-size: 28px; font-weight: 600; margin: 0; }
  p.lead { margin: 6px 0 0; max-width: 42rem; color: #3f3a33; }
  main { display: grid; grid-template-columns: minmax(0, 1.2fr) minmax(300px, 0.8fr); gap: 18px; padding: 16px 22px 36px; }
  figure { margin: 0 0 12px; background: #fffdf8; border: 1px solid #ddd4c4; }
  img { width: 100%; height: 240px; object-fit: contain; background: #161616; display: block; }
  figcaption { display: flex; gap: 10px; align-items: center; padding: 8px 10px; }
  select, input, button, textarea { font: 15px/1.3 ui-sans-serif, system-ui, sans-serif; }
  select { min-width: 4.5rem; }
  textarea, input[type="number"] { width: 100%; box-sizing: border-box; }
  textarea { min-height: 88px; padding: 10px; }
  label { display: block; margin: 14px 0 4px; font: 13px/1.2 ui-sans-serif, system-ui, sans-serif; letter-spacing: 0.04em; text-transform: uppercase; color: #5c564c; }
  .panel { background: #fffdf8; border: 1px solid #ddd4c4; padding: 14px 16px 18px; }
  table { width: 100%; border-collapse: collapse; font-variant-numeric: tabular-nums; }
  td { padding: 3px 0; }
  td:last-child { text-align: right; }
  button { border: 0; padding: 11px 12px; cursor: pointer; }
  button.primary { background: #1c1915; color: #f4f0e8; }
  button.quiet { background: #e7e0d4; color: #1c1915; }
  button.stop { background: #9d2c2c; color: white; font-weight: 650; }
  button:disabled { opacity: 0.45; cursor: default; }
  .actions { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; margin-top: 14px; }
  .check { text-transform: none; letter-spacing: 0; font: 15px/1.35 ui-sans-serif, system-ui, sans-serif; color: #1c1915; display: flex; gap: 8px; align-items: flex-start; margin-top: 12px; }
  .check input { margin-top: 3px; }
  #log { white-space: pre-wrap; font: 13px/1.4 ui-monospace, monospace; min-height: 8rem; margin: 8px 0 0; }
  .warn { color: #9d2c2c; }
  @media (max-width: 800px) { main { grid-template-columns: 1fr; } }
</style>
</head>
<body>
<header>
  <h1>MolmoAct2 on this SO-101</h1>
  <p class="lead">Write what the arm should do. The table picture is camera 0 for the model, and the gripper picture is camera 1. The first Act loads the model before anything moves. Stop makes the arm go limp.</p>
</header>
<main>
  <section>
    <figure>
      <img id="img-table" alt="Table camera">
      <figcaption>
        Table
        <select id="cam-table">
          <option value="0" selected>OpenCV 0</option>
          <option value="1">OpenCV 1</option>
          <option value="2">OpenCV 2</option>
          <option value="3">OpenCV 3</option>
        </select>
      </figcaption>
    </figure>
    <figure>
      <img id="img-close" alt="Gripper camera">
      <figcaption>
        Gripper
        <select id="cam-close">
          <option value="0">OpenCV 0</option>
          <option value="1" selected>OpenCV 1</option>
          <option value="2">OpenCV 2</option>
          <option value="3">OpenCV 3</option>
        </select>
      </figcaption>
    </figure>
  </section>
  <section class="panel">
    <label>Joints, degrees</label>
    <table id="joints"></table>
    <label for="task">Prompt</label>
    <textarea id="task">Pick up the cube and put it down.</textarea>
    <label for="duration">Seconds</label>
    <input id="duration" type="number" min="1" max="45" value="20">
    <label class="check"><input id="ready" type="checkbox" checked> Move to the training pose first. The model expects the arm partly raised, not folded.</label>
    <label class="check"><input id="confirm" type="checkbox"> I am at the power switch, and the space in front of the arm is clear.</label>
    <div class="actions">
      <button class="quiet" id="refresh" type="button">Refresh cameras</button>
      <button class="primary" id="run" type="button" disabled>Act</button>
    </div>
    <div class="actions">
      <button class="stop" id="stop" type="button">Stop and go limp</button>
    </div>
    <p id="phase">idle</p>
    <pre id="log"></pre>
  </section>
</main>
<script>
const phase = document.querySelector("#phase");
const runButton = document.querySelector("#run");
const confirmBox = document.querySelector("#confirm");

function showBlob(img, blob) {
  if (img.dataset.url) URL.revokeObjectURL(img.dataset.url);
  const url = URL.createObjectURL(blob);
  img.dataset.url = url;
  img.src = url;
}

async function showCamera(img, index) {
  const response = await fetch("/api/frame?index=" + index + "&t=" + Date.now());
  if (!response.ok) {
    img.alt = "Camera " + index + " did not return a frame.";
    return;
  }
  showBlob(img, await response.blob());
}

async function refreshViews() {
  await showCamera(document.querySelector("#img-table"), document.querySelector("#cam-table").value);
  await showCamera(document.querySelector("#img-close"), document.querySelector("#cam-close").value);
  const response = await fetch("/api/state");
  const data = await response.json();
  if (!response.ok) {
    phase.textContent = data.error || "could not read the arm";
    phase.className = "warn";
    return;
  }
  paintJoints(data.joints || {});
}

function paintJoints(joints) {
  const table = document.querySelector("#joints");
  table.innerHTML = "";
  for (const [name, value] of Object.entries(joints)) {
    const row = document.createElement("tr");
    const label = document.createElement("td");
    const reading = document.createElement("td");
    label.textContent = name;
    reading.textContent = value;
    row.append(label, reading);
    table.appendChild(row);
  }
}

async function showLive() {
  for (const [slot, img] of [["table", "#img-table"], ["close", "#img-close"]]) {
    const response = await fetch("/api/live/" + slot + "?t=" + Date.now());
    if (response.ok) showBlob(document.querySelector(img), await response.blob());
  }
}

async function poll() {
  const data = await (await fetch("/api/status")).json();
  phase.className = data.error ? "warn" : "";
  phase.textContent = data.error || ((data.phase || "idle") + (data.inference_s ? "  ·  last think " + data.inference_s + "s" : ""));
  document.querySelector("#log").textContent = (data.log || []).join("\\n");
  runButton.disabled = data.running || !confirmBox.checked;
  document.querySelector("#refresh").disabled = data.running;
  if (data.running && data.joints && Object.keys(data.joints).length) paintJoints(data.joints);
  if (data.running) showLive();
}

document.querySelector("#refresh").onclick = refreshViews;
document.querySelector("#cam-table").onchange = refreshViews;
document.querySelector("#cam-close").onchange = refreshViews;
confirmBox.onchange = poll;
document.querySelector("#run").onclick = async () => {
  runButton.disabled = true;
  phase.textContent = "starting";
  const response = await fetch("/api/run", {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify({
      task: document.querySelector("#task").value,
      duration: Number(document.querySelector("#duration").value),
      table_index: Number(document.querySelector("#cam-table").value),
      close_index: Number(document.querySelector("#cam-close").value),
      ready: document.querySelector("#ready").checked,
      confirmed: confirmBox.checked
    })
  });
  const data = await response.json();
  if (!response.ok) phase.textContent = data.error || "could not start";
  poll();
};
document.querySelector("#stop").onclick = async () => {
  await fetch("/api/stop", {method: "POST"});
  poll();
};

refreshViews();
poll();
setInterval(poll, 1000);
</script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt: str, *args) -> None:
        return

    def _json(self, payload: dict | list, code: int = 200) -> None:
        raw = json.dumps(payload).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def _jpeg(self, payload: bytes) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def _read_body(self) -> dict:
        length = int(self.headers.get("Content-Length", "0"))
        if length == 0:
            return {}
        return json.loads(self.rfile.read(length))

    def do_GET(self) -> None:
        path = self.path.split("?", 1)[0]
        if path == "/":
            raw = PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)
            return
        if path == "/api/status":
            self._json({**status, "log": log_lines})
            return
        if path == "/api/state":
            if status["running"]:
                self._json({"joints": status["joints"]})
                return
            follower = None
            try:
                with preview_lock:
                    follower = make_robot(0, 1)
                    follower.bus.connect(handshake=False)
                    joints = read_joints(follower)
                    self._json({"joints": {name: round(value, 1) for name, value in joints.items()}})
            except Exception as exc:
                self._json({"error": str(exc), "joints": {}}, 500)
            finally:
                if follower is not None and follower.bus.is_connected:
                    follower.bus.disconnect(disable_torque=False)
            return
        if path == "/api/frame":
            if status["running"]:
                self._json({"error": "cameras are in use by the arm"}, 409)
                return
            query = self.path.split("?", 1)[1] if "?" in self.path else ""
            index = 0
            for part in query.split("&"):
                if part.startswith("index="):
                    index = int(part.split("=", 1)[1])
            try:
                self._jpeg(preview_jpeg(index))
            except Exception as exc:
                self._json({"error": str(exc)}, 500)
            return
        if path in ("/api/live/table", "/api/live/close"):
            payload = latest_jpeg[path.rsplit("/", 1)[-1]]
            if not payload:
                self._json({"error": "no frame yet"}, 404)
                return
            self._jpeg(payload)
            return
        self._json({"error": "not found"}, 404)

    def do_POST(self) -> None:
        global run_thread
        path = self.path.split("?", 1)[0]
        if path == "/api/stop":
            stop_event.set()
            status["phase"] = "stopping"
            note("stop requested")
            release(robot)
            self._json({"ok": True})
            return
        if path != "/api/run":
            self._json({"error": "not found"}, 404)
            return
        body = self._read_body()
        if status["running"] or (run_thread is not None and run_thread.is_alive()):
            self._json({"error": "already running"}, 409)
            return
        if not body.get("confirmed"):
            self._json({"error": "Confirm you are at the power switch first."}, 400)
            return
        if not str(body.get("task") or "").strip():
            self._json({"error": "Write a prompt first."}, 400)
            return
        run_thread = threading.Thread(target=run_policy, args=(body,), daemon=True)
        run_thread.start()
        self._json({"ok": True})


def main() -> None:
    RUN_DIR.mkdir(parents=True, exist_ok=True)
    server = ThreadingHTTPServer((HOST, PORT), Handler)
    print(f"open http://{HOST}:{PORT}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
