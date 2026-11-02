"""Read-only scan of a Feetech STS3215 bus.

Opens the serial port, pings motors, and prints id, model, torque, and
position. It never writes a register and never enables or disables torque.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import rustypot

FOLLOWER_NAMES = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper",
}

# Factory rate first. The rest are only tried when nothing answers.
BAUD_RATES = (1_000_000, 115_200, 500_000, 250_000, 57_600, 38_400, 19_200, 128_000)


def scan(port: str, timeout_s: float) -> dict:
    model_number, definition = rustypot.find_model("sts3215")
    ids = list(range(1, 11))
    bus = rustypot.Bus(port, BAUD_RATES[0], timeout_s, {motor_id: definition for motor_id in ids})
    try:
        found: dict[int, int] = {}
        baud_used = None
        for baud in BAUD_RATES:
            bus.set_baudrate(baud)
            # One ping per id. A broadcast ping on this adapter returns a single servo.
            answering = bus.scan(definition, ids=ids)
            if answering:
                found = {int(motor_id): int(model) for motor_id, model in answering.items()}
                baud_used = baud
                break
        motors = []
        if baud_used is not None:
            ids = sorted(found)
            torque = bus.sync_read_register(ids, definition.register("torque_enable"))
            position = bus.sync_read_register(ids, definition.register("present_position"))
            for motor_id in ids:
                motors.append(
                    {
                        "id": motor_id,
                        "name": FOLLOWER_NAMES.get(motor_id),
                        "model_number": found[motor_id],
                        "expected_model_number": model_number,
                        "torque_enabled": bool(torque.get(motor_id)),
                        "present_position_raw": position.get(motor_id),
                    }
                )
        return {
            "port": port,
            "baud": baud_used,
            "expected_ids": list(FOLLOWER_NAMES),
            "motors": motors,
        }
    finally:
        bus.close()


def bench_port() -> str | None:
    path = Path(__file__).resolve().parents[1] / "config" / "bench.json"
    if not path.is_file():
        return None
    with path.open() as handle:
        port = json.load(handle).get("port")
    return port if isinstance(port, str) and port else None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=bench_port(), help="Serial port. Defaults to config/bench.json.")
    parser.add_argument("--timeout", type=float, default=0.25)
    args = parser.parse_args()
    if not args.port:
        print("Pass --port or set port in config/bench.json.", file=sys.stderr)
        return 1
    try:
        report = scan(args.port, args.timeout)
    except Exception as exc:
        print(f"scan failed: {exc}", file=sys.stderr)
        return 1
    print(json.dumps(report, indent=2))
    missing = [motor_id for motor_id in FOLLOWER_NAMES if motor_id not in {m["id"] for m in report["motors"]}]
    if report["baud"] is None:
        print("No STS3215 answered. Check the power supply and the daisy-chain.", file=sys.stderr)
        return 2
    if missing:
        print(f"Missing follower ids: {missing}", file=sys.stderr)
        return 3
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
