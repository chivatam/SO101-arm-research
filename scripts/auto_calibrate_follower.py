"""Automatic SO-101 follower calibration.

Drives each joint into its mechanical stops at low speed and saves the same
calibration file LeRobot uses after ``lerobot-calibrate``. The arm moves on
its own. Run a read-only status check with no extra flags. Motion starts only
with ``--confirm-start-pose``, after the arm is folded and the table is clear.
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

import draccus

from lerobot.motors import Motor, MotorNormMode, SerialMotorsBus
from lerobot.motors.feetech.auto_calibration import AutoCalibrationConfig, auto_calibrate
from lerobot.motors.motors_bus import MotorCalibration
from lerobot.robots.so_follower.auto_calibration_plan import SO_ARM_PLAN, SO_ARM_START_POSE
from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, ROBOTS

PORT = "/dev/tty.usbmodem5C821107241"
ROBOT_ID = "so101_follower"
ROBOT_NAME = "so_follower"


def make_bus(port: str) -> SerialMotorsBus:
    bus = SerialMotorsBus(
        port=port,
        motors={
            "shoulder_pan": Motor(1, "sts3215", MotorNormMode.DEGREES),
            "shoulder_lift": Motor(2, "sts3215", MotorNormMode.DEGREES),
            "elbow_flex": Motor(3, "sts3215", MotorNormMode.DEGREES),
            "wrist_flex": Motor(4, "sts3215", MotorNormMode.DEGREES),
            "wrist_roll": Motor(5, "sts3215", MotorNormMode.DEGREES),
            "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
        },
    )
    # The adapter misses short 50 ms reads. A quarter second still returns as soon as a servo answers.
    bus.default_timeout = 250
    return bus


def calibration_path(robot_id: str) -> Path:
    return HF_LEROBOT_CALIBRATION / ROBOTS / ROBOT_NAME / f"{robot_id}.json"


def save_calibration(calibration: dict[str, MotorCalibration], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as handle, draccus.config_type("json"):
        draccus.dump(calibration, handle, indent=4)


def print_status(bus: SerialMotorsBus) -> None:
    positions = bus.sync_read("Present_Position", normalize=False, num_retry=2)
    torque = bus.sync_read("Torque_Enable", normalize=False, num_retry=2)
    print(f"{'joint':<16} {'id':>3} {'raw':>6} {'torque':>7}")
    for name, motor in bus.motors.items():
        print(f"{name:<16} {motor.id:>3} {positions[name]:>6} {torque[name]:>7}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=PORT)
    parser.add_argument("--robot-id", default=ROBOT_ID)
    parser.add_argument("--velocity", type=int, default=150, help="Encoder steps per second. 4096 is one turn.")
    parser.add_argument("--torque-limit", type=int, default=400, help="Tenths of a percent of maximum torque.")
    parser.add_argument(
        "--confirm-start-pose",
        action="store_true",
        help="The arm is folded, the space is clear, and it may drive to its end stops.",
    )
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    logging.getLogger("lerobot").setLevel(logging.INFO)

    bus = make_bus(args.port)
    bus.connect()
    try:
        print_status(bus)
        if not args.confirm_start_pose:
            print()
            print(SO_ARM_START_POSE)
            print()
            print("Status only. The arm was not moved. Re-run with --confirm-start-pose once it is folded.")
            return 0

        print()
        print(SO_ARM_START_POSE)
        print()
        print(
            f"Moving at {args.velocity} steps/s "
            f"({args.velocity * 360 / 4096:.1f} deg/s) with torque limit {args.torque_limit}."
        )
        print("Power switch in reach. Ctrl-C stops the run and turns torque off; the arm can drop.")
        calibration = auto_calibrate(
            bus,
            SO_ARM_PLAN,
            AutoCalibrationConfig(velocity=args.velocity, torque_limit=args.torque_limit),
        )
        for name in bus.motors:
            bus.write("Lock", name, 0, normalize=False, num_retry=3)
        bus.write_calibration(calibration)
        path = calibration_path(args.robot_id)
        save_calibration(calibration, path)
        print(f"Calibration saved to {path}")
        return 0
    finally:
        bus.disconnect(disable_torque=True)


if __name__ == "__main__":
    raise SystemExit(main())
