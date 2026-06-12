from __future__ import annotations

import argparse
import importlib
import math
import os
import platform
import sys
import time
from pathlib import Path

import mujoco
import numpy as np


DEFAULT_MODEL = Path(__file__).resolve().parents[2] / "actuator.xml"
MJPYTHON_REEXEC_ENV = "MUJOCO_ACTUATOR_SIM_MJPYTHON"


def main() -> None:
    args = parse_args()
    reexec_with_mjpython_if_needed(args)
    model = mujoco.MjModel.from_xml_path(str(args.model))
    data = mujoco.MjData(model)

    if args.no_viewer:
        run(model, data, args, viewer=None)
        return

    viewer_module = importlib.import_module("mujoco.viewer")

    with viewer_module.launch_passive(model, data) as viewer:
        run(model, data, args, viewer=viewer)


def reexec_with_mjpython_if_needed(args: argparse.Namespace) -> None:
    if args.no_viewer or platform.system() != "Darwin":
        return
    if os.environ.get(MJPYTHON_REEXEC_ENV) == "1":
        return

    mjpython = Path(sys.executable).with_name("mjpython")
    if not mjpython.exists():
        return

    os.environ[MJPYTHON_REEXEC_ENV] = "1"
    os.execv(
        str(mjpython),
        [str(mjpython), "-c", "from mujoco_actuator_sim import main; main()", *sys.argv[1:]],
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Drive the minimal actuator MuJoCo model.")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--mode", choices=["hold", "step", "sine", "chirp"], default="sine")
    parser.add_argument("--amplitude", type=float, default=0.5)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--frequency", type=float, default=1.0)
    parser.add_argument("--chirp-start", type=float, default=0.1)
    parser.add_argument("--chirp-end", type=float, default=5.0)
    parser.add_argument("--step-time", type=float, default=1.0)
    parser.add_argument("--print-rate", type=float, default=20.0)
    parser.add_argument("--no-viewer", action="store_true")
    return parser.parse_args()


def run(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    args: argparse.Namespace,
    viewer: object | None,
) -> None:
    dt = model.opt.timestep
    started_at = time.monotonic()
    next_print_at = 0.0
    print_period = 1.0 / args.print_rate if args.print_rate > 0.0 else math.inf

    while should_continue(args, viewer, time.monotonic() - started_at):
        loop_started_at = time.monotonic()
        t = loop_started_at - started_at
        data.ctrl[0] = clipped_command(model, command_at(args, t))
        mujoco.mj_step(model, data)

        if t >= next_print_at:
            print_status(model, data, t)
            next_print_at += print_period

        if viewer is not None:
            viewer.sync()

        sleep_s = dt - (time.monotonic() - loop_started_at)
        if sleep_s > 0.0:
            time.sleep(sleep_s)


def should_continue(args: argparse.Namespace, viewer: object | None, elapsed_s: float) -> bool:
    if args.duration > 0.0 and elapsed_s >= args.duration:
        return False
    if viewer is None:
        return True
    return viewer.is_running()


def clipped_command(model: mujoco.MjModel, command: float) -> float:
    if model.actuator_ctrllimited[0]:
        lo, hi = model.actuator_ctrlrange[0]
        command = float(np.clip(command, lo, hi))
    return command


def command_at(args: argparse.Namespace, elapsed_s: float) -> float:
    if args.mode == "hold":
        return args.amplitude
    if args.mode == "step":
        return args.amplitude if elapsed_s >= args.step_time else 0.0
    if args.mode == "chirp":
        return args.amplitude * math.sin(chirp_phase(args, elapsed_s))
    return args.amplitude * math.sin(2.0 * math.pi * args.frequency * elapsed_s)


def chirp_phase(args: argparse.Namespace, elapsed_s: float) -> float:
    if args.duration <= 0.0 or args.chirp_start <= 0.0 or args.chirp_end <= args.chirp_start:
        return 0.0
    sweep_ratio = args.chirp_end / args.chirp_start
    exponent = min(elapsed_s, args.duration) / args.duration
    return (
        2.0
        * math.pi
        * args.chirp_start
        * args.duration
        / math.log(sweep_ratio)
        * (sweep_ratio**exponent - 1.0)
    )


def print_status(model: mujoco.MjModel, data: mujoco.MjData, elapsed_s: float) -> None:
    angle = sensor_value(model, data, "actuator_angle")
    velocity = sensor_value(model, data, "actuator_velocity")
    torque = sensor_value(model, data, "actuator_torque")
    print(
        f"t={elapsed_s:7.3f} ctrl={data.ctrl[0]: .4f} "
        f"angle={angle: .4f} velocity={velocity: .4f} torque={torque: .4f}",
        flush=True,
    )


def sensor_value(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> float:
    sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
    if sensor_id < 0:
        return math.nan
    return float(data.sensordata[model.sensor_adr[sensor_id]])
