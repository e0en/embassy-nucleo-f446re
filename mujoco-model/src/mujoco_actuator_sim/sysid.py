from __future__ import annotations

import argparse
import concurrent.futures
import csv
import json
import math
import os
import random
import struct
import tempfile
import time
import xml.etree.ElementTree as ET
from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path

import can
import mujoco
import numpy as np
from scipy.optimize import OptimizeResult, least_squares


MODEL_DIR = Path(__file__).resolve().parents[2]
DEFAULT_MODEL = MODEL_DIR / "actuator_standalone.xml"
DEFAULT_ACTUATOR_XML = MODEL_DIR / "actuator.xml"
DEFAULT_CAPTURE = MODEL_DIR / "sysid_capture.csv"
DEFAULT_TUNED_XML = MODEL_DIR / "actuator_tuned.xml"
DEFAULT_TUNED_JSON = MODEL_DIR / "actuator_sysid.json"
DEFAULT_PLOT = MODEL_DIR / "actuator_sysid_plots.png"

HOST_CAN_ID = 0x00
DEFAULT_MOTOR_CAN_ID = 0x0F

FEEDBACK_STATUS = 0x00

MOTION_ANGLE_MIN = -4.0 * math.pi
MOTION_ANGLE_MAX = 4.0 * math.pi
MOTION_VELOCITY_MIN = -50.0
MOTION_VELOCITY_MAX = 50.0
MOTION_TORQUE_MIN = -5.0
MOTION_TORQUE_MAX = 5.0
MOTION_KP_MIN = 0.0
MOTION_KP_MAX = 500.0
MOTION_KD_MIN = 0.0
MOTION_KD_MAX = 5.0

PARAMETER_NAMES = (
    "damping",
    "frictionloss",
    "armature",
    "delay_s",
    "kp",
    "kd",
)

TORQUE_LIMIT_NM = 1.5
LOWER_BOUNDS = np.array([1e-5, 1e-6, 1e-4, 0.0, 0.1, 1e-4])
UPPER_BOUNDS = np.array([20.0, 5.0, 1.0, 0.1, 2000.0, 50.0])
LOG_PARAM_MASK = np.array([True, True, True, False, True, True])
LOG_MULTI_START_SPREAD = 0.35
LINEAR_MULTI_START_SPREAD = np.array([0.0, 0.0, 0.0, 0.01, 0.0, 0.0])

ANGLE_MIN = -1000.0
ANGLE_MAX = 1000.0
VELOCITY_MIN = -500.0
VELOCITY_MAX = 500.0
TORQUE_MIN = -100.0
TORQUE_MAX = 100.0
TEMPERATURE_MIN = -100.0
TEMPERATURE_MAX = 100.0


@dataclass(frozen=True)
class PlantParams:
    damping: float
    frictionloss: float
    armature: float
    delay_s: float
    kp: float
    kd: float

    def as_array(self) -> np.ndarray:
        return np.array(
            [
                self.damping,
                self.frictionloss,
                self.armature,
                self.delay_s,
                self.kp,
                self.kd,
            ],
            dtype=float,
        )

    @classmethod
    def from_array(cls, values: np.ndarray) -> PlantParams:
        return cls(*[float(value) for value in values])


@dataclass(frozen=True)
class Sample:
    t: float
    phase: str
    command: float
    kp: float
    kd: float
    angle: float
    velocity: float
    torque: float
    temperature: float


def main() -> None:
    args = parse_args()
    if args.command == "collect":
        collect(args)
    elif args.command == "tune":
        tune_command(args)
    else:
        collect(args)
        tune_command(args)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Collect MotionControl CAN sysid data and tune the MuJoCo actuator model."
    )
    subparsers = parser.add_subparsers(dest="command")

    collect_parser = subparsers.add_parser("collect")
    add_collect_args(collect_parser)

    tune_parser = subparsers.add_parser("tune")
    add_tune_args(tune_parser)

    run_parser = subparsers.add_parser("run")
    add_collect_args(run_parser)
    add_tune_args(run_parser, include_capture=False)

    parser.set_defaults(command="run")
    add_collect_args(parser)
    add_tune_args(parser, include_capture=False)
    return parser.parse_args()


def add_collect_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--interface", default="socketcan")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--host-id", type=parse_u8, default=HOST_CAN_ID)
    parser.add_argument("--motor-id", type=parse_u8, default=DEFAULT_MOTOR_CAN_ID)
    parser.add_argument("--capture", type=Path, default=DEFAULT_CAPTURE)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--command-rate", type=float, default=100.0)
    parser.add_argument("--request-rate", type=float, default=100.0)
    parser.add_argument("--amplitude", type=float, default=0.5)
    parser.add_argument("--motion-kp", type=float, default=20.0)
    parser.add_argument("--motion-kd", type=float, default=0.5)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--yes", action="store_true")


def add_tune_args(parser: argparse.ArgumentParser, include_capture: bool = True) -> None:
    if include_capture:
        parser.add_argument("--capture", type=Path, default=DEFAULT_CAPTURE)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--actuator", type=Path, default=DEFAULT_ACTUATOR_XML)
    parser.add_argument("--output-xml", type=Path, default=DEFAULT_TUNED_XML)
    parser.add_argument("--output-json", type=Path, default=DEFAULT_TUNED_JSON)
    parser.add_argument("--output-plot", type=Path, default=DEFAULT_PLOT)
    parser.add_argument("--max-samples", type=int, default=1500)
    parser.add_argument("--max-nfev", type=int, default=200)
    parser.add_argument("--diff-step", type=float, default=0.05)
    parser.add_argument("--multi-start", type=int, default=12)
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument("--write", action="store_true")


def parse_u8(value: str) -> int:
    parsed = int(value, 0)
    if not 0 <= parsed <= 0xFF:
        raise argparse.ArgumentTypeError("must fit in u8")
    return parsed


def collect(args: argparse.Namespace) -> None:
    if not args.yes:
        raise SystemExit("refusing to drive motor without --yes")
    if args.duration <= 0.0:
        raise SystemExit("--duration must be positive")
    if args.command_rate <= 0.0 or args.request_rate <= 0.0:
        raise SystemExit("rates must be positive")
    if args.amplitude <= 0.0:
        raise SystemExit("--amplitude must be positive")
    if args.motion_kp <= 0.0 or args.motion_kd <= 0.0:
        raise SystemExit("--motion-kp and --motion-kd must be positive")

    command_period = 1.0 / args.command_rate
    request_period = 1.0 / args.request_rate
    command = 0.0
    phase = "idle"
    next_command_at = 0.0
    next_request_at = 0.0
    started_at = time.monotonic()

    args.capture.parent.mkdir(parents=True, exist_ok=True)
    with can.Bus(interface=args.interface, channel=args.channel) as bus:
        send_command(bus, args.motor_id, args.host_id, set_feedback_type(FEEDBACK_STATUS))
        send_command(bus, args.motor_id, args.host_id, enable_command())

        try:
            with args.capture.open("w", newline="") as file:
                writer = csv.DictWriter(
                    file,
                    fieldnames=[
                        "t",
                        "phase",
                        "command",
                        "kp",
                        "kd",
                        "angle",
                        "velocity",
                        "torque",
                        "temperature",
                    ],
                )
                writer.writeheader()

                while True:
                    now = time.monotonic()
                    elapsed = now - started_at
                    if elapsed >= args.duration:
                        break

                    if elapsed >= next_command_at:
                        phase, command = excitation_command(elapsed, args.duration, args.amplitude, args.seed)
                        send_command(
                            bus,
                            args.motor_id,
                            args.host_id,
                            motion_control(command, 0.0, 0.0, args.motion_kp, args.motion_kd),
                        )
                        next_command_at += command_period

                    if elapsed >= next_request_at:
                        send_command(bus, args.motor_id, args.host_id, request_status(args.host_id))
                        next_request_at += request_period

                    frame = bus.recv(timeout=0.001)
                    if frame is None:
                        continue
                    status = decode_status(frame, args.motor_id, args.host_id)
                    if status is None:
                        continue
                    status_elapsed = time.monotonic() - started_at
                    writer.writerow(
                        {
                            "t": f"{status_elapsed:.9f}",
                            "phase": phase,
                            "command": f"{command:.9g}",
                            "kp": f"{args.motion_kp:.9g}",
                            "kd": f"{args.motion_kd:.9g}",
                            "angle": f"{status[0]:.9g}",
                            "velocity": f"{status[1]:.9g}",
                            "torque": f"{status[2]:.9g}",
                            "temperature": f"{status[3]:.9g}",
                        }
                    )
        finally:
            send_command(
                bus,
                args.motor_id,
                args.host_id,
                motion_control(command, 0.0, 0.0, args.motion_kp, args.motion_kd),
            )
            send_command(bus, args.motor_id, args.host_id, stop_command())

    print(f"wrote capture: {args.capture}")


def excitation_command(elapsed: float, duration: float, amplitude: float, seed: int) -> tuple[str, float]:
    phase, phase_t, phase_duration = excitation_phase(elapsed, duration)
    envelope = min(elapsed / 1.0, 1.0)

    if phase == "settle":
        return phase, 0.0
    if phase == "small_prbs":
        return phase, envelope * prbs_value(seed, 0, phase_t, 0.12, 0.25 * amplitude)
    if phase == "medium_prbs":
        return phase, envelope * prbs_value(seed, 1, phase_t, 0.09, 0.6 * amplitude)
    if phase == "large_prbs":
        return phase, envelope * prbs_value(seed, 2, phase_t, 0.07, amplitude)
    if phase == "sweep":
        return phase, envelope * sweep_value(phase_t, phase_duration, amplitude)
    if phase == "pulses":
        return phase, envelope * pulse_value(phase_t, amplitude)
    return phase, 0.0


def excitation_phase(elapsed: float, duration: float) -> tuple[str, float, float]:
    phases = [
        ("settle", 0.08),
        ("small_prbs", 0.18),
        ("medium_prbs", 0.22),
        ("large_prbs", 0.18),
        ("sweep", 0.20),
        ("pulses", 0.14),
    ]
    start = 0.0
    for name, fraction in phases:
        phase_duration = duration * fraction
        end = start + phase_duration
        if elapsed < end:
            return name, elapsed - start, phase_duration
        start = end
    return "idle", 0.0, 0.0


def prbs_value(seed: int, phase_index: int, elapsed: float, hold_s: float, amplitude: float) -> float:
    bin_index = int(elapsed / hold_s)
    rng = random.Random((seed + 1) * 1009 + phase_index * 9176 + bin_index)
    return amplitude if rng.random() >= 0.5 else -amplitude


def sweep_value(elapsed: float, duration: float, amplitude: float) -> float:
    if duration <= 0.0:
        return 0.0
    x = min(max(elapsed / duration, 0.0), 1.0)
    if x < 0.5:
        return amplitude * (-1.0 + 4.0 * x)
    return amplitude * (3.0 - 4.0 * x)


def pulse_value(elapsed: float, amplitude: float) -> float:
    pattern = [
        (0.25, 0.0),
        (0.20, 0.4),
        (0.25, 0.0),
        (0.20, -0.4),
        (0.25, 0.0),
        (0.18, 0.7),
        (0.25, 0.0),
        (0.18, -0.7),
        (0.30, 0.0),
        (0.15, 1.0),
        (0.30, 0.0),
        (0.15, -1.0),
    ]
    cycle = sum(length for length, _ in pattern)
    t = elapsed % cycle
    cursor = 0.0
    for length, scale in pattern:
        cursor += length
        if t < cursor:
            return amplitude * scale
    return 0.0


def tune_command(args: argparse.Namespace) -> None:
    samples = load_capture(args.capture, args.max_samples)
    if len(samples) < 3:
        raise SystemExit("not enough capture samples")

    initial = initial_params(args.actuator, samples)
    tuned, stage_results = tune_staged(
        initial,
        samples,
        args.model,
        args.actuator,
        args.max_nfev,
        args.diff_step,
        args.multi_start,
        args.workers,
    )
    final_cost = stage_results[-1][1].cost
    sim = simulate(samples, args.model, args.actuator, tuned)
    metrics = fit_metrics(samples, sim)

    print_result(initial, tuned, final_cost, len(samples), stage_results, metrics)
    write_actuator_params(args.actuator, args.output_xml, tuned)
    write_sysid_json(args.output_json, tuned, final_cost, stage_results, metrics)
    write_phase_plots(args.output_plot, samples, sim)
    print(f"wrote tuned actuator XML: {args.output_xml}")
    print(f"wrote sysid parameters: {args.output_json}")
    print(f"wrote fit plots: {args.output_plot}")

    if args.write:
        write_actuator_params(args.actuator, args.actuator, tuned)
        print(f"updated {args.actuator}")


def load_capture(path: Path, max_samples: int) -> list[Sample]:
    with path.open(newline="") as file:
        rows = [
            Sample(
                t=float(row["t"]),
                phase=row.get("phase", ""),
                command=float(row["command"]),
                kp=float(row.get("kp", "nan")),
                kd=float(row.get("kd", "nan")),
                angle=float(row["angle"]),
                velocity=float(row["velocity"]),
                torque=float(row["torque"]),
                temperature=float(row["temperature"]),
            )
            for row in csv.DictReader(file)
        ]

    if not rows:
        return rows
    t0 = rows[0].t
    rows = [
        Sample(
            t=row.t - t0,
            phase=row.phase,
            command=row.command,
            kp=row.kp,
            kd=row.kd,
            angle=row.angle,
            velocity=row.velocity,
            torque=row.torque,
            temperature=row.temperature,
        )
        for row in rows
    ]
    indices = downsample_indices(len(rows), max_samples)
    return [rows[index] for index in indices]


def downsample_indices(size: int, max_samples: int) -> np.ndarray:
    if max_samples <= 0 or size <= max_samples:
        return np.arange(size)
    return np.unique(np.linspace(0, size - 1, max_samples, dtype=int))


def initial_params(actuator_path: Path, samples: list[Sample]) -> PlantParams:
    root = ET.parse(actuator_path).getroot()
    joint = required(root, ".//joint")
    position = required(root, ".//position")
    capture_kp = finite_positive_median(sample.kp for sample in samples)
    capture_kd = finite_positive_median(sample.kd for sample in samples)

    return PlantParams(
        damping=float(joint.attrib["damping"]),
        frictionloss=float(joint.attrib["frictionloss"]),
        armature=float(joint.attrib["armature"]),
        delay_s=0.0,
        kp=capture_kp if capture_kp is not None else float(position.attrib.get("kp", 20.0)),
        kd=capture_kd if capture_kd is not None else float(position.attrib.get("kv", 0.5)),
    )


def finite_positive_median(values: Iterable[float]) -> float | None:
    finite = [float(value) for value in values if math.isfinite(float(value)) and float(value) > 0.0]
    if not finite:
        return None
    return float(np.median(np.array(finite, dtype=float)))


def tune_stage(
    stage: str,
    base: PlantParams,
    samples: list[Sample],
    model_path: Path,
    actuator_path: Path,
    max_nfev: int,
    diff_step: float,
    multi_start: int,
    workers: int,
) -> OptimizeResult:
    stage_samples = select_stage_samples(samples, stage)
    mask = stage_mask(stage)
    starts = stage_starts(base, mask, max(multi_start, 1), stage)
    worker_count = resolved_worker_count(workers, len(starts))
    if worker_count <= 1:
        results = [
            run_stage_start(
                start,
                base,
                mask,
                stage_samples,
                model_path,
                actuator_path,
                max_nfev,
                diff_step,
            )
            for start in starts
        ]
    else:
        with concurrent.futures.ProcessPoolExecutor(max_workers=worker_count) as executor:
            futures = [
                executor.submit(
                    run_stage_start,
                    start,
                    base,
                    mask,
                    stage_samples,
                    model_path,
                    actuator_path,
                    max_nfev,
                    diff_step,
                )
                for start in starts
            ]
            results = [future.result() for future in futures]
    return min((stage_result(result) for result in results), key=lambda result: result.cost)


def resolved_worker_count(workers: int, start_count: int) -> int:
    if workers < 0:
        raise ValueError("--workers must be >= 0")
    if start_count <= 1:
        return 1
    if workers > 0:
        return min(workers, start_count)
    cpu_count = os.cpu_count() or 1
    return min(cpu_count, start_count)


def run_stage_start(
    start: np.ndarray,
    base: PlantParams,
    mask: np.ndarray,
    stage_samples: list[Sample],
    model_path: Path,
    actuator_path: Path,
    max_nfev: int,
    diff_step: float,
) -> dict[str, object]:
    low, high = packed_bounds(mask)
    result = least_squares(
        residuals,
        start,
        args=(base, mask, stage_samples, model_path, actuator_path),
        bounds=(low, high),
        diff_step=diff_step,
        loss="soft_l1",
        f_scale=1.0,
        ftol=1e-8,
        xtol=1e-10,
        gtol=1e-8,
        max_nfev=max_nfev,
        verbose=0,
    )
    return {
        "x": result.x,
        "cost": result.cost,
        "fun": result.fun,
        "nfev": result.nfev,
        "status": result.status,
        "message": result.message,
        "success": result.success,
    }


def stage_result(result: dict[str, object]) -> OptimizeResult:
    return OptimizeResult(
        x=result["x"],
        cost=result["cost"],
        fun=result["fun"],
        nfev=result["nfev"],
        status=result["status"],
        message=result["message"],
        success=result["success"],
    )


def tune_staged(
    initial: PlantParams,
    samples: list[Sample],
    model_path: Path,
    actuator_path: Path,
    max_nfev: int,
    diff_step: float,
    multi_start: int,
    workers: int,
) -> tuple[PlantParams, list[tuple[str, OptimizeResult]]]:
    params = initial
    results = []
    for stage in ("delay", "gains", "armature", "damping_friction"):
        result = tune_stage(
            stage,
            params,
            samples,
            model_path,
            actuator_path,
            max_nfev,
            diff_step,
            multi_start,
            workers,
        )
        params = replace_params(params, stage_mask(stage), result.x)
        results.append((stage, result))
        print(f"{stage}: cost={result.cost:.6g}, nfev={result.nfev}, status={result.status}")
    return params, results


def select_stage_samples(samples: list[Sample], stage: str) -> list[Sample]:
    if not any(sample.phase for sample in samples):
        return rebase_samples(samples)
    if stage == "gains":
        selected = [sample for sample in samples if sample.phase in {"small_prbs", "medium_prbs", "sweep"}]
    elif stage == "armature":
        selected = [sample for sample in samples if sample.phase in {"large_prbs", "pulses"}]
    elif stage == "damping_friction":
        selected = [
            sample
            for sample in samples
            if sample.phase in {"small_prbs", "medium_prbs", "sweep"}
        ]
    else:
        selected = [sample for sample in samples if sample.phase != "settle"]
    return rebase_samples(selected or samples)


def rebase_samples(samples: list[Sample]) -> list[Sample]:
    if not samples:
        return samples
    t0 = samples[0].t
    return [
        Sample(
            t=sample.t - t0,
            phase=sample.phase,
            command=sample.command,
            kp=sample.kp,
            kd=sample.kd,
            angle=sample.angle,
            velocity=sample.velocity,
            torque=sample.torque,
            temperature=sample.temperature,
        )
        for sample in samples
    ]


def stage_mask(stage: str) -> np.ndarray:
    if stage == "delay":
        return np.array([False, False, False, True, False, False])
    if stage == "gains":
        return np.array([False, False, False, False, True, True])
    if stage == "armature":
        return np.array([False, False, True, False, False, False])
    if stage == "damping_friction":
        return np.array([True, True, False, False, False, False])
    return np.array([True, True, True, True, True, True])


def packed_bounds(mask: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    low = LOWER_BOUNDS.copy()
    high = UPPER_BOUNDS.copy()
    low[LOG_PARAM_MASK] = np.log(low[LOG_PARAM_MASK])
    high[LOG_PARAM_MASK] = np.log(high[LOG_PARAM_MASK])
    return low[mask], high[mask]


def stage_starts(
    base: PlantParams,
    mask: np.ndarray,
    count: int,
    stage: str,
) -> list[np.ndarray]:
    low, high = packed_bounds(mask)
    base_values = clip_to_bounds(pack_optimizer_values(base)[mask], low, high)
    starts = [base_values]
    rng = random.Random(0x53595349 + sum(ord(char) for char in stage))
    spread = multi_start_spread(mask)
    for _ in range(count - 1):
        jitter = np.array([rng.uniform(-1.0, 1.0) for _ in range(mask.sum())])
        starts.append(clip_to_bounds(base_values + jitter * spread, low, high))
    return starts


def multi_start_spread(mask: np.ndarray) -> np.ndarray:
    spread = LINEAR_MULTI_START_SPREAD.copy()
    spread[LOG_PARAM_MASK] = LOG_MULTI_START_SPREAD
    return spread[mask]


def replace_params(base: PlantParams, mask: np.ndarray, optimizer_values: np.ndarray) -> PlantParams:
    low, high = packed_bounds(mask)
    packed = pack_optimizer_values(base)
    packed[mask] = clip_to_bounds(optimizer_values, low, high)
    return PlantParams.from_array(unpack_optimizer_values(packed))


def tune(
    initial: PlantParams,
    samples: list[Sample],
    model_path: Path,
    actuator_path: Path,
    max_nfev: int,
    diff_step: float,
) -> OptimizeResult:
    initial_values = pack_optimizer_values(initial)
    all_params = np.ones(len(PARAMETER_NAMES), dtype=bool)
    low, high = packed_bounds(all_params)
    return least_squares(
        residuals,
        clip_to_bounds(initial_values, low, high),
        args=(initial, all_params, samples, model_path, actuator_path),
        bounds=(low, high),
        diff_step=diff_step,
        loss="soft_l1",
        f_scale=1.0,
        ftol=1e-8,
        xtol=1e-10,
        gtol=1e-8,
        max_nfev=max_nfev,
        verbose=1,
    )


def pack_optimizer_values(params: PlantParams) -> np.ndarray:
    values = params.as_array()
    packed = values.copy()
    packed[LOG_PARAM_MASK] = np.log(packed[LOG_PARAM_MASK])
    return packed


def unpack_optimizer_values(values: np.ndarray) -> np.ndarray:
    unpacked = values.copy()
    unpacked[LOG_PARAM_MASK] = np.exp(unpacked[LOG_PARAM_MASK])
    return unpacked


def clip_to_bounds(values: np.ndarray, low: np.ndarray, high: np.ndarray) -> np.ndarray:
    return np.clip(values, low + 1e-12, high - 1e-12)


def residuals(
    values: np.ndarray,
    base: PlantParams,
    mask: np.ndarray,
    samples: list[Sample],
    model_path: Path,
    actuator_path: Path,
) -> np.ndarray:
    params = replace_params(base, mask, values)
    sim = simulate(samples, model_path, actuator_path, params)
    if not np.all(np.isfinite(sim)):
        return np.full(len(samples), 1e4, dtype=float)
    measured_angle = np.unwrap(np.array([sample.angle for sample in samples]))
    return weighted_residuals(measured_angle, sim[:, 0], 0.05)


def weighted_residuals(measured: np.ndarray, simulated: np.ndarray, floor: float) -> np.ndarray:
    scale = max(float(np.std(measured)), floor)
    return (simulated - measured) / scale


def fit_metrics(samples: list[Sample], sim: np.ndarray) -> dict[str, object]:
    if len(samples) == 0 or not np.all(np.isfinite(sim)):
        return {"simulation_finite": False}

    measured_angle = np.unwrap(np.array([sample.angle for sample in samples], dtype=float))
    measured_velocity = np.array([sample.velocity for sample in samples], dtype=float)
    simulated_angle = np.unwrap(sim[:, 0])
    simulated_velocity = sim[:, 1]
    phases = sorted({sample.phase for sample in samples if sample.phase})

    return {
        "simulation_finite": True,
        "all": metric_block(measured_angle, simulated_angle, measured_velocity, simulated_velocity),
        "phases": {
            phase: metric_block_for_indices(
                measured_angle,
                simulated_angle,
                measured_velocity,
                simulated_velocity,
                [index for index, sample in enumerate(samples) if sample.phase == phase],
            )
            for phase in phases
        },
    }


def write_phase_plots(path: Path, samples: list[Sample], sim: np.ndarray) -> None:
    if len(samples) == 0 or not np.all(np.isfinite(sim)):
        return

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    phases = sorted({sample.phase for sample in samples if sample.phase})
    if not phases:
        phases = ["all"]

    cols = 2
    rows = math.ceil(len(phases) / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(14, max(3.0 * rows, 4.0)), squeeze=False)
    measured_angle = np.unwrap(np.array([sample.angle for sample in samples], dtype=float))
    simulated_angle = np.unwrap(sim[:, 0])
    command = np.array([sample.command for sample in samples], dtype=float)
    times = np.array([sample.t for sample in samples], dtype=float)

    for axis, phase in zip(axes.flat, phases):
        indices = [index for index, sample in enumerate(samples) if phase == "all" or sample.phase == phase]
        if not indices:
            axis.set_visible(False)
            continue
        index_array = np.array(indices, dtype=int)
        phase_times = times[index_array] - times[index_array][0]
        phase_measured = measured_angle[index_array]
        phase_simulated = simulated_angle[index_array]
        phase_command = command[index_array]
        rmse_rad = rmse(phase_simulated - phase_measured)

        axis.plot(phase_times, phase_command, color="0.55", linewidth=1.0, label="command")
        axis.plot(phase_times, phase_measured, color="tab:blue", linewidth=1.2, label="measured")
        axis.plot(phase_times, phase_simulated, color="tab:orange", linewidth=1.2, label="simulated")
        axis.set_title(f"{phase}  angle RMSE={rmse_rad:.4g} rad")
        axis.set_xlabel("time [s]")
        axis.set_ylabel("angle [rad]")
        axis.grid(True, alpha=0.3)

    for axis in axes.flat[len(phases) :]:
        axis.set_visible(False)

    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=3)
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.95))
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=160)
    plt.close(fig)


def metric_block_for_indices(
    measured_angle: np.ndarray,
    simulated_angle: np.ndarray,
    measured_velocity: np.ndarray,
    simulated_velocity: np.ndarray,
    indices: list[int],
) -> dict[str, float]:
    index_array = np.array(indices, dtype=int)
    return metric_block(
        measured_angle[index_array],
        simulated_angle[index_array],
        measured_velocity[index_array],
        simulated_velocity[index_array],
    )


def metric_block(
    measured_angle: np.ndarray,
    simulated_angle: np.ndarray,
    measured_velocity: np.ndarray,
    simulated_velocity: np.ndarray,
) -> dict[str, float]:
    angle_error = simulated_angle - measured_angle
    velocity_error = simulated_velocity - measured_velocity
    return {
        "angle_rmse_rad": rmse(angle_error),
        "angle_mae_rad": mae(angle_error),
        "angle_max_abs_rad": max_abs(angle_error),
        "velocity_rmse_rad_s": rmse(velocity_error),
        "velocity_mae_rad_s": mae(velocity_error),
        "velocity_max_abs_rad_s": max_abs(velocity_error),
        "velocity_corr": correlation(measured_velocity, simulated_velocity),
    }


def command_segment_indices(samples: list[Sample], phase: str) -> list[list[int]]:
    segments = []
    segment: list[int] = []
    last_command = None
    for index, sample in enumerate(samples):
        if sample.phase != phase:
            if segment:
                segments.append(segment)
                segment = []
                last_command = None
            continue
        command = round(sample.command, 4)
        if last_command is not None and command != last_command:
            segments.append(segment)
            segment = []
        segment.append(index)
        last_command = command
    if segment:
        segments.append(segment)
    return segments


def rmse(error: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(error)))) if len(error) else math.nan


def mae(error: np.ndarray) -> float:
    return float(np.mean(np.abs(error))) if len(error) else math.nan


def max_abs(error: np.ndarray) -> float:
    return float(np.max(np.abs(error))) if len(error) else math.nan


def correlation(measured: np.ndarray, simulated: np.ndarray) -> float:
    if len(measured) < 2 or float(np.std(measured)) <= 1e-12 or float(np.std(simulated)) <= 1e-12:
        return math.nan
    return float(np.corrcoef(measured, simulated)[0, 1])


def simulate(
    samples: list[Sample],
    model_path: Path,
    actuator_path: Path,
    params: PlantParams,
) -> np.ndarray:
    with tempfile.TemporaryDirectory(prefix="actuator-sysid-") as tmp:
        tmp_dir = Path(tmp)
        temp_actuator = tmp_dir / "actuator.xml"
        temp_model = tmp_dir / model_path.name
        write_actuator_params(actuator_path, temp_actuator, params)
        write_plant_model(model_path, temp_model)

        model = mujoco.MjModel.from_xml_path(str(temp_model))
        data = mujoco.MjData(model)
        sample_times = np.array([sample.t for sample in samples])
        angle_refs = np.array([sample.command for sample in samples], dtype=float)
        outputs = np.zeros((len(samples), 3), dtype=float)

        data.qpos[0] = samples[0].angle
        data.qvel[0] = samples[0].velocity
        mujoco.mj_forward(model, data)

        index = 0
        while index < len(samples):
            angle_ref = float(
                np.interp(
                    data.time - params.delay_s,
                    sample_times,
                    angle_refs,
                    left=angle_refs[0],
                    right=angle_refs[-1],
                )
            )
            torque = motion_torque(angle_ref, float(data.qpos[0]), float(data.qvel[0]), params)
            data.qfrc_applied[0] = torque
            mujoco.mj_step(model, data)

            while index < len(samples) and data.time >= samples[index].t:
                outputs[index] = [float(data.qpos[0]), float(data.qvel[0]), torque]
                index += 1

    return outputs


def motion_torque(angle_ref: float, angle: float, velocity: float, params: PlantParams) -> float:
    torque = params.kp * (angle_ref - angle) - params.kd * velocity
    return float(np.clip(torque, -TORQUE_LIMIT_NM, TORQUE_LIMIT_NM))


def write_plant_model(source: Path, destination: Path) -> None:
    tree = ET.parse(source)
    root = tree.getroot()
    for tag in ("actuator",):
        for element in root.findall(tag):
            root.remove(element)
    sensor = root.find("sensor")
    if sensor is not None:
        for child in list(sensor):
            if child.tag == "actuatorfrc":
                sensor.remove(child)
    ET.indent(tree, space="  ")
    tree.write(destination, encoding="unicode")
    destination.write_text(destination.read_text() + "\n")


def read_status_payload(data: bytes) -> tuple[float, float, float, float]:
    return (
        u16_to_f32(int.from_bytes(data[0:2], "little"), ANGLE_MIN, ANGLE_MAX),
        u16_to_f32(int.from_bytes(data[2:4], "little"), VELOCITY_MIN, VELOCITY_MAX),
        u16_to_f32(int.from_bytes(data[4:6], "little"), TORQUE_MIN, TORQUE_MAX),
        u16_to_f32(int.from_bytes(data[6:8], "little"), TEMPERATURE_MIN, TEMPERATURE_MAX),
    )


def decode_status(
    frame: can.Message,
    motor_can_id: int,
    host_can_id: int,
) -> tuple[float, float, float, float] | None:
    if not frame.is_extended_id:
        return None
    arbitration_id = int(frame.arbitration_id)
    response_type = arbitration_id >> 24
    if response_type != 0x02:
        return None
    if ((arbitration_id >> 8) & 0xFF) != motor_can_id:
        return None
    if (arbitration_id & 0xFF) != host_can_id:
        return None
    if len(frame.data) < 8:
        return None
    return read_status_payload(bytes(frame.data[:8]))


def send_command(
    bus: can.BusABC,
    motor_can_id: int,
    host_can_id: int,
    msg: tuple[int, int, bytes],
) -> None:
    command_id, command_content, data = msg
    if command_id == 0x01:
        arbitration_id = (command_id << 24) | (command_content << 8) | motor_can_id
    else:
        arbitration_id = (command_id << 24) | (command_content << 16) | (host_can_id << 8) | motor_can_id
    frame = can.Message(
        arbitration_id=arbitration_id,
        data=data,
        is_extended_id=True,
    )
    bus.send(frame)


def set_feedback_type(feedback_type: int) -> tuple[int, int, bytes]:
    return 0x17, feedback_type, bytes(8)


def enable_command() -> tuple[int, int, bytes]:
    return 0x03, 0, bytes(8)


def stop_command() -> tuple[int, int, bytes]:
    return 0x04, 0, bytes(8)


def request_status(host_can_id: int) -> tuple[int, int, bytes]:
    return 0x15, host_can_id, bytes(8)


def motion_control(
    angle: float,
    velocity: float,
    torque: float,
    kp: float,
    kd: float,
) -> tuple[int, int, bytes]:
    torque_u16 = f32_to_u16(torque, MOTION_TORQUE_MIN, MOTION_TORQUE_MAX)
    data = bytearray(8)
    data[0:2] = f32_to_u16(angle, MOTION_ANGLE_MIN, MOTION_ANGLE_MAX).to_bytes(2, "little")
    data[2:4] = f32_to_u16(velocity, MOTION_VELOCITY_MIN, MOTION_VELOCITY_MAX).to_bytes(2, "little")
    data[4:6] = f32_to_u16(kp, MOTION_KP_MIN, MOTION_KP_MAX).to_bytes(2, "little")
    data[6:8] = f32_to_u16(kd, MOTION_KD_MIN, MOTION_KD_MAX).to_bytes(2, "little")
    return 0x01, torque_u16, bytes(data)


def u16_to_f32(value: int, low: float, high: float) -> float:
    return value / 65535.0 * (high - low) + low


def f32_to_u16(value: float, low: float, high: float) -> int:
    clipped = min(max(float(value), low), high)
    return round((clipped - low) / (high - low) * 65535.0)


def write_actuator_params(source: Path, destination: Path, params: PlantParams) -> None:
    tree = ET.parse(source)
    root = tree.getroot()
    joint = required(root, ".//joint")
    position = required(root, ".//position")

    joint.set("damping", format_param(params.damping))
    joint.set("frictionloss", format_param(params.frictionloss))
    joint.set("armature", format_param(params.armature))
    position.set("kp", format_param(params.kp))
    position.set("kv", format_param(params.kd))
    position.set(
        "forcerange",
        f"{format_param(-TORQUE_LIMIT_NM)} {format_param(TORQUE_LIMIT_NM)}",
    )

    ET.indent(tree, space="  ")
    tree.write(destination, encoding="unicode")
    destination.write_text(destination.read_text() + "\n")


def write_sysid_json(
    path: Path,
    params: PlantParams,
    cost: float,
    stage_results: list[tuple[str, OptimizeResult]],
    metrics: dict[str, object],
) -> None:
    path.write_text(
        json.dumps(
            json_safe(
                {
                    "damping": params.damping,
                    "frictionloss": params.frictionloss,
                    "armature": params.armature,
                    "delay_s": params.delay_s,
                    "kp": params.kp,
                    "kd": params.kd,
                    "torque_limit": TORQUE_LIMIT_NM,
                    "least_squares_cost": cost,
                    "stages": [
                        {
                            "name": name,
                            "cost": result.cost,
                            "nfev": result.nfev,
                            "status": result.status,
                            "message": result.message,
                        }
                        for name, result in stage_results
                    ],
                    "metrics": metrics,
                }
            ),
            indent=2,
        )
        + "\n"
    )


def required(root: ET.Element, pattern: str) -> ET.Element:
    element = root.find(pattern)
    if element is None:
        raise ValueError(f"missing XML element: {pattern}")
    return element


def parse_range(value: str) -> tuple[float, float]:
    lo, hi = value.split()
    return float(lo), float(hi)


def format_param(value: float) -> str:
    return f"{value:.6g}"


def print_result(
    initial: PlantParams,
    tuned: PlantParams,
    cost: float,
    sample_count: int,
    stage_results: list[tuple[str, OptimizeResult]],
    metrics: dict[str, object],
) -> None:
    print(f"samples: {sample_count}")
    print("\nstages:")
    for name, result in stage_results:
        print(
            f"  {name:16s} cost={result.cost:10.6g} "
            f"nfev={result.nfev:4d} status={result.status}"
        )
    print("\nparameters:")
    for name, before, after in zip(PARAMETER_NAMES, initial.as_array(), tuned.as_array()):
        print(f"  {name:18s} {before:10.6g} -> {after:10.6g}")
    print(f"  {'torque_limit':18s} {TORQUE_LIMIT_NM:10.6g} fixed")
    print(f"\nleast-squares cost: {cost:.6g}")
    print_metrics(metrics)
    print_bound_warnings(tuned)


def print_metrics(metrics: dict[str, object]) -> None:
    if not metrics.get("simulation_finite", False):
        print("\nfit metrics: simulation produced non-finite values")
        return
    all_metrics = metrics["all"]
    assert isinstance(all_metrics, dict)
    print("\nfit metrics:")
    print(
        "  all               "
        f"angle_rmse={all_metrics['angle_rmse_rad']:.6g} rad  "
        f"velocity_rmse={all_metrics['velocity_rmse_rad_s']:.6g} rad/s  "
        f"velocity_corr={all_metrics['velocity_corr']:.6g}"
    )

    phases = metrics["phases"]
    assert isinstance(phases, dict)
    for phase, phase_metrics in phases.items():
        assert isinstance(phase_metrics, dict)
        print(
            f"  {phase:17s} "
            f"angle_rmse={phase_metrics['angle_rmse_rad']:.6g} rad  "
            f"velocity_rmse={phase_metrics['velocity_rmse_rad_s']:.6g} rad/s"
        )

def json_safe(value: object) -> object:
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [json_safe(item) for item in value]
    if isinstance(value, tuple):
        return [json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def print_bound_warnings(params: PlantParams) -> None:
    values = params.as_array()
    warnings = []
    for name, value, lo, hi in zip(PARAMETER_NAMES, values, LOWER_BOUNDS, UPPER_BOUNDS):
        if value <= lo * 1.01:
            warnings.append(f"{name} near lower bound {lo:g}")
        elif value >= hi / 1.01:
            warnings.append(f"{name} near upper bound {hi:g}")
    if warnings:
        print("\nbound warnings:")
        for warning in warnings:
            print(f"  {warning}")
