#!/usr/bin/env python3
"""Plot AHRS static-test CSV logs.

Usage examples:
    python tools/plot_ahrs_csv.py
    python tools/plot_ahrs_csv.py --input build/jsbsim/simulation/jsbsim/Debug/ahrs_static_gyro.csv
    python tools/plot_ahrs_csv.py --input build/jsbsim/simulation/jsbsim/Debug/ahrs_static_c172x_reset00.csv
    python tools/plot_ahrs_csv.py --no-show --output-dir build/jsbsim/simulation/jsbsim/Debug
"""

from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path
from typing import Dict, List, Sequence

import matplotlib.pyplot as plt


DEFAULT_INPUT = Path("build/jsbsim/simulation/jsbsim/Debug/ahrs_static_gyro.csv")
DEFAULT_OUTPUT_NAME = "ahrs_static_gyro_plots.png"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate AHRS plots from the static-gyro CSV log."
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=DEFAULT_INPUT,
        help="Path to the AHRS CSV file.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Output folder for PNG. Default: same folder as CSV.",
    )
    parser.add_argument(
        "--output-name",
        type=str,
        default=DEFAULT_OUTPUT_NAME,
        help="Output PNG filename.",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not open the interactive plot window.",
    )
    parser.add_argument(
        "--max-plot-points",
        type=int,
        default=4000,
        help="Maximum number of samples rendered per curve (visual downsampling only).",
    )
    return parser.parse_args()


def _read_numeric_columns(csv_path: Path) -> Dict[str, List[float]]:
    with csv_path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError("CSV appears empty or malformed (missing header).")

        columns: Dict[str, List[float]] = {name: [] for name in reader.fieldnames}

        for row_idx, row in enumerate(reader, start=2):
            for key in columns:
                raw_value = row.get(key, "")
                if raw_value is None or raw_value == "":
                    raise ValueError(f"Missing value at row {row_idx}, column '{key}'.")
                try:
                    columns[key].append(float(raw_value))
                except ValueError as exc:
                    raise ValueError(
                        f"Invalid numeric value '{raw_value}' at row {row_idx}, column '{key}'."
                    ) from exc

    return columns


def _require_columns(columns: Dict[str, List[float]], required: List[str]) -> None:
    missing = [name for name in required if name not in columns]
    if missing:
        raise ValueError(f"CSV missing required columns: {', '.join(missing)}")


def _first_existing_column(columns: Dict[str, List[float]], candidates: Sequence[str]) -> str:
    for name in candidates:
        if name in columns:
            return name
    raise ValueError(f"CSV missing required columns: {', '.join(candidates)}")


def _decimate_pair(x: List[float], y: List[float], max_points: int) -> tuple[List[float], List[float]]:
    if max_points <= 0 or len(x) <= max_points:
        return x, y
    # Use evenly spaced indices and always include the last point so slow
    # drifts near the end are not hidden by slicing stride artifacts.
    n = len(x)
    denom = max_points - 1
    indices = [0]
    for i in range(1, max_points - 1):
        idx = int(round(i * (n - 1) / denom))
        if idx > indices[-1]:
            indices.append(idx)
    if indices[-1] != n - 1:
        indices.append(n - 1)
    return [x[i] for i in indices], [y[i] for i in indices]


def _unwrap_deg(values: Sequence[float]) -> List[float]:
    if not values:
        return []

    unwrapped = [float(values[0])]
    offset = 0.0
    prev = float(values[0])

    for i in range(1, len(values)):
        curr = float(values[i])
        delta = curr - prev
        if delta > 180.0:
            offset -= 360.0
        elif delta < -180.0:
            offset += 360.0
        unwrapped.append(curr + offset)
        prev = curr

    return unwrapped


def _align_angle_branch(reference: Sequence[float], target: Sequence[float]) -> List[float]:
    if not reference or not target:
        return list(target)

    # Choose an integer multiple of 360 deg that best aligns target to reference.
    delta0 = float(reference[0]) - float(target[0])
    k = int(round(delta0 / 360.0))
    shift = 360.0 * k
    return [float(v) + shift for v in target]


def _set_centered_ylim(ax: plt.Axes, center: float, series: Sequence[Sequence[float]]) -> None:
    peak = 0.0
    for values in series:
        for v in values:
            peak = max(peak, abs(float(v) - center))

    # Keep a minimal span so the chart remains readable when nearly constant.
    half_span = max(peak, 0.05)
    ax.set_ylim(center - half_span, center + half_span)


def _plot(columns: Dict[str, List[float]], title: str, max_plot_points: int) -> plt.Figure:
    t = columns["time_s"]

    roll_col = _first_existing_column(columns, ["est_roll_deg", "roll_deg"])
    pitch_col = _first_existing_column(columns, ["est_pitch_deg", "pitch_deg"])
    yaw_col = _first_existing_column(columns, ["est_yaw_deg", "yaw_deg"])

    has_true_euler = all(
        key in columns for key in ["true_roll_deg", "true_pitch_deg", "true_yaw_deg"]
    )

    fig, axes = plt.subplots(3, 2, figsize=(14, 12), constrained_layout=True)
    fig.suptitle(title, fontsize=14)

    def plot_series(ax: plt.Axes, y: List[float], label: str, **kwargs: object) -> None:
        t_plot, y_plot = _decimate_pair(t, y, max_plot_points)
        ax.plot(t_plot, y_plot, label=label, **kwargs)

    ax = axes[0][0]
    plot_series(ax, columns[roll_col], label="roll est [deg]", linewidth=0.9)
    if has_true_euler:
        plot_series(ax, columns["true_roll_deg"], label="roll true [deg]", linestyle="--", linewidth=1.0)
    ax.set_title("Roll")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("deg")
    ax.grid(True, alpha=0.3)
    ax.legend()

    ax = axes[1][0]
    plot_series(ax, columns[pitch_col], label="pitch est [deg]", linewidth=0.9)
    if has_true_euler:
        plot_series(ax, columns["true_pitch_deg"], label="pitch true [deg]", linestyle="--", linewidth=1.0)
    ax.set_title("Pitch")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("deg")
    ax.grid(True, alpha=0.3)
    ax.legend()

    yaw_est_plot = _unwrap_deg(columns[yaw_col])
    yaw_true_plot = _unwrap_deg(columns["true_yaw_deg"]) if has_true_euler else None
    if has_true_euler:
        yaw_true_plot = _align_angle_branch(yaw_est_plot, yaw_true_plot)

    ax = axes[2][0]
    plot_series(ax, yaw_est_plot, label="yaw est [deg]", linewidth=0.9)
    if has_true_euler:
        plot_series(ax, yaw_true_plot, label="yaw true [deg]", linestyle="--", linewidth=1.0)
    yaw_center = yaw_true_plot[0] if has_true_euler else yaw_est_plot[0]
    yaw_series = [yaw_est_plot, yaw_true_plot] if has_true_euler else [yaw_est_plot]
    _set_centered_ylim(ax, yaw_center, yaw_series)
    ax.set_title("Yaw")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("deg")
    ax.grid(True, alpha=0.3)
    ax.legend()

    ax = axes[0][1]
    plot_series(ax, columns["qw"], label="qw")
    plot_series(ax, columns["qx"], label="qx")
    plot_series(ax, columns["qy"], label="qy")
    plot_series(ax, columns["qz"], label="qz")
    ax.set_title("Quaternion Components")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("value")
    ax.grid(True, alpha=0.3)
    ax.legend()

    ax = axes[1][1]
    plot_series(ax, columns["bgx_rps"], label="bgx [rad/s]")
    plot_series(ax, columns["bgy_rps"], label="bgy [rad/s]")
    plot_series(ax, columns["bgz_rps"], label="bgz [rad/s]")
    ax.set_title("Estimated Gyro Bias")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("rad/s")
    ax.grid(True, alpha=0.3)
    ax.legend()

    ax = axes[2][1]
    plot_series(ax, columns["P00"], label="P00")
    plot_series(ax, columns["P11"], label="P11")
    plot_series(ax, columns["P22"], label="P22")
    plot_series(ax, columns["P33"], label="P33")
    plot_series(ax, columns["P44"], label="P44")
    plot_series(ax, columns["P55"], label="P55")
    ax.set_title("Covariance Diagonal")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("variance")
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=2)

    return fig


def main() -> int:
    args = parse_args()

    csv_path = args.input.resolve()
    if not csv_path.exists():
        print(f"ERROR: input CSV not found: {csv_path}")
        return 1

    output_dir = args.output_dir.resolve() if args.output_dir else csv_path.parent
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / args.output_name

    required = [
        "time_s",
        "qw",
        "qx",
        "qy",
        "qz",
        "bgx_rps",
        "bgy_rps",
        "bgz_rps",
        "P00",
        "P11",
        "P22",
        "P33",
        "P44",
        "P55",
    ]

    try:
        columns = _read_numeric_columns(csv_path)
        _require_columns(columns, required)
        _first_existing_column(columns, ["est_roll_deg", "roll_deg"])
        _first_existing_column(columns, ["est_pitch_deg", "pitch_deg"])
        _first_existing_column(columns, ["est_yaw_deg", "yaw_deg"])
    except ValueError as exc:
        print(f"ERROR: {exc}")
        return 1

    run_name = os.path.splitext(csv_path.name)[0]
    fig = _plot(columns, title=f"AHRS Analysis: {run_name}", max_plot_points=args.max_plot_points)
    fig.savefig(output_path, dpi=160)

    print(f"Plot saved: {output_path}")

    if not args.no_show:
        plt.show()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
