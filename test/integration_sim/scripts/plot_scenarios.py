#!/usr/bin/env python3
"""
plot_scenarios.py
读取 test/integration_sim/output/*.csv，绘制三个集成仿真场景的时域曲线。
"""

import csv
import os
import sys
from pathlib import Path

try:
    import matplotlib.pyplot as plt
except ImportError as e:
    print("Error: matplotlib is required. Install with: pip install matplotlib")
    raise e

OUTPUT_DIR = Path(__file__).resolve().parent.parent / "output"
PLOT_DIR = Path(__file__).resolve().parent.parent / "output"

SCENARIOS = ["lead_brake", "cut_in", "cross_pedestrian", "swerve_obstacle"]


def read_csv(path: Path):
    rows = []
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({k: float(v) for k, v in row.items()})
    return rows


def plot_scenario(name: str, data: list, out_dir: Path):
    t = [r["t"] for r in data]
    ego_v = [r["ego_v"] for r in data]
    accel_orig = [r["accel_original"] for r in data]
    a_safe = [r["a_safe"] for r in data]
    delta_f_orig = [r["delta_f_original"] for r in data]
    delta_f_safe = [r["delta_f_safe"] for r in data]
    min_dist = [r["min_dist"] for r in data]

    fig, axes = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
    fig.suptitle(f"Integration Simulation: {name}", fontsize=14)

    # 纵向速度
    axes[0].plot(t, ego_v, "b-", label="ego_v")
    axes[0].set_ylabel("Speed (m/s)")
    axes[0].legend()
    axes[0].grid(True)

    # 纵向加速度：原始 vs 安全
    axes[1].plot(t, accel_orig, "g--", label="accel_original (waypointFollow)")
    axes[1].plot(t, a_safe, "r-", label="a_safe (cbf)")
    axes[1].set_ylabel("Acceleration (m/s^2)")
    axes[1].legend()
    axes[1].grid(True)

    # 前轮转角：原始 vs 安全
    axes[2].plot(t, [x * 180.0 / 3.141592653589793 for x in delta_f_orig], "g--", label="delta_f_original (waypointFollow)")
    axes[2].plot(t, [x * 180.0 / 3.141592653589793 for x in delta_f_safe], "m-", label="delta_f_safe (cbf)")
    axes[2].set_ylabel("Steering (deg)")
    axes[2].legend()
    axes[2].grid(True)

    # 最小障碍物距离
    axes[3].plot(t, min_dist, "k-", label="min_dist")
    axes[3].axhline(5.0, color="r", linestyle="--", label="safety radius = 5m")
    axes[3].set_ylabel("Distance (m)")
    axes[3].set_xlabel("Time (s)")
    axes[3].legend()
    axes[3].grid(True)

    out_path = out_dir / f"{name}.png"
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"Saved plot: {out_path}")


def main():
    PLOT_DIR.mkdir(parents=True, exist_ok=True)
    for scenario in SCENARIOS:
        csv_path = OUTPUT_DIR / f"{scenario}.csv"
        if not csv_path.exists():
            print(f"Warning: {csv_path} not found, skipping.")
            continue
        data = read_csv(csv_path)
        plot_scenario(scenario, data, PLOT_DIR)


if __name__ == "__main__":
    main()
