#!/Users/qingxu/.ai-env/bin/python3
"""CBF Closed-Loop Simulation Result Plotter.

Reads scenario CSV under tests/cppTest/Integration/<scenario>/<scenario>.csv
and produces a multi-panel PNG covering trajectory, distance, velocity
and original-vs-safe control commands.
"""

import csv
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_csv(csv_path):
    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = [{k: float(v) for k, v in r.items()} for r in reader]
    return rows


def plot_scenario(scenario_name, csv_path, png_path, safety_radius=5.0):
    rows = load_csv(csv_path)
    t = [r["t"] for r in rows]
    ego_x = [r["ego_x"] for r in rows]
    ego_y = [r["ego_y"] for r in rows]
    ob0_x = [r["ob0_x"] for r in rows]
    ob0_y = [r["ob0_y"] for r in rows]
    dist = [r["ob0_dist"] for r in rows]
    v = [r["ego_v"] for r in rows]
    a_orig = [r["a_orig"] for r in rows]
    a_safe = [r["a_safe"] for r in rows]
    df_orig = [r["df_orig"] for r in rows]
    df_safe = [r["df_safe"] for r in rows]
    feasible = [r["feasible"] for r in rows]

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle(f"CBF Closed-Loop Simulation: {scenario_name}", fontsize=14)

    ax = axes[0, 0]
    ax.plot(ego_x, ego_y, "b-", label="EV trajectory")
    ax.plot(ob0_x, ob0_y, "r--", label="Obstacle trajectory")
    ax.scatter([ego_x[0]], [ego_y[0]], color="blue", marker="o", label="EV start")
    ax.scatter([ob0_x[0]], [ob0_y[0]], color="red", marker="x", label="Obs start")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Trajectories (Global frame)")
    ax.legend(loc="best", fontsize=8)
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    ax.plot(t, dist, "g-", label="EV-Obs distance")
    ax.axhline(y=safety_radius, color="r", linestyle=":", label=f"safetyRadius={safety_radius:.1f}")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Distance (m)")
    ax.set_title("EV-Obstacle Distance over Time")
    ax.legend(loc="best", fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[1, 0]
    ax.plot(t, v, "b-", label="EV speed")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Speed (m/s)")
    ax.set_title("EV Speed")
    ax.legend(loc="best", fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    ax.plot(t, a_orig, "k--", label="a original")
    ax.plot(t, a_safe, "r-", label="a CBF-safe")
    ax_twin = ax.twinx()
    ax_twin.plot(t, df_orig, "k:", label="delta_f original", alpha=0.5)
    ax_twin.plot(t, df_safe, "b-", label="delta_f CBF-safe", alpha=0.7)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Acceleration (m/s^2)")
    ax_twin.set_ylabel("Steering angle (rad)")
    ax.set_title("Original vs CBF-Safe Control")
    ax.legend(loc="upper left", fontsize=7)
    ax_twin.legend(loc="upper right", fontsize=7)
    ax.grid(True, alpha=0.3)

    infeasible_steps = sum(1 for f in feasible if f < 0.5)
    fig.text(0.5, 0.01,
             f"min distance = {min(dist):.3f} m, infeasible steps = {infeasible_steps}",
             ha="center", fontsize=9)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig(png_path, dpi=120)
    plt.close(fig)
    print(f"Saved plot: {png_path}")


def main():
    base = os.path.join(os.path.dirname(__file__), "..", "tests", "cppTest", "Integration")
    scenarios = ["lead_brake", "cut_in", "cross_pedestrian"]
    for s in scenarios:
        csv_path = os.path.join(base, s, f"{s}.csv")
        png_path = os.path.join(base, s, f"{s}_plot.png")
        if not os.path.isfile(csv_path):
            print(f"Skip (missing): {csv_path}")
            continue
        plot_scenario(s, csv_path, png_path)


if __name__ == "__main__":
    main()
