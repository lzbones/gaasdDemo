#!/Users/qingxu/.ai-env/bin/python3
"""
MBD WaypointFollow closed-loop simulation visualization
"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import csv
import os


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    build_dir = os.path.join(script_dir, '..', '..', '..', '..', 'build')
    csv_path = os.path.join(build_dir, 'WaypointFollow_mbd_sim.csv')
    if not os.path.exists(csv_path):
        print(f"CSV not found: {csv_path}")
        return

    steps = []
    ego_x, ego_y, ref_x, ref_y = [], [], [], []
    steers, accels, errs = [], [], []

    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            steps.append(int(row['step']))
            ego_x.append(float(row['egoX']))
            ego_y.append(float(row['egoY']))
            ref_x.append(float(row['refX']))
            ref_y.append(float(row['refY']))
            steers.append(float(row['steer']))
            accels.append(float(row['accel']))
            errs.append(float(row['latErr']))

    times = np.array(steps) * 0.1
    steer_deg = np.degrees(steers)

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('MBD WaypointFollow Closed-Loop Simulation', fontsize=14)

    axes[0, 0].plot(ego_x, ego_y, 'b-', linewidth=2, label='Ego (MBD)')
    axes[0, 0].plot(ref_x, ref_y, 'r--', linewidth=1.5, label='Reference Arc')
    axes[0, 0].set_xlabel('X (m)')
    axes[0, 0].set_ylabel('Y (m)')
    axes[0, 0].set_title('Vehicle Trajectory vs Reference')
    axes[0, 0].legend()
    axes[0, 0].set_aspect('equal')
    axes[0, 0].grid(True, alpha=0.3)

    axes[0, 1].plot(times, steer_deg, 'g-', linewidth=1.5)
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Steering Wheel Angle (deg)')
    axes[0, 1].set_title('Steering Command')
    axes[0, 1].grid(True, alpha=0.3)

    axes[1, 0].plot(times, errs, 'm-', linewidth=1.5)
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('Lateral Error (m)')
    axes[1, 0].set_title('Tracking Error')
    axes[1, 0].grid(True, alpha=0.3)

    axes[1, 1].plot(times, accels, 'c-', linewidth=1.5)
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_ylabel('Acceleration (m/s²)')
    axes[1, 1].set_title('Acceleration Command')
    axes[1, 1].grid(True, alpha=0.3)

    plt.tight_layout()
    out_path = os.path.join(script_dir, 'WaypointFollow_response.png')
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Plot saved to {out_path}")


if __name__ == '__main__':
    main()
