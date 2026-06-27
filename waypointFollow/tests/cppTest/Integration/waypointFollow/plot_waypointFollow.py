#!/Users/qingxu/.ai-env/bin/python3
"""
路点跟踪MPC控制器闭环仿真可视化脚本
读取 C++ 测试输出的 CSV 数据绘制真实控制结果
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
    csv_path = os.path.join(build_dir, 'waypointFollow_sim.csv')
    if not os.path.exists(csv_path):
        print(f"CSV not found: {csv_path}")
        return

    steps, times = [], []
    ego_x, ego_y, ref_x, ref_y = [], [], [], []
    steers, accels, lat_errs = [], [], []

    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            steps.append(int(row['step']))
            times.append(float(row['time']))
            ego_x.append(float(row['egoX']))
            ego_y.append(float(row['egoY']))
            ref_x.append(float(row['refX']))
            ref_y.append(float(row['refY']))
            steers.append(float(row['steer']))
            accels.append(float(row['accel']))
            lat_errs.append(float(row['latErr']))

    times = np.array(times)
    steer_deg = np.degrees(steers)

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Waypoint Follow MPC Controller - Closed-Loop Simulation', fontsize=14)

    ax1 = axes[0, 0]
    ax1.plot(ego_x, ego_y, 'b-', linewidth=2, label='Actual Trajectory')
    ax1.plot(ref_x, ref_y, 'r--', linewidth=1.5, label='Reference Arc')
    ax1.set_xlabel('X Position (m)')
    ax1.set_ylabel('Y Position (m)')
    ax1.set_title('Vehicle Trajectory vs Reference')
    ax1.legend()
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)

    ax2 = axes[0, 1]
    ax2.plot(times, steer_deg, 'g-', linewidth=1.5)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Steering Wheel Angle (deg)')
    ax2.set_title('Steering Command History')
    ax2.grid(True, alpha=0.3)

    ax3 = axes[1, 0]
    ax3.plot(times, lat_errs, 'm-', linewidth=1.5)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Lateral Error (m)')
    ax3.set_title('Lateral Tracking Error')
    ax3.grid(True, alpha=0.3)

    ax4 = axes[1, 1]
    ax4.plot(times, accels, 'c-', linewidth=1.5)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Acceleration (m/s²)')
    ax4.set_title('Acceleration Command History')
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    save_path = os.path.join(script_dir, 'waypointFollow_plot.png')
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Plot saved to {save_path}")


if __name__ == '__main__':
    main()
