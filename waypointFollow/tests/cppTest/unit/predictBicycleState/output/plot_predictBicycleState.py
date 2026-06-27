#!/Users/qingxu/.ai-env/bin/python3
"""自行车模型状态预测单元测试可视化: 多步轨迹仿真"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import os

def predict_bicycle(x, y, yaw, v, delta, accel, dt, L):
    xn = x + v * np.cos(yaw) * dt
    yn = y + v * np.sin(yaw) * dt
    yawn = yaw + v * np.tan(delta) / L * dt
    vn = v + accel * dt
    return xn, yn, yawn, vn

def simulate(x0, y0, yaw0, v0, delta, accel, dt, L, steps):
    xs, ys, yaws, vs = [x0], [y0], [yaw0], [v0]
    x, y, yaw, v = x0, y0, yaw0, v0
    for _ in range(steps):
        x, y, yaw, v = predict_bicycle(x, y, yaw, v, delta, accel, dt, L)
        xs.append(x); ys.append(y); yaws.append(yaw); vs.append(v)
    return xs, ys, yaws, vs

DT, L, N = 0.1, 2.85, 30
scenarios = [
    {"name": "Straight (v=10)", "delta": 0.0, "accel": 0.0, "v0": 10.0, "color": "blue"},
    {"name": "Left turn (δ=0.1)", "delta": 0.1, "accel": 0.0, "v0": 10.0, "color": "green"},
    {"name": "Right turn (δ=−0.1)", "delta": -0.1, "accel": 0.0, "v0": 10.0, "color": "red"},
    {"name": "Accelerating (a=2)", "delta": 0.05, "accel": 2.0, "v0": 5.0, "color": "purple"},
]

fig, axes = plt.subplots(1, 2, figsize=(14, 6))
fig.suptitle('PredictBicycleState Unit Tests — Multi-Step Trajectories', fontsize=14)

for s in scenarios:
    xs, ys, yaws, vs = simulate(0, 0, 0, s['v0'], s['delta'], s['accel'], DT, L, N)
    axes[0].plot(xs, ys, '-o', color=s['color'], markersize=3, linewidth=1.5, label=s['name'])
    t = np.arange(len(vs)) * DT
    axes[1].plot(t, vs, '-', color=s['color'], linewidth=1.5, label=s['name'])

axes[0].set_xlabel('X (m)'); axes[0].set_ylabel('Y (m)')
axes[0].set_title('Vehicle Trajectories'); axes[0].legend(fontsize=9)
axes[0].set_aspect('equal'); axes[0].grid(True, alpha=0.3)

axes[1].set_xlabel('Time (s)'); axes[1].set_ylabel('Speed (m/s)')
axes[1].set_title('Speed Profiles'); axes[1].legend(fontsize=9)
axes[1].grid(True, alpha=0.3)

plt.tight_layout()
out_dir = os.path.dirname(os.path.abspath(__file__))
plt.savefig(os.path.join(out_dir, 'predictBicycleState_plot.png'), dpi=150, bbox_inches='tight')
plt.close()
print("Plot saved.")
