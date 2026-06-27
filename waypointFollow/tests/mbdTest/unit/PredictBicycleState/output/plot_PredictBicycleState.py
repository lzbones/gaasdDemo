#!/Users/qingxu/.ai-env/bin/python3
"""PredictBicycleState MBD 单元测试可视化: 自行车模型单步/多步预测"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import json, os

script_dir = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(script_dir, '..', 'PredictBicycleState_cases.json')) as f:
    data = json.load(f)

def predict(x, y, yaw, v, delta, accel, dt, L):
    xn = x + v * np.cos(yaw) * dt
    yn = y + v * np.sin(yaw) * dt
    yawn = yaw + v * np.tan(delta) / L * dt
    vn = v + accel * dt
    return xn, yn, yawn, vn

fig, axes = plt.subplots(1, 2, figsize=(14, 6))
fig.suptitle('PredictBicycleState (MBD) — Single & Multi-Step', fontsize=14)

colors = ['blue', 'green', 'red', 'purple', 'orange']
for i, tc in enumerate(data['test_cases']):
    inp = tc['input']
    x0, y0 = inp['x'], inp['y']
    yaw0, v0 = inp['yaw'], inp['v']
    delta, accel = inp['steeringAngle'], inp['acceleration']
    param = tc.get('param', {})
    dt, L = param.get('dt', 0.1), param.get('wheelbase', 2.85)
    xs, ys = [x0], [y0]
    x, y, yaw, v = x0, y0, yaw0, v0
    for _ in range(30):
        x, y, yaw, v = predict(x, y, yaw, v, delta, accel, dt, L)
        xs.append(x); ys.append(y)
    col = colors[i % len(colors)]
    axes[0].plot(xs, ys, '-o', color=col, markersize=2, linewidth=1.5, label=tc['id'])
    exp = tc['expected_output']
    axes[0].plot(exp['xNext'], exp['yNext'], 's', color=col, markersize=10, zorder=5)

axes[0].set_xlabel('X (m)'); axes[0].set_ylabel('Y (m)')
axes[0].set_title('Predicted Trajectories (30 steps)')
axes[0].legend(fontsize=8); axes[0].set_aspect('equal'); axes[0].grid(True, alpha=0.3)

for i, tc in enumerate(data['test_cases']):
    inp = tc['input']
    x, y, yaw, v = inp['x'], inp['y'], inp['yaw'], inp['v']
    p2 = tc.get('param', {})
    vs = [v]
    for _ in range(30):
        x, y, yaw, v = predict(x, y, yaw, v, inp['steeringAngle'],
                                inp['acceleration'], p2.get('dt', 0.1), p2.get('wheelbase', 2.85))
        vs.append(v)
    t = np.arange(len(vs)) * p2.get('dt', 0.1)
    axes[1].plot(t, vs, '-', color=colors[i%len(colors)], linewidth=1.5, label=tc['id'])
axes[1].set_xlabel('Time (s)'); axes[1].set_ylabel('Speed (m/s)')
axes[1].set_title('Speed Profiles'); axes[1].legend(fontsize=8); axes[1].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(os.path.join(script_dir, 'PredictBicycleState_plot.png'), dpi=150, bbox_inches='tight')
plt.close()
print("Plot saved.")
