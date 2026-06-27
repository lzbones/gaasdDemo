#!/Users/qingxu/.ai-env/bin/python3
"""WaypointFollow MBD 单步响应可视化: 直线/左弯/低速场景路点布局"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import json, os

script_dir = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(script_dir, '..', 'WaypointFollow_cases.json')) as f:
    data = json.load(f)

tcs = data['test_cases']
n = len(tcs)
cols = min(n, 3)
rows = (n + cols - 1) // cols
fig, axes = plt.subplots(rows, cols, figsize=(5*cols, 5*rows))
if n == 1:
    axes = np.array([axes])
axes = np.atleast_2d(axes)
fig.suptitle('WaypointFollow (MBD) Unit Tests — Waypoint Layout', fontsize=14)

for idx, tc in enumerate(tcs):
    r, c_idx = divmod(idx, cols)
    ax = axes[r][c_idx]
    if 'input' not in tc:
        ax.set_visible(False)
        continue
    wp = tc['input']['waypoints']
    wx = [w[0] if isinstance(w, list) else w['x'] for w in wp]
    wy = [w[1] if isinstance(w, list) else w['y'] for w in wp]
    ax.plot(wx, wy, 'ro-', markersize=8, linewidth=1.5, label='Waypoints')
    ax.plot(0, 0, 'bs', markersize=12, label='Ego')
    if max(abs(np.array(wy))) > 0.01:
        coeffs = np.polyfit(wx, wy, 3)
        xf = np.linspace(0, max(wx), 100)
        ax.plot(xf, np.polyval(coeffs, xf), 'g--', alpha=0.5, label='Fit')
    ax.set_title(f'{tc["id"]}: {tc["description"][:25]}', fontsize=9)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.legend(fontsize=7); ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

for idx in range(n, rows*cols):
    r, c_idx = divmod(idx, cols)
    axes[r][c_idx].set_visible(False)

plt.tight_layout()
plt.savefig(os.path.join(script_dir, 'WaypointFollow_unit_plot.png'), dpi=150, bbox_inches='tight')
plt.close()
print("Plot saved.")
