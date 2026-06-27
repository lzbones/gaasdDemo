#!/Users/qingxu/.ai-env/bin/python3
"""路点跟踪控制器单步响应可视化: 直线/左弯/右弯/低速场景"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import os

test_cases = [
    {"id": "TC001", "desc": "Straight",
     "wpX": [10,20,30,40,50,60], "wpY": [0,0,0,0,0,0],
     "steer_dir": "≈0", "color": "blue"},
    {"id": "TC002", "desc": "Left curve",
     "wpX": [7.5,15,22.5,30,37.5,45], "wpY": [0.2,0.8,1.8,3.2,5.0,7.2],
     "steer_dir": ">0", "color": "green"},
    {"id": "TC003", "desc": "Right curve",
     "wpX": [7.5,15,22.5,30,37.5,45], "wpY": [-0.2,-0.8,-1.8,-3.2,-5.0,-7.2],
     "steer_dir": "<0", "color": "red"},
    {"id": "TC004", "desc": "Low speed hold",
     "wpX": [0.1,0.2,0.3,0.4,0.5,0.6], "wpY": [0,0,0,0,0,0],
     "steer_dir": "hold", "color": "gray"},
]

fig, axes = plt.subplots(2, 2, figsize=(12, 10))
fig.suptitle('WaypointFollow Unit Tests — Single-Step Scenarios', fontsize=14)

for ax, tc in zip(axes.flat, test_cases):
    wx = np.array(tc['wpX'])
    wy = np.array(tc['wpY'])
    ax.plot(wx, wy, 'ro-', markersize=8, linewidth=1.5, label='Waypoints')
    ax.plot(0, 0, 'bs', markersize=12, label='Ego (origin)')
    ax.arrow(0, 0, wx[0]*0.3, wy[0]*0.3, head_width=0.3, head_length=0.2,
             fc=tc['color'], ec=tc['color'], alpha=0.6)
    coeffs = np.polyfit(wx, wy, 3) if max(abs(wy)) > 0.01 else [0,0,0,0]
    if max(abs(wy)) > 0.01:
        xf = np.linspace(0, wx[-1], 100)
        yf = np.polyval(coeffs, xf)
        ax.plot(xf, yf, '--', color=tc['color'], alpha=0.5, label='Fitted trajectory')
    ax.set_title(f'{tc["id"]}: {tc["desc"]} (steer {tc["steer_dir"]})', fontsize=11)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

plt.tight_layout()
out_dir = os.path.dirname(os.path.abspath(__file__))
plt.savefig(os.path.join(out_dir, 'waypointFollow_unit_plot.png'), dpi=150, bbox_inches='tight')
plt.close()
print("Plot saved.")
