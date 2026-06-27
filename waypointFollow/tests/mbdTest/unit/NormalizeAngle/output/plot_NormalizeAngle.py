#!/Users/qingxu/.ai-env/bin/python3
"""NormalizeAngle 单元测试可视化: 角度归一化到 (-π, π]"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import json, os

script_dir = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(script_dir, '..', 'NormalizeAngle_cases.json')) as f:
    data = json.load(f)

def normalize_angle(a):
    while a > np.pi: a -= 2*np.pi
    while a <= -np.pi: a += 2*np.pi
    return a

fig, ax = plt.subplots(figsize=(10, 6))
fig.suptitle('NormalizeAngle Unit Tests — Angle Wrapping', fontsize=14)

x = np.linspace(-4*np.pi, 4*np.pi, 1000)
y = np.array([normalize_angle(v) for v in x])
ax.plot(x, y, 'b-', linewidth=1.5, label='normalizeAngle(θ)')
ax.plot(x, x, 'k--', alpha=0.2, linewidth=1, label='y = θ (unwrapped)')

for tc in data['test_cases']:
    inp = tc['input']['angle']
    out = tc['expected_output']['result']
    ax.plot(inp, out, 'ro', markersize=10, zorder=5)
    ax.annotate(tc['id'], (inp, out), textcoords="offset points",
                xytext=(8, 8), fontsize=8, color='red')

ax.axhline(y=np.pi, color='gray', linewidth=0.5, linestyle=':')
ax.axhline(y=-np.pi, color='gray', linewidth=0.5, linestyle=':')
ax.set_xlabel('Input Angle (rad)'); ax.set_ylabel('Normalized Angle (rad)')
ax.legend(); ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(script_dir, 'NormalizeAngle_plot.png'), dpi=150, bbox_inches='tight')
plt.close()
print("Plot saved.")
