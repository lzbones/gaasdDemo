#!/Users/qingxu/.ai-env/bin/python3
"""ClampValue 单元测试可视化: 钳位函数传递特性"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import json, os

script_dir = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(script_dir, '..', 'ClampValue_cases.json')) as f:
    data = json.load(f)

fig, ax = plt.subplots(figsize=(8, 6))
fig.suptitle('ClampValue Unit Tests — Transfer Function', fontsize=14)

x = np.linspace(-5, 15, 300)
y = np.clip(x, 0, 10)
ax.plot(x, y, 'b-', linewidth=2, label='clamp(x, 0, 10)')
ax.plot(x, x, 'k--', alpha=0.3, linewidth=1, label='y = x (unclamped)')

for tc in data['test_cases']:
    inp = tc['input']['value']
    out = tc['expected_output']['result']
    ax.plot(inp, out, 'ro', markersize=10, zorder=5)
    ax.annotate(tc['id'], (inp, out), textcoords="offset points",
                xytext=(8, 8), fontsize=8, color='red')

ax.set_xlabel('Input Value'); ax.set_ylabel('Output Value')
ax.legend(); ax.grid(True, alpha=0.3)
ax.axhline(y=0, color='gray', linewidth=0.5)
ax.axhline(y=10, color='gray', linewidth=0.5, linestyle=':')
plt.tight_layout()
plt.savefig(os.path.join(script_dir, 'ClampValue_plot.png'), dpi=150, bbox_inches='tight')
plt.close()
print("Plot saved.")
