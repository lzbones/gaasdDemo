#!/Users/qingxu/.ai-env/bin/python3
"""EvaluatePolynomialSlope 单元测试可视化: 多项式导数求值"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import json, os

script_dir = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(script_dir, '..', 'EvaluatePolynomialSlope_cases.json')) as f:
    data = json.load(f)

fig, axes = plt.subplots(1, 2, figsize=(14, 5))
fig.suptitle('EvaluatePolynomialSlope Unit Tests — Polynomial & Derivative', fontsize=14)

tc0 = data['test_cases'][0]
c = tc0['input']['coeffs']
x = np.linspace(-0.2, 1.2, 300)
y = c[0] + c[1]*x + c[2]*x**2 + c[3]*x**3
dy = c[1] + 2*c[2]*x + 3*c[3]*x**2

axes[0].plot(x, y, 'b-', linewidth=2, label=f'p(x), c={c}')
axes[0].set_title('Polynomial'); axes[0].set_xlabel('x'); axes[0].set_ylabel('p(x)')
axes[0].grid(True, alpha=0.3); axes[0].legend(fontsize=9)

axes[1].plot(x, dy, 'r-', linewidth=2, label="p'(x)")
for tc in data['test_cases']:
    px = tc['input']['x']
    py = tc['expected_output']['result']
    axes[1].plot(px, py, 'ko', markersize=8, zorder=5)
    axes[1].annotate(tc['id'], (px, py), textcoords="offset points",
                     xytext=(6, 6), fontsize=8)
axes[1].set_title('Derivative (slope)'); axes[1].set_xlabel('x'); axes[1].set_ylabel("p'(x)")
axes[1].grid(True, alpha=0.3); axes[1].legend(fontsize=9)

plt.tight_layout()
plt.savefig(os.path.join(script_dir, 'EvaluatePolynomialSlope_plot.png'), dpi=150, bbox_inches='tight')
plt.close()
print("Plot saved.")
