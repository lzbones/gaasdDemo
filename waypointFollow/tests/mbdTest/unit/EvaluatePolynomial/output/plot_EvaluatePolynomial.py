#!/Users/qingxu/.ai-env/bin/python3
"""EvaluatePolynomial 单元测试可视化: 多项式求值 (Horner 法)"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import json, os

script_dir = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(script_dir, '..', 'EvaluatePolynomial_cases.json')) as f:
    data = json.load(f)

polynomials = {}
for tc in data['test_cases']:
    c = tc['input']['coeffs']
    key = tuple(c)
    if key not in polynomials:
        polynomials[key] = {"coeffs": c, "points": [], "ids": []}
    polynomials[key]["points"].append((tc['input']['x'], tc['expected_output']['result']))
    polynomials[key]["ids"].append(tc['id'])

fig, ax = plt.subplots(figsize=(10, 6))
fig.suptitle('EvaluatePolynomial Unit Tests — Polynomial Curves', fontsize=14)
colors = ['blue', 'green', 'red', 'purple', 'orange']

for i, (key, poly) in enumerate(polynomials.items()):
    c = poly['coeffs']
    x = np.linspace(-0.5, 1.5, 300)
    y = c[0] + c[1]*x + c[2]*x**2 + c[3]*x**3
    label = f'c=[{c[0]},{c[1]},{c[2]},{c[3]}]'
    col = colors[i % len(colors)]
    ax.plot(x, y, '-', color=col, linewidth=2, label=label)
    for (px, py), tid in zip(poly['points'], poly['ids']):
        ax.plot(px, py, 'o', color=col, markersize=8, zorder=5)
        ax.annotate(tid, (px, py), textcoords="offset points",
                    xytext=(6, 6), fontsize=7, color=col)

ax.set_xlabel('x'); ax.set_ylabel('y = c₀ + c₁x + c₂x² + c₃x³')
ax.legend(fontsize=9); ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(script_dir, 'EvaluatePolynomial_plot.png'), dpi=150, bbox_inches='tight')
plt.close()
print("Plot saved.")
