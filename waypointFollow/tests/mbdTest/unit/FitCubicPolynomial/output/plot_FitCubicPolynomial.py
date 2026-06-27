#!/Users/qingxu/.ai-env/bin/python3
"""FitCubicPolynomial MBD 单元测试可视化: 数据点与拟合曲线"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import json, os

script_dir = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(script_dir, '..', 'FitCubicPolynomial_cases.json')) as f:
    data = json.load(f)

tcs = data['test_cases']
fig, axes = plt.subplots(1, len(tcs), figsize=(5*len(tcs), 4.5))
if len(tcs) == 1:
    axes = [axes]
fig.suptitle('FitCubicPolynomial (MBD) Unit Tests', fontsize=14)
colors = ['#1f77b4', '#ff7f0e', '#2ca02c']

for ax, tc, col in zip(axes, tcs, colors):
    xd = np.array(tc['input']['xData'])
    yd = np.array(tc['input']['yData'])
    coeffs = np.polyfit(xd, yd, 3)
    xf = np.linspace(xd.min(), xd.max(), 200)
    yf = np.polyval(coeffs, xf)
    ax.plot(xf, yf, '-', color=col, linewidth=2, label='Fitted')
    ax.plot(xd, yd, 'ko', markersize=8, zorder=5, label='Data')
    ax.set_title(f'{tc["id"]}: {tc["description"][:30]}', fontsize=10)
    ax.set_xlabel('x'); ax.set_ylabel('y')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(os.path.join(script_dir, 'FitCubicPolynomial_plot.png'), dpi=150, bbox_inches='tight')
plt.close()
print("Plot saved.")
