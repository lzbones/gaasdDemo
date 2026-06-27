#!/Users/qingxu/.ai-env/bin/python3
"""三次多项式拟合单元测试可视化: 绘制数据点与拟合曲线"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import os

test_cases = [
    {"id": "FIT_TC001", "desc": "y = 1 + 2x + 3x² + 4x³",
     "x": [0.0, 0.2, 0.4, 0.6, 0.8, 1.0],
     "y": [1.0, 1.552, 2.536, 4.144, 6.568, 10.0]},
    {"id": "FIT_TC002", "desc": "y = 3 + 5x (linear)",
     "x": [0.0, 0.2, 0.4, 0.6, 0.8, 1.0],
     "y": [3.0, 4.0, 5.0, 6.0, 7.0, 8.0]},
    {"id": "FIT_TC003", "desc": "y = 2x² (parabola)",
     "x": [0.0, 0.2, 0.4, 0.6, 0.8, 1.0],
     "y": [0.0, 0.08, 0.32, 0.72, 1.28, 2.0]},
]

fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
fig.suptitle('FitCubicPolynomial Unit Tests', fontsize=14)
colors = ['#1f77b4', '#ff7f0e', '#2ca02c']

for ax, tc, color in zip(axes, test_cases, colors):
    x_data = np.array(tc['x'])
    y_data = np.array(tc['y'])
    coeffs = np.polyfit(x_data, y_data, 3)
    x_fine = np.linspace(0, 1, 200)
    y_fit = np.polyval(coeffs, x_fine)
    ax.plot(x_fine, y_fit, '-', color=color, linewidth=2, label='Fitted curve')
    ax.plot(x_data, y_data, 'ko', markersize=8, zorder=5, label='Data points')
    residual = np.max(np.abs(np.polyval(coeffs, x_data) - y_data))
    ax.set_title(f'{tc["id"]}\n{tc["desc"]}', fontsize=10)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.text(0.05, 0.95, f'Max residual: {residual:.2e}',
            transform=ax.transAxes, fontsize=8, va='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

plt.tight_layout()
out_dir = os.path.dirname(os.path.abspath(__file__))
plt.savefig(os.path.join(out_dir, 'fitCubicPolynomial_plot.png'), dpi=150, bbox_inches='tight')
plt.close()
print("Plot saved.")
