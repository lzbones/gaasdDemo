#!/usr/bin/env python3
"""
draw_architecture.py
使用 matplotlib 绘制工程总体架构图与模块内部逻辑图。
产物保存到 doc/images/。
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.font_manager as fm
from pathlib import Path

# 配置中文字体（优先 Arial Unicode MS，兼容 macOS）
plt.rcParams['font.sans-serif'] = ['PingFang HK', 'Heiti TC', 'Arial Unicode MS', 'SimHei']
plt.rcParams['axes.unicode_minus'] = False

OUT_DIR = Path(__file__).resolve().parent.parent / "doc" / "images"
OUT_DIR.mkdir(parents=True, exist_ok=True)


def draw_box(ax, x, y, w, h, text, color, fontsize=9, text_color="white"):
    rect = mpatches.FancyBboxPatch(
        (x - w / 2, y - h / 2), w, h,
        boxstyle="round,pad=0.02,rounding_size=0.15",
        facecolor=color, edgecolor="black", linewidth=1.2
    )
    ax.add_patch(rect)
    ax.text(x, y, text, ha="center", va="center", fontsize=fontsize,
            color=text_color, weight="bold", wrap=True)
    return rect


def draw_arrow(ax, x1, y1, x2, y2, label="", color="black"):
    ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle="->", color=color, lw=1.5))
    if label:
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2
        ax.text(mx, my + 0.25, label, ha="center", va="bottom",
                fontsize=8, color="dimgray", style="italic")


def setup_ax(ax, title):
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 10)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_title(title, fontsize=14, weight="bold", pad=10)


def overall_architecture():
    fig, ax = plt.subplots(figsize=(12, 6))
    setup_ax(ax, "GAASD 工程总体架构")

    # 上游输入
    draw_box(ax, 1.5, 8.0, 2.2, 1.0, "端到端模型\n6 个未来路点", "#4C78A8")
    draw_box(ax, 1.5, 6.0, 2.2, 1.0, "感知模块\n障碍物位置/速度", "#4C78A8")
    draw_box(ax, 1.5, 4.0, 2.2, 1.0, "车辆状态\n速度/航向/转角", "#4C78A8")

    # 控制层
    draw_box(ax, 5.0, 7.0, 2.4, 1.2, "waypointFollow\n路点跟踪 MPC", "#E45756")
    draw_box(ax, 5.0, 4.0, 2.4, 1.2, "cbf\n安全控制修正", "#F58518")

    # 下游输出
    draw_box(ax, 8.5, 5.5, 2.2, 1.2, "执行器/仿真模型\na_safe, δ_f_safe", "#72B7B2")

    draw_arrow(ax, 2.6, 8.0, 3.8, 7.3, "waypoints[6]")
    draw_arrow(ax, 2.6, 4.0, 3.8, 4.7, "EgoState")
    draw_arrow(ax, 2.6, 6.0, 3.8, 4.4, "ObstacleState[]")
    draw_arrow(ax, 6.2, 6.4, 6.2, 4.6, "frontWheelAngle\nacceleration")
    draw_arrow(ax, 6.2, 4.6, 7.6, 5.2, "a_safe, δ_f_safe")

    fig.tight_layout()
    fig.savefig(OUT_DIR / "overall_architecture.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {OUT_DIR / 'overall_architecture.png'}")


def waypointfollow_logic():
    fig, ax = plt.subplots(figsize=(14, 8))
    setup_ax(ax, "waypointFollow 模块内部逻辑")

    # 顶层
    draw_box(ax, 5.0, 9.0, 3.0, 0.9, "waypointFollow\n顶层入口", "#E45756")

    # 轨迹拟合分支
    draw_box(ax, 2.0, 7.0, 2.6, 0.8, "fitWaypointTrajectory\n路点轨迹拟合", "#4C78A8")
    draw_box(ax, 2.0, 5.2, 2.4, 0.8, "fitCubicPolynomial\n三次多项式拟合", "#72B7B2")
    draw_box(ax, 2.0, 3.4, 2.4, 0.8, "solveLinearSystem\n高斯消元", "#72B7B2")

    # MPC 分支
    draw_box(ax, 5.0, 7.0, 2.6, 0.8, "mpcOptimize\n梯度下降优化", "#4C78A8")
    draw_box(ax, 5.0, 5.2, 2.4, 0.8, "mpcComputeCost\n代价函数", "#72B7B2")
    draw_box(ax, 8.0, 5.2, 2.4, 0.8, "predictBicycleState\n自行车模型", "#B279A2")
    draw_box(ax, 5.0, 3.4, 2.4, 0.8, "evaluatePolynomial\n多项式求值", "#B279A2")
    draw_box(ax, 8.0, 3.4, 2.4, 0.8, "evaluatePolynomialSlope\n多项式斜率", "#B279A2")
    draw_box(ax, 2.0, 1.6, 2.4, 0.8, "normalizeAngle\n角度归一化", "#B279A2")

    # 输出
    draw_box(ax, 8.0, 7.0, 2.4, 0.8, "clampValue\n限幅", "#F58518")
    draw_box(ax, 8.0, 9.0, 2.4, 0.8, "输出\nsteeringWheelAngle\nfrontWheelAngle\nacceleration", "#54A24B")

    # arrows
    draw_arrow(ax, 5.0, 8.55, 2.0, 7.4)
    draw_arrow(ax, 5.0, 8.55, 5.0, 7.4)
    draw_arrow(ax, 5.0, 8.55, 8.0, 7.4)

    draw_arrow(ax, 2.0, 6.6, 2.0, 5.6)
    draw_arrow(ax, 2.0, 4.8, 2.0, 3.8)

    draw_arrow(ax, 5.0, 6.6, 5.0, 5.6)
    draw_arrow(ax, 5.0, 4.8, 5.0, 3.8)
    draw_arrow(ax, 6.2, 4.8, 7.6, 4.8)
    draw_arrow(ax, 8.0, 4.8, 8.0, 3.8)
    draw_arrow(ax, 5.0, 3.0, 3.2, 2.0)
    draw_arrow(ax, 8.0, 3.0, 8.0, 2.0)

    draw_arrow(ax, 8.0, 7.4, 8.0, 8.55)
    draw_arrow(ax, 5.0, 7.4, 6.2, 8.1)

    fig.tight_layout()
    fig.savefig(OUT_DIR / "waypointfollow_logic.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {OUT_DIR / 'waypointfollow_logic.png'}")


def cbf_logic():
    fig, ax = plt.subplots(figsize=(14, 8))
    setup_ax(ax, "cbf 模块内部逻辑")

    draw_box(ax, 5.0, 9.0, 3.0, 0.9, "cbfControlRevision\n顶层入口", "#F58518")

    draw_box(ax, 1.8, 7.0, 2.6, 0.8, "rotateVectorByMinusYaw\n加速度 ego-frame 旋转", "#4C78A8")

    draw_box(ax, 5.0, 7.0, 3.0, 0.8, "computeObstacleAffineConstraint\nHOCBF 仿射约束", "#4C78A8")
    draw_box(ax, 2.0, 5.0, 2.6, 0.9, "barrierSquared\nB = dx²+dy²-r²", "#72B7B2", fontsize=9)
    draw_box(ax, 5.0, 5.0, 2.6, 0.9, "barrier\n1stDerivative\ndB/dt", "#72B7B2", fontsize=9)
    draw_box(ax, 8.0, 5.0, 3.0, 0.9, "barrier\n2ndDerivativeFree\nd²B/dt² free", "#72B7B2", fontsize=9)

    draw_box(ax, 5.0, 3.0, 2.8, 0.8, "solveQpHildreth\n二维 QP 对偶坐标下降", "#E45756")
    draw_box(ax, 8.5, 3.0, 2.2, 0.8, "clampControlInput\n终值饱和", "#54A24B")

    draw_box(ax, 8.5, 9.0, 2.2, 0.8, "输出\na_safe, δ_f_safe\nfeasible", "#54A24B")

    draw_arrow(ax, 5.0, 8.55, 1.8, 7.4)
    draw_arrow(ax, 5.0, 8.55, 5.0, 7.4)
    draw_arrow(ax, 5.0, 6.6, 2.2, 5.4)
    draw_arrow(ax, 5.0, 6.6, 5.0, 5.4)
    draw_arrow(ax, 5.0, 6.6, 7.8, 5.4)
    draw_arrow(ax, 5.0, 6.6, 5.0, 3.4)
    draw_arrow(ax, 5.0, 2.6, 7.5, 2.6)
    draw_arrow(ax, 1.8, 6.6, 3.1, 5.4)
    draw_arrow(ax, 8.5, 3.4, 8.5, 8.55)
    draw_arrow(ax, 5.0, 8.55, 7.4, 8.55)

    fig.tight_layout()
    fig.savefig(OUT_DIR / "cbf_logic.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {OUT_DIR / 'cbf_logic.png'}")


def integration_dataflow():
    fig, ax = plt.subplots(figsize=(14, 5))
    setup_ax(ax, "集成仿真数据流")

    draw_box(ax, 1.5, 5.0, 2.2, 1.0, "场景配置\n ego / obstacle / event", "#4C78A8")
    draw_box(ax, 4.5, 5.0, 2.4, 1.0, "waypointFollow\n路点 → 原始控制", "#E45756")
    draw_box(ax, 7.5, 5.0, 2.0, 1.0, "cbf\n安全仲裁", "#F58518")
    draw_box(ax, 10.0, 5.0, 2.2, 1.0, "自行车模型\n状态积分", "#72B7B2")
    draw_box(ax, 10.0, 2.5, 2.2, 0.8, "CSV / PNG\n记录与可视化", "#54A24B")

    draw_arrow(ax, 2.6, 5.0, 3.3, 5.0, "waypoints[6]")
    draw_arrow(ax, 5.7, 5.0, 6.5, 5.0, "frontWheelAngle\nacceleration")
    draw_arrow(ax, 8.5, 5.0, 9.0, 5.0, "a_safe\nδ_f_safe")
    draw_arrow(ax, 10.0, 4.5, 10.0, 2.9, "states")

    fig.tight_layout()
    fig.savefig(OUT_DIR / "integration_dataflow.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {OUT_DIR / 'integration_dataflow.png'}")


if __name__ == "__main__":
    overall_architecture()
    waypointfollow_logic()
    cbf_logic()
    integration_dataflow()
    print("All architecture diagrams generated.")
