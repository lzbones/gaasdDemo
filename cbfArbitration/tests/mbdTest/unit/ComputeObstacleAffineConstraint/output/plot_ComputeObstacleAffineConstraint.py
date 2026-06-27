#!/Users/qingxu/.ai-env/bin/python3
"""
ComputeObstacleAffineConstraint 模块响应可视化脚本（不依赖 C++ 端，仅复现数学公式）。
"""
import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt
import os

OUT_DIR = os.path.dirname(os.path.abspath(__file__))



def main():
    dx = np.linspace(2, 30, 200)
    alpha1 = alpha2 = 1.5
    r = 4.0
    vrx = -3.0
    B = dx**2 - r**2
    dotB = 2 * dx * vrx
    ddFree = 2 * vrx**2
    bound = -(ddFree + (alpha1 + alpha2) * dotB + alpha1 * alpha2 * B)
    betaA = -2 * dx
    plt.figure(figsize=(8, 4))
    plt.plot(dx, bound, label="bound")
    plt.plot(dx, betaA, label="betaA")
    plt.xlabel("dx [m]"); plt.ylabel("constraint coef / RHS")
    plt.title("ComputeObstacleAffineConstraint along dx")
    plt.legend(); plt.grid(True); plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, "ComputeObstacleAffineConstraint_response.png"), dpi=130)



if __name__ == "__main__":
    main()
