#!/Users/qingxu/.ai-env/bin/python3
"""computeObstacleAffineConstraint C++ 函数级行为可视化（独立于 C++ 端，纯公式复现）。"""
import os
import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt

OUT_DIR = os.path.dirname(os.path.abspath(__file__))



def main():
    dx = np.linspace(2, 30, 200)
    alpha1 = alpha2 = 1.5; r = 4.0; vrx = -3.0
    B = dx**2 - r**2
    dotB = 2 * dx * vrx
    ddFree = 2 * vrx**2
    bound = -(ddFree + (alpha1 + alpha2) * dotB + alpha1 * alpha2 * B)
    betaA = -2 * dx
    plt.figure(figsize=(8, 4))
    plt.plot(dx, bound, label="bound")
    plt.plot(dx, betaA, label="betaA")
    plt.xlabel("dx [m]"); plt.ylabel("constraint coef / RHS")
    plt.title("computeObstacleAffineConstraint along dx")
    plt.legend(); plt.grid(True); plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, "computeObstacleAffineConstraint_response.png"), dpi=130)



if __name__ == "__main__":
    main()
