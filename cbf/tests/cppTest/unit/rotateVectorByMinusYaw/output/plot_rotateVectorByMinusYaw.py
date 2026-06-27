#!/Users/qingxu/.ai-env/bin/python3
"""rotateVectorByMinusYaw C++ 函数级行为可视化（独立于 C++ 端，纯公式复现）。"""
import os
import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt

OUT_DIR = os.path.dirname(os.path.abspath(__file__))



def main():
    yaws = np.linspace(0, 2 * np.pi, 200)
    vx_global, vy_global = 1.0, 0.0
    vx_ego = np.cos(yaws) * vx_global + np.sin(yaws) * vy_global
    vy_ego = -np.sin(yaws) * vx_global + np.cos(yaws) * vy_global
    plt.figure(figsize=(8, 4))
    plt.plot(yaws, vx_ego, label="vxEgo")
    plt.plot(yaws, vy_ego, label="vyEgo")
    plt.xlabel("yaw [rad]"); plt.ylabel("ego frame component")
    plt.title("rotateVectorByMinusYaw response")
    plt.legend(); plt.grid(True); plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, "rotateVectorByMinusYaw_response.png"), dpi=130)



if __name__ == "__main__":
    main()
