#!/Users/qingxu/.ai-env/bin/python3
"""
RotateVectorByMinusYaw 模块响应可视化脚本（不依赖 C++ 端，仅复现数学公式）。
"""
import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt
import os

OUT_DIR = os.path.dirname(os.path.abspath(__file__))



def main():
    yaws = np.linspace(0, 2 * np.pi, 200)
    vx_global = 1.0
    vy_global = 0.0
    vx_ego = np.cos(yaws) * vx_global + np.sin(yaws) * vy_global
    vy_ego = -np.sin(yaws) * vx_global + np.cos(yaws) * vy_global
    plt.figure(figsize=(8, 4))
    plt.plot(yaws, vx_ego, label="vxEgo")
    plt.plot(yaws, vy_ego, label="vyEgo")
    plt.xlabel("yaw [rad]")
    plt.ylabel("ego frame component")
    plt.title("RotateVectorByMinusYaw response")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, "RotateVectorByMinusYaw_response.png"), dpi=130)



if __name__ == "__main__":
    main()
