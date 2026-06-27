#!/Users/qingxu/.ai-env/bin/python3
"""computeBarrierSquared C++ 函数级行为可视化（独立于 C++ 端，纯公式复现）。"""
import os
import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt

OUT_DIR = os.path.dirname(os.path.abspath(__file__))



def main():
    dx = np.linspace(-15, 15, 200); r = 4.0
    plt.figure(figsize=(8, 4))
    for dy in (0.0, 2.0, 4.0):
        plt.plot(dx, dx**2 + dy**2 - r**2, label="dy=" + str(dy))
    plt.axhline(0, color="k", linewidth=0.6)
    plt.xlabel("dx [m]"); plt.ylabel("Barrier B")
    plt.title("computeBarrierSquared (r_safe=4)")
    plt.legend(); plt.grid(True); plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, "computeBarrierSquared_response.png"), dpi=130)



if __name__ == "__main__":
    main()
