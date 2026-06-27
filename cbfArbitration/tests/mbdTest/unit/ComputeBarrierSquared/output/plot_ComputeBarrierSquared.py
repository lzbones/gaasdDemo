#!/Users/qingxu/.ai-env/bin/python3
"""
ComputeBarrierSquared 模块响应可视化脚本（不依赖 C++ 端，仅复现数学公式）。
"""
import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt
import os

OUT_DIR = os.path.dirname(os.path.abspath(__file__))



def main():
    dx = np.linspace(-15, 15, 200)
    r = 4.0
    for dy in (0.0, 2.0, 4.0):
        plt.plot(dx, dx**2 + dy**2 - r**2, label=f"dy={{dy}}".format(dy=dy))
    plt.axhline(0, color="k", linewidth=0.6)
    plt.xlabel("dx [m]"); plt.ylabel("Barrier B")
    plt.title("ComputeBarrierSquared (r_safe=4)")
    plt.legend(); plt.grid(True); plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, "ComputeBarrierSquared_response.png"), dpi=130)



if __name__ == "__main__":
    main()
