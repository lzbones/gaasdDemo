#!/Users/qingxu/.ai-env/bin/python3
"""
ComputeBarrierFirstDerivative 模块响应可视化脚本（不依赖 C++ 端，仅复现数学公式）。
"""
import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt
import os

OUT_DIR = os.path.dirname(os.path.abspath(__file__))



def main():
    dx = np.linspace(0, 30, 200)
    for vrx in (-1.0, -3.0, -6.0):
        plt.plot(dx, 2 * dx * vrx, label=f"vRx={{v}}".format(v=vrx))
    plt.xlabel("dx [m]"); plt.ylabel("dotB")
    plt.title("ComputeBarrierFirstDerivative (dy=0)")
    plt.legend(); plt.grid(True); plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, "ComputeBarrierFirstDerivative_response.png"), dpi=130)



if __name__ == "__main__":
    main()
