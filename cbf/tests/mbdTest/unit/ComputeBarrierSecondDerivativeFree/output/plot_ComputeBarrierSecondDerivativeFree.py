#!/Users/qingxu/.ai-env/bin/python3
"""
ComputeBarrierSecondDerivativeFree 模块响应可视化脚本（不依赖 C++ 端，仅复现数学公式）。
"""
import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt
import os

OUT_DIR = os.path.dirname(os.path.abspath(__file__))



def main():
    vrx = np.linspace(-10, 10, 200)
    for ax in (0.0, -2.0, -4.0):
        plt.plot(vrx, 2 * vrx**2 + 2 * 10.0 * ax, label=f"axOb={{a}}".format(a=ax))
    plt.xlabel("vRx [m/s]"); plt.ylabel("ddBFree")
    plt.title("ComputeBarrierSecondDerivativeFree (dx=10, dy=vRy=0)")
    plt.legend(); plt.grid(True); plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, "ComputeBarrierSecondDerivativeFree_response.png"), dpi=130)



if __name__ == "__main__":
    main()
