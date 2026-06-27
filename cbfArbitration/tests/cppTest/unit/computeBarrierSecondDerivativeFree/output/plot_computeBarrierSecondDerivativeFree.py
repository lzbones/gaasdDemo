#!/Users/qingxu/.ai-env/bin/python3
"""computeBarrierSecondDerivativeFree C++ 函数级行为可视化（独立于 C++ 端，纯公式复现）。"""
import os
import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt

OUT_DIR = os.path.dirname(os.path.abspath(__file__))



def main():
    vrx = np.linspace(-10, 10, 200)
    plt.figure(figsize=(8, 4))
    for ax in (0.0, -2.0, -4.0):
        plt.plot(vrx, 2 * vrx**2 + 2 * 10.0 * ax, label="axOb=" + str(ax))
    plt.xlabel("vRx [m/s]"); plt.ylabel("ddBFree")
    plt.title("computeBarrierSecondDerivativeFree (dx=10, dy=vRy=0)")
    plt.legend(); plt.grid(True); plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, "computeBarrierSecondDerivativeFree_response.png"), dpi=130)



if __name__ == "__main__":
    main()
