#!/Users/qingxu/.ai-env/bin/python3
"""computeBarrierFirstDerivative C++ 函数级行为可视化（独立于 C++ 端，纯公式复现）。"""
import os
import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt

OUT_DIR = os.path.dirname(os.path.abspath(__file__))



def main():
    dx = np.linspace(0, 30, 200)
    plt.figure(figsize=(8, 4))
    for vrx in (-1.0, -3.0, -6.0):
        plt.plot(dx, 2 * dx * vrx, label="vRx=" + str(vrx))
    plt.xlabel("dx [m]"); plt.ylabel("dotB")
    plt.title("computeBarrierFirstDerivative (dy=0)")
    plt.legend(); plt.grid(True); plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, "computeBarrierFirstDerivative_response.png"), dpi=130)



if __name__ == "__main__":
    main()
