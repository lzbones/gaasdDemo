#!/Users/qingxu/.ai-env/bin/python3
"""cbfControlRevision C++ 函数级行为可视化（独立于 C++ 端，纯公式复现）。"""
import os
import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt

OUT_DIR = os.path.dirname(os.path.abspath(__file__))



def main():
    dx = np.linspace(4, 25, 80)
    a_orig = 2.0; r = 5.0; alpha1 = alpha2 = 1.2; vrx = -3.0
    a_safe = []
    for x in dx:
        B = x**2 - r**2
        dotB = 2 * x * vrx
        ddFree = 2 * vrx**2
        bound = -(ddFree + (alpha1 + alpha2) * dotB + alpha1 * alpha2 * B)
        lhs = -2 * x * a_orig
        if lhs >= bound:
            a_safe.append(a_orig)
        else:
            a_safe.append(-bound / (2 * x))
    a_safe = np.clip(np.array(a_safe), -7.0, 7.0)
    plt.figure(figsize=(8, 4))
    plt.plot(dx, np.full_like(dx, a_orig), "--", label="a_original")
    plt.plot(dx, a_safe, label="a_safe (analytical)")
    plt.xlabel("relative distance dx [m]"); plt.ylabel("acceleration [m/s^2]")
    plt.title("cbfControlRevision sweep (single obstacle)")
    plt.legend(); plt.grid(True); plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, "cbfControlRevision_response.png"), dpi=130)



if __name__ == "__main__":
    main()
