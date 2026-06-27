#!/Users/qingxu/.ai-env/bin/python3
"""
CbfControlRevision 模块响应可视化脚本（不依赖 C++ 端，仅复现数学公式）。
"""
import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt
import os

OUT_DIR = os.path.dirname(os.path.abspath(__file__))



def main():
    # 单障碍跟驰：扫 dx，看 a_safe vs a_orig
    dx = np.linspace(4, 25, 80)
    a_orig = 2.0
    r = 5.0
    alpha1 = alpha2 = 1.2
    vrx = -3.0
    a_safe = []
    for x in dx:
        B = x**2 - r**2
        dotB = 2 * x * vrx
        ddFree = 2 * vrx**2
        # bound = -(ddFree + (a1+a2)*dotB + a1*a2*B)；约束 -2 x * a >= bound
        bound = -(ddFree + (alpha1 + alpha2) * dotB + alpha1 * alpha2 * B)
        # 若 a_orig 满足 -2 x a_orig >= bound 则不修正
        lhs = -2 * x * a_orig
        if lhs >= bound:
            a_safe.append(a_orig)
        else:
            a_safe.append(-bound / (2 * x))
    a_safe = np.clip(np.array(a_safe), -7.0, 7.0)
    plt.plot(dx, np.full_like(dx, a_orig), "--", label="a_original")
    plt.plot(dx, a_safe, label="a_safe (analytical)")
    plt.xlabel("relative distance dx [m]"); plt.ylabel("acceleration [m/s^2]")
    plt.title("CbfControlRevision sweep (single obstacle)")
    plt.legend(); plt.grid(True); plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, "CbfControlRevision_response.png"), dpi=130)



if __name__ == "__main__":
    main()
