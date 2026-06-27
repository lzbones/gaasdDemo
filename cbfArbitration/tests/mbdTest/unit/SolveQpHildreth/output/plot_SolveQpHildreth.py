#!/Users/qingxu/.ai-env/bin/python3
"""
SolveQpHildreth 模块响应可视化脚本（不依赖 C++ 端，仅复现数学公式）。
"""
import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt
import os

OUT_DIR = os.path.dirname(os.path.abspath(__file__))



def main():
    # 求解 min 0.5 q*(u-u0)^2 s.t. u<=ub  扫 ub
    q = 2.0; u0 = 5.0
    ubs = np.linspace(-2, 8, 200)
    u_opt = np.minimum(u0, ubs)
    plt.plot(ubs, u_opt, label="u*(ub)")
    plt.axhline(u0, color="k", linestyle="--", label="unconstrained u0=5")
    plt.xlabel("upper bound ub"); plt.ylabel("optimal u")
    plt.title("SolveQpHildreth scalar example")
    plt.legend(); plt.grid(True); plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, "SolveQpHildreth_response.png"), dpi=130)



if __name__ == "__main__":
    main()
