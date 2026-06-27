#!/Users/qingxu/.ai-env/bin/python3
"""solveQpHildreth C++ 函数级行为可视化（独立于 C++ 端，纯公式复现）。"""
import os
import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt

OUT_DIR = os.path.dirname(os.path.abspath(__file__))



def main():
    q = 2.0; u0 = 5.0
    ubs = np.linspace(-2, 8, 200)
    u_opt = np.minimum(u0, ubs)
    plt.figure(figsize=(8, 4))
    plt.plot(ubs, u_opt, label="u*(ub)")
    plt.axhline(u0, color="k", linestyle="--", label="unconstrained u0=5")
    plt.xlabel("upper bound ub"); plt.ylabel("optimal u")
    plt.title("solveQpHildreth scalar example")
    plt.legend(); plt.grid(True); plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, "solveQpHildreth_response.png"), dpi=130)



if __name__ == "__main__":
    main()
