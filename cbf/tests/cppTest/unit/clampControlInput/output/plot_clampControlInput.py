#!/Users/qingxu/.ai-env/bin/python3
"""clampControlInput C++ 函数级行为可视化（独立于 C++ 端，纯公式复现）。"""
import os
import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt

OUT_DIR = os.path.dirname(os.path.abspath(__file__))



def main():
    plt.figure(figsize=(8, 4))
    a = np.linspace(-12, 12, 200)
    plt.plot(a, np.clip(a, -7, 7), label="aFinal")
    delta = np.linspace(-1, 1, 200)
    plt.plot(delta, np.clip(delta, -0.5, 0.5), label="deltaFFinal")
    plt.xlabel("raw input"); plt.ylabel("clamped output")
    plt.title("clampControlInput saturation")
    plt.legend(); plt.grid(True); plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, "clampControlInput_response.png"), dpi=130)



if __name__ == "__main__":
    main()
