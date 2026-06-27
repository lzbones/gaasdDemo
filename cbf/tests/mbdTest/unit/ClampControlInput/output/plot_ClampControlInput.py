#!/Users/qingxu/.ai-env/bin/python3
"""
ClampControlInput 模块响应可视化脚本（不依赖 C++ 端，仅复现数学公式）。
"""
import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt
import os

OUT_DIR = os.path.dirname(os.path.abspath(__file__))



def main():
    a = np.linspace(-12, 12, 200)
    plt.plot(a, np.clip(a, -7, 7), label="aFinal")
    delta = np.linspace(-1, 1, 200)
    plt.plot(delta, np.clip(delta, -0.5, 0.5), label="deltaFFinal")
    plt.xlabel("raw input"); plt.ylabel("clamped output")
    plt.title("ClampControlInput saturation")
    plt.legend(); plt.grid(True); plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, "ClampControlInput_response.png"), dpi=130)



if __name__ == "__main__":
    main()
