"""为 cppTest/unit/<name>/output/ 批量生成 plot_<name>.py 并执行。
单元函数级行为可视化，纯 Python 复现公式（与 MBD plot 类似但目录指向 cpp 单测）。
"""
import os, subprocess, stat

ROOT = "/Users/qingxu/Documents/Software/Cpp/GAASD/gaasdDemo/cbf"

PLOT_TEMPLATE = '''#!/Users/qingxu/.ai-env/bin/python3
"""{name} C++ 函数级行为可视化（独立于 C++ 端，纯公式复现）。"""
import os
import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt

OUT_DIR = os.path.dirname(os.path.abspath(__file__))


{body}


if __name__ == "__main__":
    main()
'''

PLOT_BODIES = {
    "rotateVectorByMinusYaw": '''
def main():
    yaws = np.linspace(0, 2 * np.pi, 200)
    vx_global, vy_global = 1.0, 0.0
    vx_ego = np.cos(yaws) * vx_global + np.sin(yaws) * vy_global
    vy_ego = -np.sin(yaws) * vx_global + np.cos(yaws) * vy_global
    plt.figure(figsize=(8, 4))
    plt.plot(yaws, vx_ego, label="vxEgo")
    plt.plot(yaws, vy_ego, label="vyEgo")
    plt.xlabel("yaw [rad]"); plt.ylabel("ego frame component")
    plt.title("rotateVectorByMinusYaw response")
    plt.legend(); plt.grid(True); plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, "rotateVectorByMinusYaw_response.png"), dpi=130)
''',
    "computeBarrierSquared": '''
def main():
    dx = np.linspace(-15, 15, 200); r = 4.0
    plt.figure(figsize=(8, 4))
    for dy in (0.0, 2.0, 4.0):
        plt.plot(dx, dx**2 + dy**2 - r**2, label="dy=" + str(dy))
    plt.axhline(0, color="k", linewidth=0.6)
    plt.xlabel("dx [m]"); plt.ylabel("Barrier B")
    plt.title("computeBarrierSquared (r_safe=4)")
    plt.legend(); plt.grid(True); plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, "computeBarrierSquared_response.png"), dpi=130)
''',
    "computeBarrierFirstDerivative": '''
def main():
    dx = np.linspace(0, 30, 200)
    plt.figure(figsize=(8, 4))
    for vrx in (-1.0, -3.0, -6.0):
        plt.plot(dx, 2 * dx * vrx, label="vRx=" + str(vrx))
    plt.xlabel("dx [m]"); plt.ylabel("dotB")
    plt.title("computeBarrierFirstDerivative (dy=0)")
    plt.legend(); plt.grid(True); plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, "computeBarrierFirstDerivative_response.png"), dpi=130)
''',
    "computeBarrierSecondDerivativeFree": '''
def main():
    vrx = np.linspace(-10, 10, 200)
    plt.figure(figsize=(8, 4))
    for ax in (0.0, -2.0, -4.0):
        plt.plot(vrx, 2 * vrx**2 + 2 * 10.0 * ax, label="axOb=" + str(ax))
    plt.xlabel("vRx [m/s]"); plt.ylabel("ddBFree")
    plt.title("computeBarrierSecondDerivativeFree (dx=10, dy=vRy=0)")
    plt.legend(); plt.grid(True); plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, "computeBarrierSecondDerivativeFree_response.png"), dpi=130)
''',
    "computeObstacleAffineConstraint": '''
def main():
    dx = np.linspace(2, 30, 200)
    alpha1 = alpha2 = 1.5; r = 4.0; vrx = -3.0
    B = dx**2 - r**2
    dotB = 2 * dx * vrx
    ddFree = 2 * vrx**2
    bound = -(ddFree + (alpha1 + alpha2) * dotB + alpha1 * alpha2 * B)
    betaA = -2 * dx
    plt.figure(figsize=(8, 4))
    plt.plot(dx, bound, label="bound")
    plt.plot(dx, betaA, label="betaA")
    plt.xlabel("dx [m]"); plt.ylabel("constraint coef / RHS")
    plt.title("computeObstacleAffineConstraint along dx")
    plt.legend(); plt.grid(True); plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, "computeObstacleAffineConstraint_response.png"), dpi=130)
''',
    "clampControlInput": '''
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
''',
    "solveQpHildreth": '''
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
''',
    "cbfControlRevision": '''
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
''',
}


def main():
    for name, body in PLOT_BODIES.items():
        out_dir = os.path.join(ROOT, f"tests/cppTest/unit/{name}/output")
        os.makedirs(out_dir, exist_ok=True)
        plot_py = os.path.join(out_dir, f"plot_{name}.py")
        with open(plot_py, "w", encoding="utf-8") as fp:
            fp.write(PLOT_TEMPLATE.format(name=name, body=body))
        os.chmod(plot_py, 0o755)
        print("Wrote", plot_py)


if __name__ == "__main__":
    main()
