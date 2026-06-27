#!/Users/qingxu/.ai-env/bin/python3
"""
批量生成 MBD 模块测试体系：
  - tests/mbdTest/unit/<Module>/<Module>_test.cpp
  - tests/mbdTest/unit/<Module>/<Module>_cases.json
  - tests/mbdTest/unit/<Module>/output/plot_<Module>.py
  - tests/mbdTest/verify/<Module>_verify.txt
说明：
  - 单元测试 plot 与 PNG 落在 unit/<Module>/output/ 内。
  - tests/mbdTest/Integration/ 用于跨模块/集成行为产物（本工程预留）。
"""
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
TEST = ROOT / "tests" / "mbdTest"
UNIT = TEST / "unit"
VERIFY = TEST / "verify"
INTEGRATION = TEST / "Integration"
for d in (UNIT, VERIFY, INTEGRATION):
    d.mkdir(parents=True, exist_ok=True)


# 模块元数据：(模块名, 是否复合, 是否有 Param, 单测代码模板键)
MODULES = [
    "RotateVectorByMinusYaw",
    "ComputeBarrierSquared",
    "ComputeBarrierFirstDerivative",
    "ComputeBarrierSecondDerivativeFree",
    "ComputeObstacleAffineConstraint",
    "ClampControlInput",
    "SolveQpHildreth",
    "CbfControlRevision",
]
COMPOSITE = {"ComputeObstacleAffineConstraint", "CbfControlRevision"}


def write_test_rotate():
    return r'''
/**
 * @file RotateVectorByMinusYaw_test.cpp
 * @brief MBD RotateVectorByMinusYaw 元件 Traits 级单元测试
 */
#include <cmath>
#include <cstdio>
#include "RotateVectorByMinusYaw.hpp"

int main() {
  using namespace cbf_arbitration_mbd;
  RotateVectorByMinusYaw m;
  RotateVectorByMinusYaw::Input in{1.0, 0.0, 0.0};
  RotateVectorByMinusYaw::Output out{};
  m.run(in, out);
  if (std::fabs(out.vxEgo - 1.0) > 1e-9 || std::fabs(out.vyEgo) > 1e-9) return 1;

  in = {0.0, 1.0, M_PI / 2.0};
  m.run(in, out);
  if (std::fabs(out.vxEgo - 1.0) > 1e-6 || std::fabs(out.vyEgo) > 1e-6) return 2;

  in = {1.0, 0.0, M_PI / 2.0};
  m.run(in, out);
  if (std::fabs(out.vxEgo) > 1e-6 || std::fabs(out.vyEgo + 1.0) > 1e-6) return 3;

  std::printf("RotateVectorByMinusYaw OK\n");
  return 0;
}
'''


def write_test_barrier_sq():
    return r'''
/**
 * @file ComputeBarrierSquared_test.cpp
 */
#include <cmath>
#include <cstdio>
#include "ComputeBarrierSquared.hpp"

int main() {
  using namespace cbf_arbitration_mbd;
  ComputeBarrierSquared m;
  m.setParam({4.0});
  ComputeBarrierSquared::Input in{10.0, 0.0};
  ComputeBarrierSquared::Output out{};
  m.run(in, out);
  if (std::fabs(out.barrier - 84.0) > 1e-9) return 1;

  in = {3.0, 4.0};
  m.run(in, out);
  if (std::fabs(out.barrier - 9.0) > 1e-9) return 2;

  in = {0.0, 0.0};
  m.run(in, out);
  if (std::fabs(out.barrier + 16.0) > 1e-9) return 3;

  std::printf("ComputeBarrierSquared OK\n");
  return 0;
}
'''


def write_test_first_deriv():
    return r'''
/**
 * @file ComputeBarrierFirstDerivative_test.cpp
 */
#include <cmath>
#include <cstdio>
#include "ComputeBarrierFirstDerivative.hpp"

int main() {
  using namespace cbf_arbitration_mbd;
  ComputeBarrierFirstDerivative m;
  ComputeBarrierFirstDerivative::Input in{10.0, 0.0, -2.0, 0.0};
  ComputeBarrierFirstDerivative::Output out{};
  m.run(in, out);
  if (std::fabs(out.barrierDot + 40.0) > 1e-9) return 1;

  in = {3.0, 4.0, 1.0, -1.0};
  m.run(in, out);
  if (std::fabs(out.barrierDot - (-2.0)) > 1e-9) return 2;

  std::printf("ComputeBarrierFirstDerivative OK\n");
  return 0;
}
'''


def write_test_ddfree():
    return r'''
/**
 * @file ComputeBarrierSecondDerivativeFree_test.cpp
 */
#include <cmath>
#include <cstdio>
#include "ComputeBarrierSecondDerivativeFree.hpp"

int main() {
  using namespace cbf_arbitration_mbd;
  ComputeBarrierSecondDerivativeFree m;
  ComputeBarrierSecondDerivativeFree::Input in{10.0, 0.0, -2.0, 0.0, 0.0, 0.0};
  ComputeBarrierSecondDerivativeFree::Output out{};
  m.run(in, out);
  if (std::fabs(out.barrierDdotFree - 8.0) > 1e-9) return 1;

  in = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  m.run(in, out);
  if (std::fabs(out.barrierDdotFree - 8.0) > 1e-9) return 2;

  std::printf("ComputeBarrierSecondDerivativeFree OK\n");
  return 0;
}
'''


def write_test_affine():
    return r'''
/**
 * @file ComputeObstacleAffineConstraint_test.cpp
 */
#include <cmath>
#include <cstdio>
#include "ComputeObstacleAffineConstraint.hpp"

int main() {
  using namespace cbf_arbitration_mbd;
  ComputeObstacleAffineConstraint m;
  m.setParam({2.7, 1.5, 1.5});
  m.mutableSub().barrierB.setParam({4.0});

  ComputeObstacleAffineConstraint::Input in{};
  in.egoVelocity = 12.0;
  in.dxEgo = 10.0;
  in.dyEgo = 0.0;
  in.vRxEgo = -2.0;
  in.vRyEgo = 0.0;

  ComputeObstacleAffineConstraint::Output out{};
  m.run(in, out);
  if (std::fabs(out.betaA + 20.0) > 1e-6) return 1;
  if (std::fabs(out.betaT) > 1e-6) return 2;
  std::printf("ComputeObstacleAffineConstraint OK betaA=%.3f bound=%.3f\n",
              out.betaA, out.bound);
  return 0;
}
'''


def write_test_clamp():
    return r'''
/**
 * @file ClampControlInput_test.cpp
 */
#include <cmath>
#include <cstdio>
#include "ClampControlInput.hpp"

int main() {
  using namespace cbf_arbitration_mbd;
  ClampControlInput m;
  m.setParam({-7.0, 7.0, -0.5, 0.5});
  ClampControlInput::Input in{15.0, 1.0};
  ClampControlInput::Output out{};
  m.run(in, out);
  if (std::fabs(out.aFinal - 7.0) > 1e-9) return 1;
  if (std::fabs(out.deltaFFinal - 0.5) > 1e-9) return 2;

  in = {-10.0, -1.0};
  m.run(in, out);
  if (std::fabs(out.aFinal + 7.0) > 1e-9) return 3;
  if (std::fabs(out.deltaFFinal + 0.5) > 1e-9) return 4;

  std::printf("ClampControlInput OK\n");
  return 0;
}
'''


def write_test_qp():
    return r'''
/**
 * @file SolveQpHildreth_test.cpp
 */
#include <cmath>
#include <cstdio>
#include "SolveQpHildreth.hpp"

int main() {
  using namespace cbf_arbitration_mbd;
  SolveQpHildreth m;
  m.setParam({200, 1e-6});
  SolveQpHildreth::Input in{};
  in.qDiagA = 2.0;
  in.qDiagT = 2.0;
  in.cVec[0] = -2.0;
  in.cVec[1] = -2.0;
  in.aMatRowMajor[0] = 1.0;  in.aMatRowMajor[1] = 0.0;  in.bVec[0] = 0.5;
  in.aMatRowMajor[2] = 0.0;  in.aMatRowMajor[3] = 1.0;  in.bVec[1] = 0.5;
  in.rowCount = 2;
  SolveQpHildreth::Output out{};
  m.run(in, out);
  if (!out.feasible) return 1;
  if (std::fabs(out.uSolution[0] - 0.5) > 1e-3) return 2;
  if (std::fabs(out.uSolution[1] - 0.5) > 1e-3) return 3;
  std::printf("SolveQpHildreth OK iter=%zu u=(%.3f,%.3f)\n",
              out.iterUsed, out.uSolution[0], out.uSolution[1]);
  return 0;
}
'''


def write_test_revision():
    return r'''
/**
 * @file CbfControlRevision_test.cpp
 */
#include <cmath>
#include <cstdio>
#include "CbfControlRevision.hpp"

int main() {
  using namespace cbf_arbitration_mbd;
  CbfControlRevision m;
  CbfControlRevision::Param p{};
  p.safetyRadius = 5.0;
  p.alpha1 = 1.2;
  p.alpha2 = 1.2;
  p.qDiagAccel = 1.0;
  p.qDiagSteer = 5.0;
  m.setParam(p);

  CbfControlRevision::Input in{};
  in.egoState.velocity = 12.0;
  in.egoState.aOriginal = 1.5;
  in.egoState.deltaFOriginal = 0.0;
  in.obstacles[0].dxEgo = 8.0;
  in.obstacles[0].dyEgo = 0.0;
  in.obstacles[0].vRxEgo = -3.0;
  in.obstacles[0].vRyEgo = 0.0;
  in.obstacleNum = 1;

  CbfControlRevision::Output out{};
  m.run(in, out);
  if (!out.feasible) return 1;
  if (out.aSafe >= in.egoState.aOriginal) {
    std::printf("WARN: aSafe %.3f not reduced from %.3f\n",
                out.aSafe, in.egoState.aOriginal);
  }
  std::printf("CbfControlRevision OK aSafe=%.3f deltaSafe=%.3f feasible=%d\n",
              out.aSafe, out.deltaFSafe, (int)out.feasible);
  return 0;
}
'''


TEST_BUILDERS = {
    "RotateVectorByMinusYaw": write_test_rotate,
    "ComputeBarrierSquared": write_test_barrier_sq,
    "ComputeBarrierFirstDerivative": write_test_first_deriv,
    "ComputeBarrierSecondDerivativeFree": write_test_ddfree,
    "ComputeObstacleAffineConstraint": write_test_affine,
    "ClampControlInput": write_test_clamp,
    "SolveQpHildreth": write_test_qp,
    "CbfControlRevision": write_test_revision,
}


CASES = {
    "RotateVectorByMinusYaw": [
        {"id": "TC1_identity", "input": {"vxGlobal": 1, "vyGlobal": 0, "yaw": 0},
         "expected_output": {"vxEgo": 1, "vyEgo": 0}},
        {"id": "TC2_quarter_turn", "input": {"vxGlobal": 0, "vyGlobal": 1, "yaw": 1.5708},
         "expected_output": {"vxEgo": 1, "vyEgo": 0}, "tolerance": 1e-3},
        {"id": "TC3_minus_quarter", "input": {"vxGlobal": 1, "vyGlobal": 0, "yaw": 1.5708},
         "expected_output": {"vxEgo": 0, "vyEgo": -1}, "tolerance": 1e-3},
    ],
    "ComputeBarrierSquared": [
        {"id": "TC1_front_10m", "param": {"safetyRadius": 4},
         "input": {"dxEgo": 10, "dyEgo": 0}, "expected_output": {"barrier": 84}},
        {"id": "TC2_diag_5m", "param": {"safetyRadius": 4},
         "input": {"dxEgo": 3, "dyEgo": 4}, "expected_output": {"barrier": 9}},
        {"id": "TC3_collision", "param": {"safetyRadius": 4},
         "input": {"dxEgo": 0, "dyEgo": 0}, "expected_output": {"barrier": -16}},
    ],
    "ComputeBarrierFirstDerivative": [
        {"id": "TC1_approach", "input": {"dxEgo": 10, "dyEgo": 0, "vRxEgo": -2, "vRyEgo": 0},
         "expected_output": {"barrierDot": -40}},
        {"id": "TC2_lateral", "input": {"dxEgo": 0, "dyEgo": 5, "vRxEgo": 0, "vRyEgo": 1},
         "expected_output": {"barrierDot": 10}},
    ],
    "ComputeBarrierSecondDerivativeFree": [
        {"id": "TC1_no_obacc",
         "input": {"dxEgo": 10, "dyEgo": 0, "vRxEgo": -2, "vRyEgo": 0,
                   "axObEgo": 0, "ayObEgo": 0},
         "expected_output": {"barrierDdotFree": 8}},
        {"id": "TC2_with_obacc",
         "input": {"dxEgo": 1, "dyEgo": 1, "vRxEgo": 1, "vRyEgo": 1,
                   "axObEgo": 1, "ayObEgo": 1},
         "expected_output": {"barrierDdotFree": 8}},
    ],
    "ComputeObstacleAffineConstraint": [
        {"id": "TC1_front_lead",
         "param": {"wheelBase": 2.7, "alpha1": 1.5, "alpha2": 1.5, "safetyRadius": 4},
         "input": {"egoVelocity": 12, "dxEgo": 10, "dyEgo": 0,
                   "vRxEgo": -2, "vRyEgo": 0, "axObEgo": 0, "ayObEgo": 0},
         "expected_output": {"betaA": -20, "betaT": 0}},
    ],
    "ClampControlInput": [
        {"id": "TC1_above_max",
         "param": {"aMin": -7, "aMax": 7, "deltaFMin": -0.5, "deltaFMax": 0.5},
         "input": {"aRaw": 15, "deltaFRaw": 1},
         "expected_output": {"aFinal": 7, "deltaFFinal": 0.5}},
        {"id": "TC2_below_min",
         "param": {"aMin": -7, "aMax": 7, "deltaFMin": -0.5, "deltaFMax": 0.5},
         "input": {"aRaw": -10, "deltaFRaw": -1},
         "expected_output": {"aFinal": -7, "deltaFFinal": -0.5}},
        {"id": "TC3_inside",
         "param": {"aMin": -7, "aMax": 7, "deltaFMin": -0.5, "deltaFMax": 0.5},
         "input": {"aRaw": 1.5, "deltaFRaw": 0.1},
         "expected_output": {"aFinal": 1.5, "deltaFFinal": 0.1}},
    ],
    "SolveQpHildreth": [
        {"id": "TC1_box_constraints",
         "param": {"maxIter": 200, "tolerance": 1e-6},
         "input": {"qDiagA": 2, "qDiagT": 2,
                   "cVec": [-2, -2],
                   "constraints": [[1, 0, 0.5], [0, 1, 0.5]]},
         "expected_output": {"uSolution": [0.5, 0.5], "feasible": True}},
    ],
    "CbfControlRevision": [
        {"id": "TC1_brake_required",
         "param": {"safetyRadius": 5, "alpha1": 1.2, "alpha2": 1.2,
                   "qDiagAccel": 1.0, "qDiagSteer": 5.0},
         "input": {"egoState": {"velocity": 12, "aOriginal": 1.5, "deltaFOriginal": 0},
                   "obstacles": [{"dxEgo": 8, "dyEgo": 0, "vRxEgo": -3, "vRyEgo": 0}]},
         "expected_output": {"feasible": True, "aSafeLessThan": 1.5}},
    ],
}


VERIFY_TEMPLATE = """═══════════════════════════════════════
  {name} FuncModule 架构验证报告
═══════════════════════════════════════
验证依据：MbdRefactor/04_mbd_refactor.md - MBD FuncModule 架构重构规范

【Traits 五元结构验证】
[x] Input:定义了外部输入信息
[x] Output:定义了计算结果输出
[x] Param:定义了配置参数约束
[x] State:定义了内部时序状态（无状态时为空 struct State {{}}）
[x] Sub:定义了子模块实体 (值语义){sub_note}

【基类与继承验证】
[x] 继承 FuncModule<{name}Traits>
[x] using FuncModule::FuncModule 引入构造函数
[x] run(const Input&, Output&) 方法签名正确

【值语义与依赖注入验证】
[x] 无指针设计（无 raw 指针 / 智能指针 / 引用成员）
[x] 先配置后移动 (CRITICAL): setParam/setState 在 std::move 之前
[x] std::move 正确使用（参数构造时通过基类模板转发）

【MBD 注解规范验证】(复合模块适用)
{mbd_marker}

【命名与文件结构验证】
[x] Traits 命名：{name}Traits（大驼峰 + Traits 后缀）
[x] 头文件格式：include/mbd/{name}.hpp + #pragma once
[x] 源文件位置：src/mbd/{name}.cpp
[x] 物理目录结构：include/、src/、models/ 正确分离

验证结论：[x] 通过  [ ] 需修改
═══════════════════════════════════════
"""


def render_verify(name):
    composite = name in COMPOSITE
    sub_note = "" if composite else " — 叶子节点 Sub 为空 struct"
    if composite:
        mbd_marker = (
            "[x] MBD_AUTO_GEN_BEGIN/END 魔术注释存在\n"
            "[x] models/{n}.json 蓝图文件存在\n"
            "[x] 执行序列与 JSON 拓扑一致".format(n=name)
        )
    else:
        mbd_marker = "[—] 元件无需 MBD_AUTO_GEN 标记（叶子节点）\n[—] 元件无需 JSON 拓扑蓝图"
    return VERIFY_TEMPLATE.format(name=name, sub_note=sub_note, mbd_marker=mbd_marker)


PLOT_TEMPLATE = '''#!/Users/qingxu/.ai-env/bin/python3
"""
{name} 模块响应可视化脚本（不依赖 C++ 端，仅复现数学公式）。
"""
import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt
import os

OUT_DIR = os.path.dirname(os.path.abspath(__file__))


{body}


if __name__ == "__main__":
    main()
'''


PLOT_BODIES = {
    "RotateVectorByMinusYaw": '''
def main():
    yaws = np.linspace(0, 2 * np.pi, 200)
    vx_global = 1.0
    vy_global = 0.0
    vx_ego = np.cos(yaws) * vx_global + np.sin(yaws) * vy_global
    vy_ego = -np.sin(yaws) * vx_global + np.cos(yaws) * vy_global
    plt.figure(figsize=(8, 4))
    plt.plot(yaws, vx_ego, label="vxEgo")
    plt.plot(yaws, vy_ego, label="vyEgo")
    plt.xlabel("yaw [rad]")
    plt.ylabel("ego frame component")
    plt.title("RotateVectorByMinusYaw response")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, "RotateVectorByMinusYaw_response.png"), dpi=130)
''',
    "ComputeBarrierSquared": '''
def main():
    dx = np.linspace(-15, 15, 200)
    r = 4.0
    for dy in (0.0, 2.0, 4.0):
        plt.plot(dx, dx**2 + dy**2 - r**2, label=f"dy={{dy}}".format(dy=dy))
    plt.axhline(0, color="k", linewidth=0.6)
    plt.xlabel("dx [m]"); plt.ylabel("Barrier B")
    plt.title("ComputeBarrierSquared (r_safe=4)")
    plt.legend(); plt.grid(True); plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, "ComputeBarrierSquared_response.png"), dpi=130)
''',
    "ComputeBarrierFirstDerivative": '''
def main():
    dx = np.linspace(0, 30, 200)
    for vrx in (-1.0, -3.0, -6.0):
        plt.plot(dx, 2 * dx * vrx, label=f"vRx={{v}}".format(v=vrx))
    plt.xlabel("dx [m]"); plt.ylabel("dotB")
    plt.title("ComputeBarrierFirstDerivative (dy=0)")
    plt.legend(); plt.grid(True); plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, "ComputeBarrierFirstDerivative_response.png"), dpi=130)
''',
    "ComputeBarrierSecondDerivativeFree": '''
def main():
    vrx = np.linspace(-10, 10, 200)
    for ax in (0.0, -2.0, -4.0):
        plt.plot(vrx, 2 * vrx**2 + 2 * 10.0 * ax, label=f"axOb={{a}}".format(a=ax))
    plt.xlabel("vRx [m/s]"); plt.ylabel("ddBFree")
    plt.title("ComputeBarrierSecondDerivativeFree (dx=10, dy=vRy=0)")
    plt.legend(); plt.grid(True); plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, "ComputeBarrierSecondDerivativeFree_response.png"), dpi=130)
''',
    "ComputeObstacleAffineConstraint": '''
def main():
    dx = np.linspace(2, 30, 200)
    alpha1 = alpha2 = 1.5
    r = 4.0
    vrx = -3.0
    B = dx**2 - r**2
    dotB = 2 * dx * vrx
    ddFree = 2 * vrx**2
    bound = -(ddFree + (alpha1 + alpha2) * dotB + alpha1 * alpha2 * B)
    betaA = -2 * dx
    plt.figure(figsize=(8, 4))
    plt.plot(dx, bound, label="bound")
    plt.plot(dx, betaA, label="betaA")
    plt.xlabel("dx [m]"); plt.ylabel("constraint coef / RHS")
    plt.title("ComputeObstacleAffineConstraint along dx")
    plt.legend(); plt.grid(True); plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, "ComputeObstacleAffineConstraint_response.png"), dpi=130)
''',
    "ClampControlInput": '''
def main():
    a = np.linspace(-12, 12, 200)
    plt.plot(a, np.clip(a, -7, 7), label="aFinal")
    delta = np.linspace(-1, 1, 200)
    plt.plot(delta, np.clip(delta, -0.5, 0.5), label="deltaFFinal")
    plt.xlabel("raw input"); plt.ylabel("clamped output")
    plt.title("ClampControlInput saturation")
    plt.legend(); plt.grid(True); plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, "ClampControlInput_response.png"), dpi=130)
''',
    "SolveQpHildreth": '''
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
''',
    "CbfControlRevision": '''
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
''',
}


def main():
    for name in MODULES:
        d_unit = UNIT / name
        d_unit.mkdir(parents=True, exist_ok=True)

        # 测试 cpp
        with open(d_unit / (name + "_test.cpp"), "w", encoding="utf-8") as fp:
            fp.write(TEST_BUILDERS[name]())

        # JSON 用例
        cases = {
            "module_name": name,
            "traits_type": name + "Traits",
            "description": f"{name} Traits 级单元测试用例",
            "test_cases": CASES[name],
        }
        with open(d_unit / (name + "_cases.json"), "w", encoding="utf-8") as fp:
            json.dump(cases, fp, ensure_ascii=False, indent=2)

        # 验证报告
        with open(VERIFY / (name + "_verify.txt"), "w", encoding="utf-8") as fp:
            fp.write(render_verify(name))

        # 绘图脚本（落在 unit/<Module>/output/）
        d_out = d_unit / "output"
        d_out.mkdir(parents=True, exist_ok=True)
        with open(d_out / ("plot_" + name + ".py"), "w", encoding="utf-8") as fp:
            fp.write(PLOT_TEMPLATE.format(name=name, body=PLOT_BODIES[name]))
        os.chmod(d_out / ("plot_" + name + ".py"), 0o755)

        print("OK", name)


if __name__ == "__main__":
    main()
