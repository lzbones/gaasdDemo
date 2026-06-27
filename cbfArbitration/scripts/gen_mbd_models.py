#!/Users/qingxu/.ai-env/bin/python3
"""一次性生成 MBD 复合模块的 JSON 拓扑蓝图。"""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
MODELS = ROOT / "models"
MODELS.mkdir(parents=True, exist_ok=True)

CONSTRAINT = {
    "name": "ComputeObstacleAffineConstraint",
    "includes": [
        "FuncModule.hpp",
        "ComputeBarrierSquared.hpp",
        "ComputeBarrierFirstDerivative.hpp",
        "ComputeBarrierSecondDerivativeFree.hpp",
    ],
    "ports": {
        "Input": [
            "double egoVelocity = 0.0;",
            "double dxEgo = 0.0;",
            "double dyEgo = 0.0;",
            "double vRxEgo = 0.0;",
            "double vRyEgo = 0.0;",
            "double axObEgo = 0.0;",
            "double ayObEgo = 0.0;",
        ],
        "Output": [
            "double betaA = 0.0;",
            "double betaT = 0.0;",
            "double bound = 0.0;",
        ],
        "Param": [
            "double wheelBase = 2.7;",
            "double alpha1 = 1.5;",
            "double alpha2 = 1.5;",
        ],
        "State": [],
        "Sub": [
            "ComputeBarrierSquared barrierB;",
            "ComputeBarrierFirstDerivative barrierDot;",
            "ComputeBarrierSecondDerivativeFree barrierDdotFree;",
        ],
    },
    "execution_sequence": [
        {"node": "barrierB", "type": "ComputeBarrierSquared",
         "inputs": {"dxEgo": "input.dxEgo", "dyEgo": "input.dyEgo"}},
        {"node": "barrierDot", "type": "ComputeBarrierFirstDerivative",
         "inputs": {"dxEgo": "input.dxEgo", "dyEgo": "input.dyEgo",
                    "vRxEgo": "input.vRxEgo", "vRyEgo": "input.vRyEgo"}},
        {"node": "barrierDdotFree", "type": "ComputeBarrierSecondDerivativeFree",
         "inputs": {"dxEgo": "input.dxEgo", "dyEgo": "input.dyEgo",
                    "vRxEgo": "input.vRxEgo", "vRyEgo": "input.vRyEgo",
                    "axObEgo": "input.axObEgo", "ayObEgo": "input.ayObEgo"}},
    ],
    "outputs": {
        "betaA": "-2 * input.dxEgo",
        "betaT": "-2 * (input.egoVelocity^2 / param.wheelBase) * input.dyEgo",
        "bound": "-(ddOut.barrierDdotFree + (alpha1+alpha2)*dotOut.barrierDot + alpha1*alpha2*bOut.barrier)",
    },
}

REVISION = {
    "name": "CbfControlRevision",
    "includes": [
        "FuncModule.hpp",
        "RotateVectorByMinusYaw.hpp",
        "ComputeObstacleAffineConstraint.hpp",
        "SolveQpHildreth.hpp",
        "ClampControlInput.hpp",
    ],
    "ports": {
        "Input": [
            "EgoSnapshot egoState;",
            "ObstacleSnapshot obstacles[MAX_MBD_OBSTACLES];",
            "size_t obstacleNum = 0;",
        ],
        "Output": [
            "double aSafe = 0.0;",
            "double deltaFSafe = 0.0;",
            "bool feasible = true;",
            "size_t activeNum = 0;",
            "size_t iterUsed = 0;",
        ],
        "Param": [
            "double safetyRadius = 5.0;",
            "double alpha1 = 1.2;",
            "double alpha2 = 1.2;",
            "double wheelBase = 2.7;",
            "double aMin = -7.0;",
            "double aMax = 7.0;",
            "double deltaFMin = -0.5;",
            "double deltaFMax = 0.5;",
            "double qDiagAccel = 1.0;",
            "double qDiagSteer = 5.0;",
            "size_t qpMaxIter = 200;",
            "double qpTolerance = 1e-6;",
        ],
        "State": [],
        "Sub": [
            "RotateVectorByMinusYaw rotator;",
            "ComputeObstacleAffineConstraint constraintGen;",
            "SolveQpHildreth qpSolver;",
            "ClampControlInput clamper;",
        ],
    },
    "execution_sequence": [
        {"node": "rotator", "type": "RotateVectorByMinusYaw",
         "inputs": {"vxGlobal": "obstacles[i].axGlobal",
                    "vyGlobal": "obstacles[i].ayGlobal",
                    "yaw": "egoState.phi"}},
        {"node": "constraintGen", "type": "ComputeObstacleAffineConstraint",
         "inputs": {"egoVelocity": "egoState.velocity",
                    "dxEgo": "obstacles[i].dxEgo",
                    "dyEgo": "obstacles[i].dyEgo",
                    "vRxEgo": "obstacles[i].vRxEgo",
                    "vRyEgo": "obstacles[i].vRyEgo",
                    "axObEgo": "rotatorOut.vxEgo",
                    "ayObEgo": "rotatorOut.vyEgo"}},
        {"node": "qpSolver", "type": "SolveQpHildreth",
         "inputs": {"qDiagA": "param.qDiagAccel",
                    "qDiagT": "param.qDiagSteer",
                    "cVec": "[-2*qDiagAccel*aOriginal, -2*qDiagSteer*tan(deltaFOriginal)]",
                    "aMatRowMajor": "stacked_constraints",
                    "bVec": "stacked_bounds",
                    "rowCount": "obstacleNum + 4"}},
        {"node": "clamper", "type": "ClampControlInput",
         "inputs": {"aRaw": "qpSolverOut.uSolution[0]",
                    "deltaFRaw": "atan(qpSolverOut.uSolution[1])"}},
    ],
    "outputs": {
        "aSafe": "clamperOut.aFinal",
        "deltaFSafe": "clamperOut.deltaFFinal",
        "feasible": "qpSolverOut.feasible",
        "activeNum": "obstacleNum",
        "iterUsed": "qpSolverOut.iterUsed",
    },
}

for blueprint in (CONSTRAINT, REVISION):
    out = MODELS / (blueprint["name"] + ".json")
    with open(out, "w", encoding="utf-8") as fp:
        json.dump(blueprint, fp, ensure_ascii=False, indent=2)
    print("WROTE", out)
