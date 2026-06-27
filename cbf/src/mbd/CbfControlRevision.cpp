/**
 * @file CbfControlRevision.cpp
 * @brief MBD 顶层复合模块实现：装配多障碍 HOCBF 约束并求解 QP
 */

#include "CbfControlRevision.hpp"

#include <algorithm>
#include <cmath>

namespace cbf_mbd {

void CbfControlRevision::run(const Input& input, Output& output) {
  // === MBD_AUTO_GEN_BEGIN [CbfControlRevision] ===
  constexpr std::size_t BOX_CONSTRAINT_NUM = 4U;
  const std::size_t obstacleNumClipped = std::min(input.obstacleNum, MAX_MBD_OBSTACLES);

  SolveQpHildreth::Input qpIn{};
  qpIn.qDiagA = 2.0 * param_.qDiagAccel;
  qpIn.qDiagT = 2.0 * param_.qDiagSteer;

  const double tanDeltaFOriginal = std::tan(input.egoState.deltaFOriginal);
  qpIn.cVec[0] = -2.0 * param_.qDiagAccel * input.egoState.aOriginal;
  qpIn.cVec[1] = -2.0 * param_.qDiagSteer * tanDeltaFOriginal;

  // 配置子模块（先配置后移动语义已在 sub_ 静态实例上完成）
  sub_.constraintGen.setParam({param_.wheelBase, param_.alpha1, param_.alpha2});
  sub_.constraintGen.mutableSub().barrierB.setParam({param_.safetyRadius});

  std::size_t rowCount = 0;
  for (std::size_t i = 0; i < obstacleNumClipped; ++i) {
    RotateVectorByMinusYaw::Input rotIn{input.obstacles[i].axGlobal,
                                         input.obstacles[i].ayGlobal,
                                         input.egoState.phi};
    RotateVectorByMinusYaw::Output rotOut{};
    sub_.rotator.run(rotIn, rotOut);

    ComputeObstacleAffineConstraint::Input cIn{input.egoState.velocity,
                                                input.obstacles[i].dxEgo,
                                                input.obstacles[i].dyEgo,
                                                input.obstacles[i].vRxEgo,
                                                input.obstacles[i].vRyEgo,
                                                rotOut.vxEgo,
                                                rotOut.vyEgo};
    ComputeObstacleAffineConstraint::Output cOut{};
    sub_.constraintGen.run(cIn, cOut);

    qpIn.aMatRowMajor[2 * rowCount + 0] = -cOut.betaA;
    qpIn.aMatRowMajor[2 * rowCount + 1] = -cOut.betaT;
    qpIn.bVec[rowCount] = -cOut.bound;
    rowCount += 1U;
  }

  const double tanDeltaFMax = std::tan(param_.deltaFMax);
  const double tanDeltaFMin = std::tan(param_.deltaFMin);

  qpIn.aMatRowMajor[2 * rowCount + 0] = 1.0;
  qpIn.aMatRowMajor[2 * rowCount + 1] = 0.0;
  qpIn.bVec[rowCount] = param_.aMax;
  rowCount += 1U;

  qpIn.aMatRowMajor[2 * rowCount + 0] = -1.0;
  qpIn.aMatRowMajor[2 * rowCount + 1] = 0.0;
  qpIn.bVec[rowCount] = -param_.aMin;
  rowCount += 1U;

  qpIn.aMatRowMajor[2 * rowCount + 0] = 0.0;
  qpIn.aMatRowMajor[2 * rowCount + 1] = 1.0;
  qpIn.bVec[rowCount] = tanDeltaFMax;
  rowCount += 1U;

  qpIn.aMatRowMajor[2 * rowCount + 0] = 0.0;
  qpIn.aMatRowMajor[2 * rowCount + 1] = -1.0;
  qpIn.bVec[rowCount] = -tanDeltaFMin;
  rowCount += 1U;

  qpIn.rowCount = rowCount;
  sub_.qpSolver.setParam({param_.qpMaxIter, param_.qpTolerance});

  SolveQpHildreth::Output qpOut{};
  sub_.qpSolver.run(qpIn, qpOut);

  ClampControlInput::Input clampIn{qpOut.uSolution[0], std::atan(qpOut.uSolution[1])};
  sub_.clamper.setParam({param_.aMin, param_.aMax, param_.deltaFMin, param_.deltaFMax});
  ClampControlInput::Output clampOut{};
  sub_.clamper.run(clampIn, clampOut);

  output.aSafe = clampOut.aFinal;
  output.deltaFSafe = clampOut.deltaFFinal;
  output.feasible = qpOut.feasible;
  output.activeNum = obstacleNumClipped;
  output.iterUsed = qpOut.iterUsed;
  // === MBD_AUTO_GEN_END [CbfControlRevision] ===
  (void)BOX_CONSTRAINT_NUM;
}

}  // namespace cbf_mbd
