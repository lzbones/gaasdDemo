/**
 * @file cbfControlRevision.cpp
 * @brief CBF 安全控制修正模块顶层入口实现
 *
 * 流程：
 *   1) 障碍物绝对加速度从全局 frame 旋转到 ego frame；
 *   2) 逐个障碍物生成 HOCBF 仿射约束 betaA*a + betaT*tanδ >= bound；
 *   3) 加上控制输入 box 约束（aMin/aMax, deltaFMin/deltaFMax）；
 *   4) 装配 QP，调用 Hildreth 求解器；
 *   5) 解 atan 还原 deltaF，再做一次饱和限幅得到最终安全控制。
 */

#include "cbfControlRevision.hpp"

#include <algorithm>
#include <cmath>

#include "clampControlInput.hpp"
#include "computeObstacleAffineConstraint.hpp"
#include "rotateVectorByMinusYaw.hpp"
#include "solveQpHildreth.hpp"

namespace cbf {

CbfOutput cbfControlRevision(const EgoState& egoState,
                             const ObstacleState* obstacles,
                             std::size_t obstacleNum,
                             const CbfParam& param) {
  constexpr std::size_t BOX_CONSTRAINT_NUM = 4U;                         // 4 条 box 约束
  constexpr std::size_t MAX_TOTAL_ROWS = MAX_OBSTACLE_NUM + BOX_CONSTRAINT_NUM;

  Real aMatRowMajor[MAX_TOTAL_ROWS * 2] = {0};                           // 约束矩阵（行优先）
  Real bVec[MAX_TOTAL_ROWS] = {0};                                       // 约束右端

  const std::size_t obstacleNumClipped =
      std::min(obstacleNum, MAX_OBSTACLE_NUM);                           // 防越界

  std::size_t rowCount = 0;                                              // 当前装配行数

  for (std::size_t i = 0; i < obstacleNumClipped; ++i) {                 // 装配障碍物约束
    Real axEgo = 0.0;
    Real ayEgo = 0.0;
    rotateVectorByMinusYaw(obstacles[i].axGlobal,
                           obstacles[i].ayGlobal,
                           egoState.phi,
                           axEgo,
                           ayEgo);                                       // 加速度入 ego frame
    const AffineConstraint cons =
        computeObstacleAffineConstraint(egoState.velocity,
                                        param.wheelBase,
                                        param.alpha1,
                                        param.alpha2,
                                        obstacles[i],
                                        axEgo,
                                        ayEgo,
                                        param.safetyRadius);            // 生成单条 CBF 约束
    aMatRowMajor[2 * rowCount + 0] = -cons.betaA;                        // 标准化为 Au<=b
    aMatRowMajor[2 * rowCount + 1] = -cons.betaT;
    bVec[rowCount] = -cons.bound;
    rowCount += 1U;
  }

  const Real tanDeltaFMax = std::tan(param.deltaFMax);                   // box 上界
  const Real tanDeltaFMin = std::tan(param.deltaFMin);                   // box 下界

  aMatRowMajor[2 * rowCount + 0] = 1.0;                                  // a <= aMax
  aMatRowMajor[2 * rowCount + 1] = 0.0;
  bVec[rowCount] = param.aMax;
  rowCount += 1U;

  aMatRowMajor[2 * rowCount + 0] = -1.0;                                 // -a <= -aMin
  aMatRowMajor[2 * rowCount + 1] = 0.0;
  bVec[rowCount] = -param.aMin;
  rowCount += 1U;

  aMatRowMajor[2 * rowCount + 0] = 0.0;                                  // tanδ <= tanδMax
  aMatRowMajor[2 * rowCount + 1] = 1.0;
  bVec[rowCount] = tanDeltaFMax;
  rowCount += 1U;

  aMatRowMajor[2 * rowCount + 0] = 0.0;                                  // -tanδ <= -tanδMin
  aMatRowMajor[2 * rowCount + 1] = -1.0;
  bVec[rowCount] = -tanDeltaFMin;
  rowCount += 1U;

  const Real qaScaled = 2.0 * param.qDiagAccel;                          // QP 标准形 Qd
  const Real qtScaled = 2.0 * param.qDiagSteer;
  const Real tanDeltaFOriginal = std::tan(egoState.deltaFOriginal);
  const Real cVec[2] = {-2.0 * param.qDiagAccel * egoState.aOriginal,    // 线性项 c
                        -2.0 * param.qDiagSteer * tanDeltaFOriginal};

  Real uSolution[2] = {egoState.aOriginal, tanDeltaFOriginal};           // 求解结果初值
  std::size_t iterUsed = 0;
  const bool feasible = solveQpHildreth(qaScaled,
                                        qtScaled,
                                        cVec,
                                        aMatRowMajor,
                                        bVec,
                                        rowCount,
                                        param.qpMaxIter,
                                        param.qpTolerance,
                                        uSolution,
                                        iterUsed);                       // QP 求解

  const Real deltaFRaw = std::atan(uSolution[1]);                        // tanδ → δ
  Real aFinal = 0.0;
  Real deltaFFinal = 0.0;
  clampControlInput(uSolution[0],
                    deltaFRaw,
                    param.aMin,
                    param.aMax,
                    param.deltaFMin,
                    param.deltaFMax,
                    aFinal,
                    deltaFFinal);                                         // 终值限幅

  CbfOutput output;
  output.aSafe = aFinal;
  output.deltaFSafe = deltaFFinal;
  output.feasible = feasible;
  output.activeNum = obstacleNumClipped;
  output.iterUsed = iterUsed;
  return output;                                                         // 单一出口
}

}  // namespace cbf
