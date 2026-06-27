/**
 * @file computeObstacleAffineConstraint.cpp
 * @brief HOCBF + D-CBF 仿射约束生成实现
 */

#include "computeObstacleAffineConstraint.hpp"

#include "computeBarrierFirstDerivative.hpp"
#include "computeBarrierSecondDerivativeFree.hpp"
#include "computeBarrierSquared.hpp"

namespace cbf_arbitration {

AffineConstraint computeObstacleAffineConstraint(Real egoVelocity,
                                                 Real wheelBase,
                                                 Real alpha1,
                                                 Real alpha2,
                                                 const ObstacleState& obstacle,
                                                 Real axObEgo,
                                                 Real ayObEgo,
                                                 Real safetyRadius) {
  const Real barrierValue = computeBarrierSquared(obstacle.dxEgo,
                                                  obstacle.dyEgo,
                                                  safetyRadius);          // B
  const Real barrierDot = computeBarrierFirstDerivative(obstacle.dxEgo,
                                                        obstacle.dyEgo,
                                                        obstacle.vRxEgo,
                                                        obstacle.vRyEgo); // dotB
  const Real barrierDdotFree =
      computeBarrierSecondDerivativeFree(obstacle.dxEgo,
                                         obstacle.dyEgo,
                                         obstacle.vRxEgo,
                                         obstacle.vRyEgo,
                                         axObEgo,
                                         ayObEgo);                        // ddB 自由项

  const Real wheelBaseSafe = (wheelBase > 1e-6) ? wheelBase : 1e-6;       // 防 L=0
  const Real velocitySquaredOverL = egoVelocity * egoVelocity / wheelBaseSafe;

  const Real coefficientA = -2.0 * obstacle.dxEgo;                        // 加速度通道系数 betaA
  const Real coefficientT = -2.0 * velocitySquaredOverL * obstacle.dyEgo; // 转向通道系数 betaT

  const Real classKaSum = (alpha1 + alpha2) * barrierDot;                 // (α1+α2)·dotB
  const Real classKaProd = alpha1 * alpha2 * barrierValue;                // α1·α2·B
  const Real residualConst = barrierDdotFree + classKaSum + classKaProd;  // u 无关项之和

  AffineConstraint result;
  result.betaA = coefficientA;                                            // 输出加速度系数
  result.betaT = coefficientT;                                            // 输出转向系数
  result.bound = -residualConst;                                          // 不等式右端
  return result;                                                          // 单一出口
}

}  // namespace cbf_arbitration
