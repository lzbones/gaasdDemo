/**
 * @file computeBarrierSecondDerivativeFree.cpp
 * @brief Barrier 二阶导数中与 u 无关的自由项实现
 */

#include "computeBarrierSecondDerivativeFree.hpp"

namespace cbf {

Real computeBarrierSecondDerivativeFree(Real dxEgo,
                                        Real dyEgo,
                                        Real vRxEgo,
                                        Real vRyEgo,
                                        Real axObEgo,
                                        Real ayObEgo) {
  const Real velocitySquaredSum = vRxEgo * vRxEgo + vRyEgo * vRyEgo;        // 相对速度模平方
  const Real positionAccelInner = dxEgo * axObEgo + dyEgo * ayObEgo;        // 位置-障碍加速度内积
  const Real freeTerm = 2.0 * velocitySquaredSum + 2.0 * positionAccelInner; // 自由项总和
  return freeTerm;                                                          // 单一出口
}

}  // namespace cbf
