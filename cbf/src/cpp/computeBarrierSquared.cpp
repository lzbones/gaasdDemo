/**
 * @file computeBarrierSquared.cpp
 * @brief 平方型距离 Barrier 函数实现
 */

#include "computeBarrierSquared.hpp"

namespace cbf {

Real computeBarrierSquared(Real dxEgo, Real dyEgo, Real safetyRadius) {
  const Real distanceSquared = dxEgo * dxEgo + dyEgo * dyEgo;       // 自车-障碍物距离平方
  const Real radiusSquared = safetyRadius * safetyRadius;           // 安全半径平方
  const Real barrierValue = distanceSquared - radiusSquared;        // 平方型 B 值
  return barrierValue;                                              // 单一出口
}

}  // namespace cbf
