/**
 * @file ComputeBarrierSquared.cpp
 * @brief 平方型 Barrier 函数计算
 */

#include "ComputeBarrierSquared.hpp"

namespace cbf_arbitration_mbd {

void ComputeBarrierSquared::run(const Input& input, Output& output) {
  const double distSquared = input.dxEgo * input.dxEgo + input.dyEgo * input.dyEgo;
  const double radiusSquared = param_.safetyRadius * param_.safetyRadius;
  output.barrier = distSquared - radiusSquared;
}

}  // namespace cbf_arbitration_mbd
