/**
 * @file ComputeBarrierFirstDerivative.cpp
 * @brief Barrier 一阶导数计算
 */

#include "ComputeBarrierFirstDerivative.hpp"

namespace cbf_mbd {

void ComputeBarrierFirstDerivative::run(const Input& input, Output& output) {
  const double innerProduct = input.dxEgo * input.vRxEgo + input.dyEgo * input.vRyEgo;
  output.barrierDot = 2.0 * innerProduct;
}

}  // namespace cbf_mbd
