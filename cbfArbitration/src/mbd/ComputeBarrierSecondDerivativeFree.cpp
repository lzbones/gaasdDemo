/**
 * @file ComputeBarrierSecondDerivativeFree.cpp
 * @brief Barrier 二阶导数自由项计算
 */

#include "ComputeBarrierSecondDerivativeFree.hpp"

namespace cbf_arbitration_mbd {

void ComputeBarrierSecondDerivativeFree::run(const Input& input, Output& output) {
  const double velSq = input.vRxEgo * input.vRxEgo + input.vRyEgo * input.vRyEgo;
  const double obAccTerm = input.dxEgo * input.axObEgo + input.dyEgo * input.ayObEgo;
  output.barrierDdotFree = 2.0 * velSq + 2.0 * obAccTerm;
}

}  // namespace cbf_arbitration_mbd
