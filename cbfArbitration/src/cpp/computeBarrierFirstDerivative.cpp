/**
 * @file computeBarrierFirstDerivative.cpp
 * @brief Barrier 一阶时间导数实现
 */

#include "computeBarrierFirstDerivative.hpp"

namespace cbf_arbitration {

Real computeBarrierFirstDerivative(Real dxEgo,
                                   Real dyEgo,
                                   Real vRxEgo,
                                   Real vRyEgo) {
  const Real dotProduct = dxEgo * vRxEgo + dyEgo * vRyEgo;  // 位置-速度内积
  const Real barrierDot = 2.0 * dotProduct;                 // dB/dt
  return barrierDot;                                        // 单一出口
}

}  // namespace cbf_arbitration
