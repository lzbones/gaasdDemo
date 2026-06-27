#pragma once
/**
 * @file ComputeBarrierFirstDerivative.hpp
 * @brief Barrier 一阶导数 dotB = 2(dx*vRx + dy*vRy) 元件
 */

#include "FuncModule.hpp"

namespace cbf_arbitration_mbd {

struct ComputeBarrierFirstDerivativeTraits {
  struct Input {
    double dxEgo = 0.0;
    double dyEgo = 0.0;
    double vRxEgo = 0.0;
    double vRyEgo = 0.0;
  };
  struct Output {
    double barrierDot = 0.0;  ///< 一阶导数 dotB
  };
  struct Param {};
  struct State {};
  struct Sub {};
};

class ComputeBarrierFirstDerivative
    : public FuncModule<ComputeBarrierFirstDerivativeTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

}  // namespace cbf_arbitration_mbd
