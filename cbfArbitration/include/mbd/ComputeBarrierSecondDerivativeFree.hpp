#pragma once
/**
 * @file ComputeBarrierSecondDerivativeFree.hpp
 * @brief Barrier 二阶导数中与控制 u 无关的「自由项」元件
 *
 * ddBFree = 2*(vRx^2 + vRy^2) + 2*(dx*axOb + dy*ayOb)
 */

#include "FuncModule.hpp"

namespace cbf_arbitration_mbd {

struct ComputeBarrierSecondDerivativeFreeTraits {
  struct Input {
    double dxEgo = 0.0;
    double dyEgo = 0.0;
    double vRxEgo = 0.0;
    double vRyEgo = 0.0;
    double axObEgo = 0.0;
    double ayObEgo = 0.0;
  };
  struct Output {
    double barrierDdotFree = 0.0;
  };
  struct Param {};
  struct State {};
  struct Sub {};
};

class ComputeBarrierSecondDerivativeFree
    : public FuncModule<ComputeBarrierSecondDerivativeFreeTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

}  // namespace cbf_arbitration_mbd
