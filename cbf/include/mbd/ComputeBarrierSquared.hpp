#pragma once
/**
 * @file ComputeBarrierSquared.hpp
 * @brief 平方型 Barrier 函数 B = dx^2 + dy^2 - r_safe^2 元件
 */

#include "FuncModule.hpp"

namespace cbf_mbd {

struct ComputeBarrierSquaredTraits {
  struct Input {
    double dxEgo = 0.0;  ///< ego 系纵向相对位置
    double dyEgo = 0.0;  ///< ego 系横向相对位置
  };
  struct Output {
    double barrier = 0.0;  ///< Barrier 值 B
  };
  struct Param {
    double safetyRadius = 4.0;  ///< 安全圆半径 r_safe
  };
  struct State {};
  struct Sub {};
};

class ComputeBarrierSquared : public FuncModule<ComputeBarrierSquaredTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

}  // namespace cbf_mbd
