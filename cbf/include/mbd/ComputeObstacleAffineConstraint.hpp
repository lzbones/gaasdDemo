#pragma once
/**
 * @file ComputeObstacleAffineConstraint.hpp
 * @brief 单障碍物 HOCBF 仿射约束生成（复合模块）
 *
 * Sub 中包含三个叶子元件：
 *   - ComputeBarrierSquared
 *   - ComputeBarrierFirstDerivative
 *   - ComputeBarrierSecondDerivativeFree
 *
 * 输出：betaA*a + betaT*tan(δ_f) >= bound
 */

#include "ComputeBarrierFirstDerivative.hpp"
#include "ComputeBarrierSecondDerivativeFree.hpp"
#include "ComputeBarrierSquared.hpp"
#include "FuncModule.hpp"

namespace cbf_mbd {

struct ComputeObstacleAffineConstraintTraits {
  struct Input {
    double egoVelocity = 0.0;
    double dxEgo = 0.0;
    double dyEgo = 0.0;
    double vRxEgo = 0.0;
    double vRyEgo = 0.0;
    double axObEgo = 0.0;
    double ayObEgo = 0.0;
  };
  struct Output {
    double betaA = 0.0;
    double betaT = 0.0;
    double bound = 0.0;
  };
  struct Param {
    double wheelBase = 2.7;
    double alpha1 = 1.5;
    double alpha2 = 1.5;
  };
  struct State {};
  struct Sub {
    ComputeBarrierSquared barrierB{};
    ComputeBarrierFirstDerivative barrierDot{};
    ComputeBarrierSecondDerivativeFree barrierDdotFree{};
  };
};

class ComputeObstacleAffineConstraint
    : public FuncModule<ComputeObstacleAffineConstraintTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

}  // namespace cbf_mbd
