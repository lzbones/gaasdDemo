/**
 * @file ComputeObstacleAffineConstraint.cpp
 * @brief 复合模块实现：装配单条 HOCBF 仿射约束
 */

#include "ComputeObstacleAffineConstraint.hpp"

namespace cbf_arbitration_mbd {

void ComputeObstacleAffineConstraint::run(const Input& input, Output& output) {
  // === MBD_AUTO_GEN_BEGIN [ComputeObstacleAffineConstraint] ===
  ComputeBarrierSquared::Input bIn{input.dxEgo, input.dyEgo};
  ComputeBarrierSquared::Output bOut{};
  sub_.barrierB.run(bIn, bOut);

  ComputeBarrierFirstDerivative::Input dotIn{input.dxEgo, input.dyEgo, input.vRxEgo, input.vRyEgo};
  ComputeBarrierFirstDerivative::Output dotOut{};
  sub_.barrierDot.run(dotIn, dotOut);

  ComputeBarrierSecondDerivativeFree::Input ddIn{input.dxEgo, input.dyEgo,
                                                  input.vRxEgo, input.vRyEgo,
                                                  input.axObEgo, input.ayObEgo};
  ComputeBarrierSecondDerivativeFree::Output ddOut{};
  sub_.barrierDdotFree.run(ddIn, ddOut);

  const double wheelBaseSafe = (param_.wheelBase > 1e-6) ? param_.wheelBase : 1e-6;
  const double velocitySquaredOverL = input.egoVelocity * input.egoVelocity / wheelBaseSafe;

  output.betaA = -2.0 * input.dxEgo;
  output.betaT = -2.0 * velocitySquaredOverL * input.dyEgo;
  const double residualConst = ddOut.barrierDdotFree
                             + (param_.alpha1 + param_.alpha2) * dotOut.barrierDot
                             + param_.alpha1 * param_.alpha2 * bOut.barrier;
  output.bound = -residualConst;
  // === MBD_AUTO_GEN_END [ComputeObstacleAffineConstraint] ===
}

}  // namespace cbf_arbitration_mbd
