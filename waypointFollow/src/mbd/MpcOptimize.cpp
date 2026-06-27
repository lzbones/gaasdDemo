#include "MpcOptimize.hpp"
#include <cmath>

namespace waypoint_follow_mbd {

void MpcOptimize::run(const Input& input, Output& output) {
  // === MBD_AUTO_GEN_BEGIN [MpcOptimize] ===
  const auto& p = param_.mpcParam;
  const int nc = p.controlHorizon;
  const double eps = p.finiteDiffEps;

  waypoint_follow::ControlInput controls[waypoint_follow::MAX_CONTROL_HORIZON] = {};
  for (int i = 0; i < nc; ++i) controls[i] = input.warmStartSequence[i];

  // 构造代价函数输入模板
  MpcComputeCostTraits::Input costIn;
  costIn.initialState = input.initialState;
  costIn.trajectory = input.trajectory;
  costIn.prevAppliedControl = input.prevAppliedControl;

  auto computeCost = [&](const waypoint_follow::ControlInput* seq) -> double {
    for (int i = 0; i < nc; ++i) costIn.controlSequence[i] = seq[i];
    MpcComputeCostTraits::Output costOut;
    sub_.computeCost.run(costIn, costOut);
    return costOut.totalCost;
  };

  double currentCost = computeCost(controls);
  double lr = p.learningRate;

  for (int iter = 0; iter < p.maxOptIterations; ++iter) {
    double gDelta[waypoint_follow::MAX_CONTROL_HORIZON] = {};
    double gAccel[waypoint_follow::MAX_CONTROL_HORIZON] = {};

    // 中心差分梯度
    for (int i = 0; i < nc; ++i) {
      waypoint_follow::ControlInput pp[waypoint_follow::MAX_CONTROL_HORIZON] = {};
      waypoint_follow::ControlInput pm[waypoint_follow::MAX_CONTROL_HORIZON] = {};
      for (int j = 0; j < nc; ++j) { pp[j] = controls[j]; pm[j] = controls[j]; }
      pp[i].steeringAngle += eps; pm[i].steeringAngle -= eps;
      gDelta[i] = (computeCost(pp) - computeCost(pm)) / (2.0 * eps);

      for (int j = 0; j < nc; ++j) { pp[j] = controls[j]; pm[j] = controls[j]; }
      pp[i].acceleration += eps; pm[i].acceleration -= eps;
      gAccel[i] = (computeCost(pp) - computeCost(pm)) / (2.0 * eps);
    }

    // 梯度范数裁剪: 超过单位长度时缩放
    double gnormSq = 0.0;
    for (int i = 0; i < nc; ++i) { gnormSq += gDelta[i]*gDelta[i] + gAccel[i]*gAccel[i]; }
    const double gn = std::sqrt(gnormSq);
    const double scale = (gnormSq < 1e-16) ? 0.0 : (gn > 1.0) ? (1.0 / gn) : 1.0;

    // 候选更新
    waypoint_follow::ControlInput cand[waypoint_follow::MAX_CONTROL_HORIZON] = {};
    ClampValueTraits::Input clIn;
    ClampValueTraits::Output clOut;

    for (int i = 0; i < nc; ++i) {
      double ds = controls[i].steeringAngle - lr * gDelta[i] * scale;
      double da = controls[i].acceleration - lr * gAccel[i] * scale;
      clIn = {ds, -p.maxSteeringAngle, p.maxSteeringAngle}; sub_.clampVal.run(clIn, clOut); ds = clOut.result;
      clIn = {da, p.minAcceleration, p.maxAcceleration}; sub_.clampVal.run(clIn, clOut); da = clOut.result;
      cand[i].steeringAngle = ds;
      cand[i].acceleration = da;
    }

    // 转向变化率约束
    clIn = {cand[0].steeringAngle, input.prevAppliedControl.steeringAngle - p.maxSteeringRate,
            input.prevAppliedControl.steeringAngle + p.maxSteeringRate};
    sub_.clampVal.run(clIn, clOut); cand[0].steeringAngle = clOut.result;
    for (int i = 1; i < nc; ++i) {
      clIn = {cand[i].steeringAngle, cand[i-1].steeringAngle - p.maxSteeringRate,
              cand[i-1].steeringAngle + p.maxSteeringRate};
      sub_.clampVal.run(clIn, clOut); cand[i].steeringAngle = clOut.result;
    }

    const double candCost = computeCost(cand);
    if (candCost < currentCost) {
      for (int i = 0; i < nc; ++i) controls[i] = cand[i];
      currentCost = candCost;
    } else {
      lr *= 0.5;
    }
  }

  for (int i = 0; i < nc; ++i) output.optimalSequence[i] = controls[i];
  output.finalCost = currentCost;
  // === MBD_AUTO_GEN_END [MpcOptimize] ===
}

} // namespace waypoint_follow_mbd
