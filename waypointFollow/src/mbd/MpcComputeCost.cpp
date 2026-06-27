#include "MpcComputeCost.hpp"
#include <cmath>

namespace waypoint_follow_mbd {

void MpcComputeCost::run(const Input& input, Output& output) {
  // === MBD_AUTO_GEN_BEGIN [MpcComputeCost] ===
  const auto& p = param_.mpcParam;
  double cost = 0.0;
  double cx = input.initialState.x, cy = input.initialState.y;
  double cyaw = input.initialState.yaw, cv = input.initialState.v;

  for (int k = 0; k < p.predictionHorizon; ++k) {
    const int ci = (k < p.controlHorizon) ? k : (p.controlHorizon - 1);
    const auto& ctrl = input.controlSequence[ci];

    // 调用自行车模型子模块
    PredictBicycleStateTraits::Input pbsIn{cx, cy, cyaw, cv, ctrl.steeringAngle, ctrl.acceleration};
    PredictBicycleStateTraits::Output pbsOut;
    sub_.predictState.run(pbsIn, pbsOut);

    // 调用多项式求值子模块 → 参考Y
    const double xNorm = (pbsOut.xNext - input.trajectory.xOffset) / input.trajectory.xScale;
    EvaluatePolynomialTraits::Input epIn{input.trajectory.poly, xNorm};
    EvaluatePolynomialTraits::Output epOut;
    sub_.evalPoly.run(epIn, epOut);

    // 调用斜率子模块 → 参考航向
    EvaluatePolynomialSlopeTraits::Input esIn{input.trajectory.poly, xNorm};
    EvaluatePolynomialSlopeTraits::Output esOut;
    sub_.evalSlope.run(esIn, esOut);
    const double refHeading = std::atan2(esOut.result, input.trajectory.xScale);

    // 调用角度归一化子模块
    NormalizeAngleTraits::Input naIn{pbsOut.yawNext - refHeading};
    NormalizeAngleTraits::Output naOut;
    sub_.normAngle.run(naIn, naOut);

    const double eLat = pbsOut.yNext - epOut.result;
    const double eHead = naOut.result;
    const double eVel = pbsOut.vNext - input.trajectory.refSpeed;

    cost += p.wLateral * eLat * eLat + p.wHeading * eHead * eHead + p.wVelocity * eVel * eVel;

    if (k < p.controlHorizon) {
      cost += p.wSteering * ctrl.steeringAngle * ctrl.steeringAngle;
      cost += p.wAccel * ctrl.acceleration * ctrl.acceleration;
      const double pDelta = (k == 0) ? input.prevAppliedControl.steeringAngle : input.controlSequence[k - 1].steeringAngle;
      const double pAccel = (k == 0) ? input.prevAppliedControl.acceleration : input.controlSequence[k - 1].acceleration;
      cost += p.wSteeringRate * (ctrl.steeringAngle - pDelta) * (ctrl.steeringAngle - pDelta);
      cost += p.wAccelRate * (ctrl.acceleration - pAccel) * (ctrl.acceleration - pAccel);
    }

    cx = pbsOut.xNext; cy = pbsOut.yNext; cyaw = pbsOut.yawNext; cv = pbsOut.vNext;
  }

  output.totalCost = cost;
  // === MBD_AUTO_GEN_END [MpcComputeCost] ===
}

} // namespace waypoint_follow_mbd
