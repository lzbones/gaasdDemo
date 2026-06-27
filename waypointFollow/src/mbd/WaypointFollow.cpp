#include "WaypointFollow.hpp"

namespace waypoint_follow_mbd {

void WaypointFollow::run(const Input& input, Output& output) {
  // === MBD_AUTO_GEN_BEGIN [WaypointFollow] ===
  const double currentFront = input.currentSteeringWheelAngle / param_.steeringRatio;

  // 低速保护
  if (input.egoSpeed < waypoint_follow::MIN_SPEED_THRESHOLD) {
    output.steeringWheelAngle = input.currentSteeringWheelAngle;
    output.frontWheelAngle = currentFront;
    output.acceleration = 0.0;
    state_.lastAppliedControl.steeringAngle = currentFront;
    state_.lastAppliedControl.acceleration = 0.0;
    return;
  }

  // 初始化暖启动
  if (!state_.initialized) {
    for (int i = 0; i < waypoint_follow::MAX_CONTROL_HORIZON; ++i) {
      state_.prevControlSequence[i].steeringAngle = currentFront;
      state_.prevControlSequence[i].acceleration = 0.0;
    }
    state_.lastAppliedControl = {currentFront, 0.0};
    state_.initialized = true;
  }

  // 调用轨迹拟合子模块
  FitWaypointTrajectoryTraits::Input fitIn;
  for (int i = 0; i < waypoint_follow::NUM_WAYPOINTS; ++i)
    fitIn.waypoints[i] = input.waypoints[i];
  fitIn.numWaypoints = waypoint_follow::NUM_WAYPOINTS;
  fitIn.egoSpeed = input.egoSpeed;
  FitWaypointTrajectoryTraits::Output fitOut;
  sub_.fitTrajectory.run(fitIn, fitOut);

  if (!fitOut.trajectory.valid) {
    output.steeringWheelAngle = input.currentSteeringWheelAngle;
    output.frontWheelAngle = currentFront;
    output.acceleration = 0.0;
    return;
  }

  // 构造MPC优化输入
  MpcOptimizeTraits::Input optIn;
  optIn.initialState = {0.0, 0.0, 0.0, input.egoSpeed};
  optIn.trajectory = fitOut.trajectory;
  optIn.prevAppliedControl = state_.lastAppliedControl;
  const int nc = param_.mpcParam.controlHorizon;
  for (int i = 0; i < nc - 1; ++i) optIn.warmStartSequence[i] = state_.prevControlSequence[i + 1];
  optIn.warmStartSequence[nc - 1] = state_.prevControlSequence[nc - 1];

  // 调用MPC优化子模块
  MpcOptimizeTraits::Output optOut;
  sub_.mpcOptimize.run(optIn, optOut);

  // 限幅输出
  const double maxSW = param_.mpcParam.maxSteeringAngle * param_.steeringRatio;
  ClampValueTraits::Input clIn;
  ClampValueTraits::Output clOut;
  clIn = {optOut.optimalSequence[0].steeringAngle * param_.steeringRatio, -maxSW, maxSW};
  sub_.clampVal.run(clIn, clOut);
  output.steeringWheelAngle = clOut.result;
  output.frontWheelAngle = output.steeringWheelAngle / param_.steeringRatio;
  clIn = {optOut.optimalSequence[0].acceleration, param_.mpcParam.minAcceleration, param_.mpcParam.maxAcceleration};
  sub_.clampVal.run(clIn, clOut);
  output.acceleration = clOut.result;

  // 更新状态
  for (int i = 0; i < nc; ++i) state_.prevControlSequence[i] = optOut.optimalSequence[i];
  state_.lastAppliedControl = optOut.optimalSequence[0];

  // === MBD_AUTO_GEN_END [WaypointFollow] ===
}

} // namespace waypoint_follow_mbd
