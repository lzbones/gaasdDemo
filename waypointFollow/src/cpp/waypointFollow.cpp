#include "waypointFollow.hpp"
#include "fitWaypointTrajectory.hpp"
#include "mpcOptimize.hpp"
#include "clampValue.hpp"

namespace waypoint_follow {

/**
 * @brief 路点跟踪MPC控制器顶层入口
 * @cn_name 路点跟踪控制
 * @type block
 * @tag_level0 需确认
 * @tag_level1 控制
 * @tag_level2 轨迹跟踪
 * @tag_level3 路点跟踪MPC
 * @version 2.0
 * @date 2026-06-26
 * @author 许庆
 *
 * 控制流程:
 *   (1) 方向盘转角换算为前轮转角
 *   (2) 6个路点拟合为三次多项式轨迹
 *   (3) MPC优化求解最优控制序列
 *   (4) 提取首步控制量, 换算回方向盘转角
 *   (5) 更新暖启动状态
 */
WaypointFollowOutput waypointFollow(const WaypointFollowInput& input,
                                     const WaypointFollowParam& param,
                                     WaypointFollowState& state) {
  WaypointFollowOutput output;

  // 当前前轮转角 = 方向盘转角 / 传动比
  const double currentFrontWheelAngle =
    input.currentSteeringWheelAngle / param.steeringRatio;

  // 低速保护: 车速过低时输出零控制
  const bool isLowSpeed = (input.egoSpeed < MIN_SPEED_THRESHOLD);
  if (isLowSpeed) {
    output.steeringWheelAngle = input.currentSteeringWheelAngle;
    output.frontWheelAngle = currentFrontWheelAngle;
    output.acceleration = 0.0;
    state.lastAppliedControl.steeringAngle = currentFrontWheelAngle;
    state.lastAppliedControl.acceleration = 0.0;

    return output;
  }

  // 初始化暖启动序列
  if (!state.initialized) {
    for (int i = 0; i < MAX_CONTROL_HORIZON; ++i) {
      state.prevControlSequence[i].steeringAngle = currentFrontWheelAngle;
      state.prevControlSequence[i].acceleration = 0.0;
    }
    state.lastAppliedControl.steeringAngle = currentFrontWheelAngle;
    state.lastAppliedControl.acceleration = 0.0;
    state.initialized = true;
  }

  // 路点拟合为三次多项式轨迹
  const FittedTrajectory trajectory =
    fitWaypointTrajectory(input.waypoints, NUM_WAYPOINTS, input.egoSpeed);

  // 拟合失败时保持当前状态
  if (!trajectory.valid) {
    output.steeringWheelAngle = input.currentSteeringWheelAngle;
    output.frontWheelAngle = currentFrontWheelAngle;
    output.acceleration = 0.0;

    return output;
  }

  // 构造初始状态 (自车坐标系原点, 航向为0)
  VehicleState initialState;
  initialState.x = 0.0;
  initialState.y = 0.0;
  initialState.yaw = 0.0;
  initialState.v = input.egoSpeed;

  // 构造暖启动序列: 将上次控制序列左移一步
  ControlInput warmStart[MAX_CONTROL_HORIZON] = {};
  const int nc = param.mpcParam.controlHorizon;
  for (int i = 0; i < nc - 1; ++i) {
    warmStart[i] = state.prevControlSequence[i + 1];
  }
  warmStart[nc - 1] = state.prevControlSequence[nc - 1];

  // MPC优化求解
  const MpcOptimizeOutput mpcResult = mpcOptimize(
    initialState, trajectory, warmStart,
    state.lastAppliedControl, param.mpcParam);

  // 提取首步最优控制量
  const double optimalDelta = mpcResult.optimalSequence[0].steeringAngle;
  const double optimalAccel = mpcResult.optimalSequence[0].acceleration;

  // 前轮转角换算为方向盘转角
  const double outputSteeringWheelAngle = optimalDelta * param.steeringRatio;

  // 限幅保护
  const double maxSteeringWheelAngle =
    param.mpcParam.maxSteeringAngle * param.steeringRatio;
  output.steeringWheelAngle = clampValue(
    outputSteeringWheelAngle,
    -maxSteeringWheelAngle, maxSteeringWheelAngle);
  output.frontWheelAngle = output.steeringWheelAngle / param.steeringRatio;
  output.acceleration = clampValue(
    optimalAccel,
    param.mpcParam.minAcceleration, param.mpcParam.maxAcceleration);

  // 更新状态: 保存当前控制序列和已施加控制量
  for (int i = 0; i < nc; ++i) {
    state.prevControlSequence[i] = mpcResult.optimalSequence[i];
  }
  state.lastAppliedControl.steeringAngle = optimalDelta;
  state.lastAppliedControl.acceleration = optimalAccel;

  return output;
}

} // namespace waypoint_follow
