#pragma once

#include "waypointFollowTypes.hpp"

/// @file mpcOptimize.hpp
/// @brief MPC梯度下降优化器声明

namespace waypoint_follow {

/// MPC优化器输出
struct MpcOptimizeOutput {
  ControlInput optimalSequence[MAX_CONTROL_HORIZON]; ///< 优化后的控制序列
  double finalCost = 0.0;                            ///< 优化后的代价值
};

/// 使用数值梯度下降法优化MPC控制序列
/// @param initialState 初始车辆状态
/// @param trajectory 拟合参考轨迹
/// @param warmStartSequence 暖启动初始控制序列
/// @param prevAppliedControl 上一周期实际施加的控制量
/// @param param MPC参数
/// @return 优化结果 (最优控制序列和最终代价)
MpcOptimizeOutput mpcOptimize(const VehicleState& initialState,
                               const FittedTrajectory& trajectory,
                               const ControlInput* warmStartSequence,
                               const ControlInput& prevAppliedControl,
                               const MpcParam& param);

} // namespace waypoint_follow
