#pragma once

#include "waypointFollowTypes.hpp"

/// @file mpcComputeCost.hpp
/// @brief MPC代价函数计算声明

namespace waypoint_follow {

/// 计算给定控制序列对应的MPC总代价
/// 代价 = 轨迹跟踪误差代价 + 控制量代价 + 控制变化率代价
/// @param controlSequence 控制序列 (controlHorizon长度)
/// @param initialState 初始车辆状态
/// @param trajectory 拟合参考轨迹
/// @param prevAppliedControl 上一周期实际施加的控制量 (用于首步变化率计算)
/// @param param MPC参数
/// @return MPC总代价值
double mpcComputeCost(const ControlInput* controlSequence,
                      const VehicleState& initialState,
                      const FittedTrajectory& trajectory,
                      const ControlInput& prevAppliedControl,
                      const MpcParam& param);

} // namespace waypoint_follow
