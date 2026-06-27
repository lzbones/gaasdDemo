#pragma once

#include "waypointFollowTypes.hpp"

/// @file fitWaypointTrajectory.hpp
/// @brief 路点轨迹拟合组件声明

namespace waypoint_follow {

/// 将6个参考路点拟合为三次多项式轨迹并估算参考车速
/// @param waypoints 路点数组 (NUM_WAYPOINTS个), 自车坐标系
/// @param numWaypoints 路点数量
/// @param egoSpeed 自车当前车速 (m/s)
/// @return 拟合后的轨迹 (含多项式系数、归一化参数和参考车速)
FittedTrajectory fitWaypointTrajectory(const Waypoint* waypoints,
                                        int numWaypoints,
                                        double egoSpeed);

} // namespace waypoint_follow
