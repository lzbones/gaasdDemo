#pragma once

#include "waypointFollowTypes.hpp"

/// @file waypointFollow.hpp
/// @brief 路点跟踪MPC控制器顶层入口声明

namespace waypoint_follow {

/// 路点跟踪MPC控制器顶层入口函数
/// 完成轨迹拟合→MPC优化→控制量输出的完整控制周期
/// @param input 控制器输入 (车速、方向盘转角、6个参考路点)
/// @param param 控制器参数 (MPC参数、传动比)
/// @param state 控制器跨周期状态 (暖启动数据), 函数内部更新
/// @return 控制器输出 (目标方向盘转角、目标加速度)
WaypointFollowOutput waypointFollow(const WaypointFollowInput& input,
                                     const WaypointFollowParam& param,
                                     WaypointFollowState& state);

} // namespace waypoint_follow
