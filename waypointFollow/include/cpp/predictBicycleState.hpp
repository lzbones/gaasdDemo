#pragma once

#include "waypointFollowTypes.hpp"

/// @file predictBicycleState.hpp
/// @brief 运动学自行车模型单步状态预测声明

namespace waypoint_follow {

/// 基于运动学自行车模型预测下一时刻车辆状态
/// x_next = x + v*cos(yaw)*dt
/// y_next = y + v*sin(yaw)*dt
/// yaw_next = yaw + v*tan(delta)/L * dt
/// v_next = v + a*dt
/// @param state 当前车辆状态
/// @param control 当前控制输入 (前轮转角、加速度)
/// @param dt 时间步长 (s)
/// @param wheelbase 前后轴距 (m)
/// @return 预测的下一时刻车辆状态
VehicleState predictBicycleState(const VehicleState& state,
                                  const ControlInput& control,
                                  double dt,
                                  double wheelbase);

} // namespace waypoint_follow
