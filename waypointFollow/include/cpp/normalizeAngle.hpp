#pragma once

/// @file normalizeAngle.hpp
/// @brief 角度归一化声明

namespace waypoint_follow {

/// 将角度归一化到 [-π, π] 区间
/// @param angle 待归一化角度 (rad)
/// @return 归一化后的角度 (rad), 取值范围 [-π, π]
double normalizeAngle(double angle);

} // namespace waypoint_follow
