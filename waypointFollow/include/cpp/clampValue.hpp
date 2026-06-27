#pragma once

/// @file clampValue.hpp
/// @brief 数值限幅截断声明

namespace waypoint_follow {

/// 将数值截断到指定区间 [minVal, maxVal]
/// @param value 待截断数值
/// @param minVal 下限
/// @param maxVal 上限
/// @return 截断后的数值
double clampValue(double value, double minVal, double maxVal);

} // namespace waypoint_follow
