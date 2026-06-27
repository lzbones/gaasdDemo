#pragma once

#include "waypointFollowTypes.hpp"

/// @file fitCubicPolynomial.hpp
/// @brief 三次多项式最小二乘拟合声明

namespace waypoint_follow {

/// 对给定数据点进行三次多项式最小二乘拟合 y = c0 + c1*x + c2*x^2 + c3*x^3
/// @param xData X坐标数组 (numPoints个元素)
/// @param yData Y坐标数组 (numPoints个元素)
/// @param numPoints 数据点数量 (需 >= 4)
/// @return 拟合得到的三次多项式系数; 失败时系数全零
PolynomialCoeffs fitCubicPolynomial(const double* xData, const double* yData, int numPoints);

} // namespace waypoint_follow
