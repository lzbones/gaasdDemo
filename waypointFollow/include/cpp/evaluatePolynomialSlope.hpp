#pragma once

#include "waypointFollowTypes.hpp"

/// @file evaluatePolynomialSlope.hpp
/// @brief 三次多项式一阶导数求值声明

namespace waypoint_follow {

/// 计算三次多项式在给定点的一阶导数（斜率）
/// dy/dx = c[1] + 2*c[2]*x + 3*c[3]*x^2
/// @param coeffs 多项式系数
/// @param x 自变量值
/// @return 多项式在x处的斜率
double evaluatePolynomialSlope(const PolynomialCoeffs& coeffs, double x);

} // namespace waypoint_follow
