#pragma once

#include "waypointFollowTypes.hpp"

/// @file evaluatePolynomial.hpp
/// @brief 三次多项式求值声明

namespace waypoint_follow {

/// 使用Horner法计算三次多项式在给定点的值
/// y = c[0] + c[1]*x + c[2]*x^2 + c[3]*x^3
/// @param coeffs 多项式系数
/// @param x 自变量值
/// @return 多项式在x处的函数值
double evaluatePolynomial(const PolynomialCoeffs& coeffs, double x);

} // namespace waypoint_follow
