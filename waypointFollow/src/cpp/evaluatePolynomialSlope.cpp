#include "evaluatePolynomialSlope.hpp"

namespace waypoint_follow {

/**
 * @brief 计算三次多项式在给定点的一阶导数
 * @cn_name 多项式斜率求值
 * @type block
 * @tag_level0 需确认
 * @tag_level1 控制
 * @tag_level2 轨迹跟踪
 * @tag_level3 多项式计算
 * @version 2.0
 * @date 2026-06-26
 * @author 许庆
 *
 * dy/dx = c[1] + 2*c[2]*x + 3*c[3]*x^2
 * 采用 Horner 法: dy/dx = c[1] + x*(2*c[2] + 3*c[3]*x)
 */
double evaluatePolynomialSlope(const PolynomialCoeffs& coeffs, const double x) {
  const double result = coeffs.c[1] + x * (2.0 * coeffs.c[2] + 3.0 * coeffs.c[3] * x);
  return result;
}

} // namespace waypoint_follow
