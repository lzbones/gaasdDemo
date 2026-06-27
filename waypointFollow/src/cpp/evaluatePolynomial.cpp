#include "evaluatePolynomial.hpp"

namespace waypoint_follow {

/**
 * @brief 使用Horner法计算三次多项式在给定点的值
 * @cn_name 多项式求值
 * @type block
 * @tag_level0 需确认
 * @tag_level1 控制
 * @tag_level2 轨迹跟踪
 * @tag_level3 多项式计算
 * @version 2.0
 * @date 2026-06-26
 * @author 许庆
 *
 * 采用 Horner 嵌套乘法以减少乘法次数:
 * y = c[0] + x*(c[1] + x*(c[2] + x*c[3]))
 */
double evaluatePolynomial(const PolynomialCoeffs& coeffs, const double x) {
  const double result = coeffs.c[0] + x * (coeffs.c[1] + x * (coeffs.c[2] + x * coeffs.c[3]));
  return result;
}

} // namespace waypoint_follow
