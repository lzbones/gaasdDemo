#include "fitCubicPolynomial.hpp"
#include "solveLinearSystem.hpp"

namespace waypoint_follow {

/**
 * @brief 对数据点进行三次多项式最小二乘拟合
 * @cn_name 三次多项式拟合
 * @type block
 * @tag_level0 需确认
 * @tag_level1 控制
 * @tag_level2 轨迹跟踪
 * @tag_level3 曲线拟合
 * @version 2.0
 * @date 2026-06-26
 * @author 许庆
 *
 * 构造法方程 (A^T A) c = A^T y, 其中 A 为 Vandermonde 矩阵,
 * 然后调用高斯消元求解得到多项式系数。
 */
PolynomialCoeffs fitCubicPolynomial(const double* xData,
                                     const double* yData,
                                     const int numPoints) {
  constexpr int ORDER = POLY_COEFFS_NUM; // 4个系数 (三次+常数)

  // 构造法方程矩阵 A^T*A 和右端向量 A^T*y
  double ata[MAX_LINEAR_SYSTEM_DIM * MAX_LINEAR_SYSTEM_DIM] = {};
  double aty[MAX_LINEAR_SYSTEM_DIM] = {};

  for (int i = 0; i < numPoints; ++i) {
    // 构造当前数据点的 Vandermonde 行向量 [1, x, x^2, x^3]
    double xPow[POLY_COEFFS_NUM] = {};
    xPow[0] = 1.0;
    for (int j = 1; j < ORDER; ++j) {
      xPow[j] = xPow[j - 1] * xData[i];
    }

    // 累加外积 xPow * xPow^T 到 A^T*A
    for (int r = 0; r < ORDER; ++r) {
      for (int c = 0; c < ORDER; ++c) {
        ata[r * MAX_LINEAR_SYSTEM_DIM + c] += xPow[r] * xPow[c];
      }
      aty[r] += xPow[r] * yData[i];
    }
  }

  // 求解法方程 (A^T A) c = A^T y
  const bool solved = solveLinearSystem(ata, aty, ORDER, MAX_LINEAR_SYSTEM_DIM);

  PolynomialCoeffs result;
  const int coeffCount = solved ? POLY_COEFFS_NUM : 0;
  for (int i = 0; i < coeffCount; ++i) {
    result.c[i] = aty[i];
  }

  return result;
}

} // namespace waypoint_follow
