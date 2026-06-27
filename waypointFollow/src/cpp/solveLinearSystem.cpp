#include "solveLinearSystem.hpp"
#include <cmath>

namespace waypoint_follow {

/**
 * @brief 使用高斯消元法（部分主元选取）求解n×n线性方程组
 * @cn_name 线性方程组求解
 * @type block
 * @tag_level0 需确认
 * @tag_level1 控制
 * @tag_level2 轨迹跟踪
 * @tag_level3 数值求解
 * @version 2.0
 * @date 2026-06-26
 * @author 许庆
 */
bool solveLinearSystem(double* matrix, double* rhs, const int n, const int maxDim) {
  bool success = true;

  // 前向消元：逐列消去下三角元素
  for (int col = 0; col < n && success; ++col) {
    // 部分主元选取：在当前列中选择绝对值最大的行作为主元
    int pivotRow = col;
    double maxAbsVal = std::fabs(matrix[col * maxDim + col]);
    for (int row = col + 1; row < n; ++row) {
      const double absVal = std::fabs(matrix[row * maxDim + col]);
      const bool isBetter = (absVal > maxAbsVal);
      pivotRow = isBetter ? row : pivotRow;
      maxAbsVal = isBetter ? absVal : maxAbsVal;
    }

    // 主元过小则判定为奇异矩阵
    constexpr double SINGULARITY_THRESHOLD = 1e-12;
    success = (maxAbsVal > SINGULARITY_THRESHOLD);

    if (success) {
      // 行交换：将主元行交换到对角线位置
      if (pivotRow != col) {
        for (int j = col; j < n; ++j) {
          const double temp = matrix[col * maxDim + j];
          matrix[col * maxDim + j] = matrix[pivotRow * maxDim + j];
          matrix[pivotRow * maxDim + j] = temp;
        }
        const double tempRhs = rhs[col];
        rhs[col] = rhs[pivotRow];
        rhs[pivotRow] = tempRhs;
      }

      // 消元：将主元列下方所有元素消为零
      const double pivotVal = matrix[col * maxDim + col];
      for (int row = col + 1; row < n; ++row) {
        const double factor = matrix[row * maxDim + col] / pivotVal;
        for (int j = col + 1; j < n; ++j) {
          matrix[row * maxDim + j] -= factor * matrix[col * maxDim + j];
        }
        matrix[row * maxDim + col] = 0.0;
        rhs[row] -= factor * rhs[col];
      }
    }
  }

  // 回代求解：从最后一行向上逐行求解
  if (success) {
    for (int row = n - 1; row >= 0; --row) {
      double sum = rhs[row];
      for (int j = row + 1; j < n; ++j) {
        sum -= matrix[row * maxDim + j] * rhs[j];
      }
      rhs[row] = sum / matrix[row * maxDim + row];
    }
  }

  return success;
}

} // namespace waypoint_follow
