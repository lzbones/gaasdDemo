/**
 * @file solveQpHildreth.cpp
 * @brief Hildreth 对偶坐标下降 QP 求解器实现（决策变量维度=2）
 *
 * 算法说明：
 *   原问题 min 0.5 u^T Qd u + c^T u  s.t.  A u <= b
 *   对偶问题 max -0.5 λ^T H λ - λ^T k  s.t.  λ >= 0
 *   其中 H = A Qd^-1 A^T, k = b + A Qd^-1 c
 *   Hildreth 迭代：λ_i ← max(0, λ_i - (k_i + Σ_j H_ij λ_j)/H_ii)
 *   原始解：u = -Qd^-1 (c + A^T λ)
 *
 * 静态内存上限：约束数最多 MAX_OBSTACLE_NUM + 4(box) = 36，本文件使用上限 64。
 */

#include "solveQpHildreth.hpp"

#include <algorithm>
#include <cmath>

namespace cbf {

namespace {
constexpr std::size_t SOLVER_MAX_CONSTRAINTS = 64U;  ///< 内部静态缓存上限
}

bool solveQpHildreth(Real qa,
                     Real qt,
                     const Real cVec[2],
                     const Real* aMatRowMajor,
                     const Real* bVec,
                     std::size_t numConstraints,
                     std::size_t maxIter,
                     Real tolerance,
                     Real uOut[2],
                     std::size_t& iterUsedOut) {
  const Real qaSafe = (qa > 1e-9) ? qa : 1e-9;                         // 防奇异
  const Real qtSafe = (qt > 1e-9) ? qt : 1e-9;                         // 防奇异
  const Real invQa = 1.0 / qaSafe;                                     // Qd^-1 对角元
  const Real invQt = 1.0 / qtSafe;                                     // Qd^-1 对角元

  const std::size_t mClipped =
      std::min(numConstraints, SOLVER_MAX_CONSTRAINTS);                // 防溢出

  Real hMat[SOLVER_MAX_CONSTRAINTS * SOLVER_MAX_CONSTRAINTS] = {0};    // 对偶 Hessian
  Real kVec[SOLVER_MAX_CONSTRAINTS] = {0};                             // 对偶线性项
  Real lambdaVec[SOLVER_MAX_CONSTRAINTS] = {0};                        // 对偶变量

  for (std::size_t i = 0; i < mClipped; ++i) {                         // 装配 H 与 k
    const Real ai0 = aMatRowMajor[2 * i + 0];
    const Real ai1 = aMatRowMajor[2 * i + 1];
    const Real qiAi0 = invQa * ai0;
    const Real qiAi1 = invQt * ai1;
    for (std::size_t j = 0; j < mClipped; ++j) {
      const Real aj0 = aMatRowMajor[2 * j + 0];
      const Real aj1 = aMatRowMajor[2 * j + 1];
      hMat[i * SOLVER_MAX_CONSTRAINTS + j] = qiAi0 * aj0 + qiAi1 * aj1;
    }
    kVec[i] = bVec[i] + qiAi0 * cVec[0] + qiAi1 * cVec[1];
  }

  std::size_t iter = 0;                                                // 迭代计数
  bool converged = false;                                              // 收敛标志
  Real maxDelta = 0.0;                                                 // 最大变化
  while ((iter < maxIter) && (!converged)) {
    maxDelta = 0.0;
    for (std::size_t i = 0; i < mClipped; ++i) {
      Real residual = kVec[i];
      for (std::size_t j = 0; j < mClipped; ++j) {
        if (j != i) {
          residual += hMat[i * SOLVER_MAX_CONSTRAINTS + j] * lambdaVec[j];
        }
      }
      const Real hii = hMat[i * SOLVER_MAX_CONSTRAINTS + i];
      const Real hiiSafe = (hii > 1e-12) ? hii : 1e-12;
      const Real lambdaCandidate = -residual / hiiSafe;
      const Real lambdaNew = std::max(0.0, lambdaCandidate);
      const Real deltaLambda = std::fabs(lambdaNew - lambdaVec[i]);
      maxDelta = std::max(maxDelta, deltaLambda);
      lambdaVec[i] = lambdaNew;
    }
    converged = (maxDelta < tolerance);
    iter += 1U;
  }

  Real atLambda0 = 0.0;                                                // A^T λ 第 1 分量
  Real atLambda1 = 0.0;                                                // A^T λ 第 2 分量
  for (std::size_t i = 0; i < mClipped; ++i) {
    atLambda0 += aMatRowMajor[2 * i + 0] * lambdaVec[i];
    atLambda1 += aMatRowMajor[2 * i + 1] * lambdaVec[i];
  }
  const Real u0 = -invQa * (cVec[0] + atLambda0);                      // 原始解 a
  const Real u1 = -invQt * (cVec[1] + atLambda1);                      // 原始解 tan(δf)

  Real maxViolation = 0.0;                                             // 约束违反度
  for (std::size_t i = 0; i < mClipped; ++i) {
    const Real lhs =
        aMatRowMajor[2 * i + 0] * u0 + aMatRowMajor[2 * i + 1] * u1;
    const Real violation = lhs - bVec[i];
    maxViolation = std::max(maxViolation, violation);
  }
  const bool feasible = (maxViolation < 1e-3);                         // 1mm/s^2 量级容差

  uOut[0] = u0;                                                        // 单一出口
  uOut[1] = u1;
  iterUsedOut = iter;
  return feasible;
}

}  // namespace cbf
