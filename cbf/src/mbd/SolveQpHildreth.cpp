/**
 * @file SolveQpHildreth.cpp
 * @brief Hildreth 对偶坐标下降 QP 求解器（与 cpp/solveQpHildreth 数值等价）
 */

#include "SolveQpHildreth.hpp"

#include <algorithm>
#include <cmath>

namespace cbf_mbd {

void SolveQpHildreth::run(const Input& input, Output& output) {
  const double qaSafe = (input.qDiagA > 1e-9) ? input.qDiagA : 1e-9;
  const double qtSafe = (input.qDiagT > 1e-9) ? input.qDiagT : 1e-9;
  const double invQa = 1.0 / qaSafe;
  const double invQt = 1.0 / qtSafe;
  const std::size_t mClipped = std::min(input.rowCount, MBD_QP_MAX_ROWS);

  double hMat[MBD_QP_MAX_ROWS * MBD_QP_MAX_ROWS] = {0};
  double kVec[MBD_QP_MAX_ROWS] = {0};
  double lambdaVec[MBD_QP_MAX_ROWS] = {0};

  for (std::size_t i = 0; i < mClipped; ++i) {
    const double ai0 = input.aMatRowMajor[2 * i + 0];
    const double ai1 = input.aMatRowMajor[2 * i + 1];
    const double qiAi0 = invQa * ai0;
    const double qiAi1 = invQt * ai1;
    for (std::size_t j = 0; j < mClipped; ++j) {
      const double aj0 = input.aMatRowMajor[2 * j + 0];
      const double aj1 = input.aMatRowMajor[2 * j + 1];
      hMat[i * MBD_QP_MAX_ROWS + j] = qiAi0 * aj0 + qiAi1 * aj1;
    }
    kVec[i] = input.bVec[i] + qiAi0 * input.cVec[0] + qiAi1 * input.cVec[1];
  }

  std::size_t iter = 0;
  bool converged = false;
  while ((iter < param_.maxIter) && (!converged)) {
    double maxDelta = 0.0;
    for (std::size_t i = 0; i < mClipped; ++i) {
      double residual = kVec[i];
      for (std::size_t j = 0; j < mClipped; ++j) {
        if (j != i) {
          residual += hMat[i * MBD_QP_MAX_ROWS + j] * lambdaVec[j];
        }
      }
      const double hii = hMat[i * MBD_QP_MAX_ROWS + i];
      const double hiiSafe = (hii > 1e-12) ? hii : 1e-12;
      const double lambdaCandidate = -residual / hiiSafe;
      const double lambdaNew = std::max(0.0, lambdaCandidate);
      const double deltaLambda = std::fabs(lambdaNew - lambdaVec[i]);
      maxDelta = std::max(maxDelta, deltaLambda);
      lambdaVec[i] = lambdaNew;
    }
    converged = (maxDelta < param_.tolerance);
    iter += 1U;
  }

  double atLambda0 = 0.0;
  double atLambda1 = 0.0;
  for (std::size_t i = 0; i < mClipped; ++i) {
    atLambda0 += input.aMatRowMajor[2 * i + 0] * lambdaVec[i];
    atLambda1 += input.aMatRowMajor[2 * i + 1] * lambdaVec[i];
  }
  const double u0 = -invQa * (input.cVec[0] + atLambda0);
  const double u1 = -invQt * (input.cVec[1] + atLambda1);

  double maxViolation = 0.0;
  for (std::size_t i = 0; i < mClipped; ++i) {
    const double lhs =
        input.aMatRowMajor[2 * i + 0] * u0 + input.aMatRowMajor[2 * i + 1] * u1;
    maxViolation = std::max(maxViolation, lhs - input.bVec[i]);
  }

  output.uSolution[0] = u0;
  output.uSolution[1] = u1;
  output.iterUsed = iter;
  output.feasible = (maxViolation < 1e-3);
}

}  // namespace cbf_mbd
