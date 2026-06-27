#include "FitCubicPolynomial.hpp"
#include <cmath>

namespace waypoint_follow_mbd {

/// 内部高斯消元求解 (与 Step 01 solveLinearSystem 等价)
static bool solveSystem(double* matrix, double* rhs, int n, int maxDim) {
  bool success = true;
  for (int col = 0; col < n && success; ++col) {
    int pivotRow = col;
    double maxAbs = std::fabs(matrix[col * maxDim + col]);
    for (int row = col + 1; row < n; ++row) {
      const double a = std::fabs(matrix[row * maxDim + col]);
      if (a > maxAbs) { pivotRow = row; maxAbs = a; }
    }
    success = (maxAbs > 1e-12);
    if (success) {
      if (pivotRow != col) {
        for (int j = col; j < n; ++j) {
          const double t = matrix[col * maxDim + j];
          matrix[col * maxDim + j] = matrix[pivotRow * maxDim + j];
          matrix[pivotRow * maxDim + j] = t;
        }
        const double t = rhs[col]; rhs[col] = rhs[pivotRow]; rhs[pivotRow] = t;
      }
      for (int row = col + 1; row < n; ++row) {
        const double f = matrix[row * maxDim + col] / matrix[col * maxDim + col];
        for (int j = col + 1; j < n; ++j)
          matrix[row * maxDim + j] -= f * matrix[col * maxDim + j];
        matrix[row * maxDim + col] = 0.0;
        rhs[row] -= f * rhs[col];
      }
    }
  }
  if (success) {
    for (int row = n - 1; row >= 0; --row) {
      double s = rhs[row];
      for (int j = row + 1; j < n; ++j) s -= matrix[row * maxDim + j] * rhs[j];
      rhs[row] = s / matrix[row * maxDim + row];
    }
  }
  return success;
}

void FitCubicPolynomial::run(const Input& input, Output& output) {
  constexpr int ORD = waypoint_follow::POLY_COEFFS_NUM;
  constexpr int MD = 8;
  double ata[MD * MD] = {};
  double aty[MD] = {};

  for (int i = 0; i < input.numPoints; ++i) {
    double xp[ORD] = {};
    xp[0] = 1.0;
    for (int j = 1; j < ORD; ++j) xp[j] = xp[j - 1] * input.xData[i];
    for (int r = 0; r < ORD; ++r) {
      for (int c = 0; c < ORD; ++c) ata[r * MD + c] += xp[r] * xp[c];
      aty[r] += xp[r] * input.yData[i];
    }
  }

  output.success = solveSystem(ata, aty, ORD, MD);
  for (int i = 0; i < ORD; ++i)
    output.coeffs.c[i] = output.success ? aty[i] : 0.0;
}

} // namespace waypoint_follow_mbd
