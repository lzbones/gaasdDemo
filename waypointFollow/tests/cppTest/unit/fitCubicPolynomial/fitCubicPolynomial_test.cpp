/**
 * @file fitCubicPolynomial_test.cpp
 * @brief 三次多项式最小二乘拟合单元测试
 * @cn_name 三次多项式拟合测试
 * @type block
 * @tag_level0 需确认
 * @tag_level1 控制
 * @tag_level2 轨迹跟踪
 * @tag_level3 曲线拟合
 * @version 2.0
 * @date 2026-06-26
 * @author 许庆
 */

#include <iostream>
#include <cmath>
#include "fitCubicPolynomial.hpp"
#include "evaluatePolynomial.hpp"

/// 测试用例结构体
struct FitTestCase {
  const char* id;            ///< 测试编号
  const char* description;   ///< 测试意图
  int numPoints;             ///< 数据点数
  double xData[10];          ///< X坐标
  double yData[10];          ///< Y坐标
  double expectedCoeffs[4];  ///< 期望多项式系数
  double coeffTolerance;     ///< 系数容差
  double evalTolerance;      ///< 求值容差
};

static const FitTestCase g_fitTestCases[] = {
  {
    "FIT_TC001",
    "已知三次多项式精确恢复: y = 1 + 2x + 3x^2 + 4x^3",
    6,
    {0.0, 0.2, 0.4, 0.6, 0.8, 1.0},
    {1.0, 1.552, 2.536, 4.144, 6.568, 10.0},
    {1.0, 2.0, 3.0, 4.0},
    0.01,
    0.01
  },
  {
    "FIT_TC002",
    "线性函数: y = 3 + 5x (二次和三次系数应接近零)",
    6,
    {0.0, 0.2, 0.4, 0.6, 0.8, 1.0},
    {3.0, 4.0, 5.0, 6.0, 7.0, 8.0},
    {3.0, 5.0, 0.0, 0.0},
    0.01,
    0.01
  },
  {
    "FIT_TC003",
    "抛物线: y = 2x^2 (常数项和一次项应接近零)",
    6,
    {0.0, 0.2, 0.4, 0.6, 0.8, 1.0},
    {0.0, 0.08, 0.32, 0.72, 1.28, 2.0},
    {0.0, 0.0, 2.0, 0.0},
    0.01,
    0.01
  },
};

bool runFitTest(const FitTestCase& tc) {
  using namespace waypoint_follow;

  const PolynomialCoeffs result = fitCubicPolynomial(
    tc.xData, tc.yData, tc.numPoints);

  // 验证系数
  bool coeffsOk = true;
  for (int i = 0; i < POLY_COEFFS_NUM; ++i) {
    const double err = std::fabs(result.c[i] - tc.expectedCoeffs[i]);
    const bool thisOk = (err < tc.coeffTolerance);
    coeffsOk = coeffsOk && thisOk;
  }

  // 验证各数据点上的拟合精度
  bool evalOk = true;
  for (int i = 0; i < tc.numPoints; ++i) {
    const double yFit = evaluatePolynomial(result, tc.xData[i]);
    const double err = std::fabs(yFit - tc.yData[i]);
    const bool thisOk = (err < tc.evalTolerance);
    evalOk = evalOk && thisOk;
  }

  const bool passed = coeffsOk && evalOk;
  if (passed) {
    std::cout << "[PASS] " << tc.id << ": " << tc.description
              << " | c=[" << result.c[0] << ", " << result.c[1]
              << ", " << result.c[2] << ", " << result.c[3] << "]"
              << std::endl;
  } else {
    std::cerr << "[FAIL] " << tc.id << ": " << tc.description
              << " | c=[" << result.c[0] << ", " << result.c[1]
              << ", " << result.c[2] << ", " << result.c[3] << "]"
              << " coeffsOk=" << coeffsOk << " evalOk=" << evalOk
              << std::endl;
  }

  return passed;
}

int main() {
  unsigned int passed = 0;
  constexpr unsigned int total = sizeof(g_fitTestCases) / sizeof(g_fitTestCases[0]);

  std::cout << "=== FitCubicPolynomial Unit Tests ===" << std::endl;

  for (const auto& tc : g_fitTestCases) {
    const bool ok = runFitTest(tc);
    passed += ok ? 1 : 0;
  }

  std::cout << "\n=== Test Summary ===" << std::endl;
  std::cout << "Passed: " << passed << "/" << total << std::endl;

  const int exitCode = (passed == total) ? 0 : 1;
  return exitCode;
}
