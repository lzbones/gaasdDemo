/**
 * @file FitCubicPolynomial_test.cpp
 * @brief FitCubicPolynomial 元件 Traits 级单元测试
 */
#include <iostream>
#include <cmath>
#include "FitCubicPolynomial.hpp"

using namespace waypoint_follow_mbd;

struct TestCase {
  const char* id;
  double xData[7]; double yData[7]; int n;
  double exC[4]; double tol;
};

static bool runTest(const TestCase& tc) {
  FitCubicPolynomial mod;
  FitCubicPolynomialTraits::Input in;
  in.numPoints = tc.n;
  for (int i = 0; i < tc.n; ++i) { in.xData[i] = tc.xData[i]; in.yData[i] = tc.yData[i]; }
  FitCubicPolynomialTraits::Output out;
  mod.run(in, out);

  bool p = out.success;
  for (int i = 0; i < 4 && p; ++i)
    p = (std::fabs(out.coeffs.c[i] - tc.exC[i]) < tc.tol);

  std::cout << (p ? "[PASS] " : "[FAIL] ") << tc.id;
  if (!p) {
    std::cout << "  got=[";
    for (int i = 0; i < 4; ++i) std::cout << out.coeffs.c[i] << (i<3?",":" ");
    std::cout << "] success=" << out.success;
  }
  std::cout << std::endl;
  return p;
}

int main() {
  static const TestCase cases[] = {
    {"FP_TC001",
     {0.0,0.2,0.4,0.6,0.8,1.0,0}, {1.0,1.552,2.536,4.144,6.568,10.0,0}, 6,
     {1.0,2.0,3.0,4.0}, 0.5},
    {"FP_TC002",
     {0.0,0.5,1.0,1.5,2.0,0,0}, {0.0,1.5,3.0,4.5,6.0,0,0}, 5,
     {0.0,3.0,0.0,0.0}, 0.1},
  };
  unsigned int p = 0;
  std::cout << "=== MBD Unit Tests: FitCubicPolynomial ===" << std::endl;
  for (const auto& tc : cases) { if (runTest(tc)) ++p; }
  const auto total = sizeof(cases)/sizeof(cases[0]);
  std::cout << "\nPassed: " << p << "/" << total << std::endl;
  return (p == total) ? 0 : 1;
}
