/**
 * @file EvaluatePolynomialSlope_test.cpp
 * @brief EvaluatePolynomialSlope 元件 Traits 级单元测试
 */
#include <iostream>
#include <cmath>
#include "EvaluatePolynomialSlope.hpp"

using namespace waypoint_follow_mbd;

struct TestCase {
  const char* id;
  double c[4]; double x;
  double expected; double tolerance;
};

static bool runTest(const TestCase& tc) {
  EvaluatePolynomialSlope mod;
  EvaluatePolynomialSlopeTraits::Input in;
  for (int i = 0; i < 4; ++i) in.coeffs.c[i] = tc.c[i];
  in.x = tc.x;
  EvaluatePolynomialSlopeTraits::Output out;
  mod.run(in, out);
  const bool passed = std::fabs(out.result - tc.expected) < tc.tolerance;
  std::cout << (passed ? "[PASS] " : "[FAIL] ") << tc.id;
  if (!passed) std::cout << "  expected=" << tc.expected << " got=" << out.result;
  std::cout << std::endl;
  return passed;
}

int main() {
  static const TestCase cases[] = {
    {"ES_TC001", {5.0, 0.0, 0.0, 0.0}, 1.0, 0.0, 1e-12},
    {"ES_TC002", {2.0, 3.0, 0.0, 0.0}, 1.0, 3.0, 1e-12},
    {"ES_TC003", {1.0, 2.0, 3.0, 4.0}, 0.5, 8.0, 1e-10},
  };
  unsigned int p = 0;
  std::cout << "=== MBD Unit Tests: EvaluatePolynomialSlope ===" << std::endl;
  for (const auto& tc : cases) { if (runTest(tc)) ++p; }
  const auto total = sizeof(cases)/sizeof(cases[0]);
  std::cout << "\nPassed: " << p << "/" << total << std::endl;
  return (p == total) ? 0 : 1;
}
