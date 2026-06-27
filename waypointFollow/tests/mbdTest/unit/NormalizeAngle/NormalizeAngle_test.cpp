/**
 * @file NormalizeAngle_test.cpp
 * @brief NormalizeAngle 元件 Traits 级单元测试
 */
#include <iostream>
#include <cmath>
#include "NormalizeAngle.hpp"

using namespace waypoint_follow_mbd;

struct TestCase {
  const char* id;
  double angle;
  double expected;
  double tolerance;
};

static bool runTest(const TestCase& tc) {
  NormalizeAngle mod;
  NormalizeAngleTraits::Input in{tc.angle};
  NormalizeAngleTraits::Output out;
  mod.run(in, out);
  const bool passed = std::fabs(out.result - tc.expected) < tc.tolerance;
  std::cout << (passed ? "[PASS] " : "[FAIL] ") << tc.id;
  if (!passed) std::cout << "  expected=" << tc.expected << " got=" << out.result;
  std::cout << std::endl;
  return passed;
}

int main() {
  constexpr double PI = 3.14159265358979323846;
  static const TestCase cases[] = {
    {"NA_TC001", 0.0, 0.0, 1e-12},
    {"NA_TC002", 2.0*PI + 1.0, 1.0, 0.01},
    {"NA_TC003", -2.0*PI - 1.0, -1.0, 0.01},
    {"NA_TC004", PI, PI, 1e-6},
    {"NA_TC005", -0.5, -0.5, 1e-12},
  };
  unsigned int p = 0;
  std::cout << "=== MBD Unit Tests: NormalizeAngle ===" << std::endl;
  for (const auto& tc : cases) { if (runTest(tc)) ++p; }
  const auto total = sizeof(cases)/sizeof(cases[0]);
  std::cout << "\nPassed: " << p << "/" << total << std::endl;
  return (p == total) ? 0 : 1;
}
