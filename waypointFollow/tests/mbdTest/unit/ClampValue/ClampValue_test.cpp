/**
 * @file ClampValue_test.cpp
 * @brief ClampValue 元件 Traits 级单元测试
 */
#include <iostream>
#include <cmath>
#include "ClampValue.hpp"

using namespace waypoint_follow_mbd;

struct TestCase {
  const char* id;
  double value, minVal, maxVal;
  double expected;
  double tolerance;
};

static bool runTest(const TestCase& tc) {
  ClampValue mod;
  ClampValueTraits::Input in{tc.value, tc.minVal, tc.maxVal};
  ClampValueTraits::Output out;
  mod.run(in, out);
  const bool passed = std::fabs(out.result - tc.expected) < tc.tolerance;
  std::cout << (passed ? "[PASS] " : "[FAIL] ") << tc.id;
  if (!passed) std::cout << "  expected=" << tc.expected << " got=" << out.result;
  std::cout << std::endl;
  return passed;
}

int main() {
  static const TestCase cases[] = {
    {"CV_TC001", 5.0, 0.0, 10.0, 5.0, 1e-12},
    {"CV_TC002", -3.0, 0.0, 10.0, 0.0, 1e-12},
    {"CV_TC003", 15.0, 0.0, 10.0, 10.0, 1e-12},
    {"CV_TC004", 0.0, 0.0, 10.0, 0.0, 1e-12},
    {"CV_TC005", 10.0, 0.0, 10.0, 10.0, 1e-12},
  };
  unsigned int p = 0;
  std::cout << "=== MBD Unit Tests: ClampValue ===" << std::endl;
  for (const auto& tc : cases) { if (runTest(tc)) ++p; }
  const auto total = sizeof(cases)/sizeof(cases[0]);
  std::cout << "\nPassed: " << p << "/" << total << std::endl;
  return (p == total) ? 0 : 1;
}
