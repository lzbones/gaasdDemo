/**
 * @file computeBarrierSecondDerivativeFree_test.cpp
 * @brief computeBarrierSecondDerivativeFree 单元测试
 * @cn_name Barrier 二阶导自由项测试
 * @type element_test
 * @tag_level0 自动驾驶安全控制
 * @tag_level1 控制屏障函数
 * @tag_level2 高阶导数
 * @tag_level3 二阶导自由项
 * @version 1.0
 * @date 2026-06-26
 * @author CBF Module Team
 */

#include "computeBarrierSecondDerivativeFree.hpp"

#include <cmath>
#include <cstdio>

namespace {
struct TestCase {
  const char* id;
  double dxEgo;
  double dyEgo;
  double vRxEgo;
  double vRyEgo;
  double axObEgo;
  double ayObEgo;
  double expected;
  double tolerance;
};

constexpr TestCase kCases[] = {
    {"TC001_pure_velocity", 0.0, 0.0, 3.0, 4.0, 0.0, 0.0, 50.0, 1e-9},
    {"TC002_position_accel", 2.0, 3.0, 0.0, 0.0, 1.0, 1.0, 10.0, 1e-9},
    {"TC003_combined", 1.0, 1.0, 1.0, 1.0, 0.5, 0.5, 6.0, 1e-9},
};
}  // namespace

int main() {
  unsigned int passedCount = 0;
  const unsigned int totalCount =
      static_cast<unsigned int>(sizeof(kCases) / sizeof(kCases[0]));
  for (unsigned int i = 0; i < totalCount; ++i) {
    const double got = cbf::computeBarrierSecondDerivativeFree(
        kCases[i].dxEgo,
        kCases[i].dyEgo,
        kCases[i].vRxEgo,
        kCases[i].vRyEgo,
        kCases[i].axObEgo,
        kCases[i].ayObEgo);
    const bool passed = std::fabs(got - kCases[i].expected) < kCases[i].tolerance;
    std::printf("[%s] %s expected=%.6f got=%.6f\n",
                passed ? "PASS" : "FAIL",
                kCases[i].id,
                kCases[i].expected,
                got);
    if (passed) {
      passedCount += 1U;
    }
  }
  std::printf("Passed %u/%u\n", passedCount, totalCount);
  const int exitCode = (passedCount == totalCount) ? 0 : 1;
  return exitCode;
}
