/**
 * @file computeBarrierSquared_test.cpp
 * @brief computeBarrierSquared 单元测试
 * @cn_name Barrier 测试
 * @type element_test
 * @tag_level0 自动驾驶安全控制
 * @tag_level1 控制屏障函数
 * @tag_level2 距离类 Barrier
 * @tag_level3 平方距离形式
 * @version 1.0
 * @date 2026-06-26
 * @author CBF Module Team
 */

#include "computeBarrierSquared.hpp"

#include <cmath>
#include <cstdio>

namespace {
struct TestCase {
  const char* id;
  double dxEgo;
  double dyEgo;
  double safetyRadius;
  double expected;
  double tolerance;
};

constexpr TestCase kCases[] = {
    {"TC001_outside_safe", 10.0, 0.0, 4.0, 84.0, 1e-9},
    {"TC002_inside_safe", 1.0, 1.0, 4.0, -14.0, 1e-9},
    {"TC003_on_boundary", 4.0, 0.0, 4.0, 0.0, 1e-9},
};
}  // namespace

int main() {
  unsigned int passedCount = 0;
  const unsigned int totalCount =
      static_cast<unsigned int>(sizeof(kCases) / sizeof(kCases[0]));
  for (unsigned int i = 0; i < totalCount; ++i) {
    const double got =
        cbf::computeBarrierSquared(kCases[i].dxEgo, kCases[i].dyEgo, kCases[i].safetyRadius);
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
