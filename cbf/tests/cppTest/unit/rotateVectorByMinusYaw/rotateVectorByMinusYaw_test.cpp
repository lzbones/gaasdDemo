/**
 * @file rotateVectorByMinusYaw_test.cpp
 * @brief rotateVectorByMinusYaw 单元测试
 * @cn_name 旋转测试
 * @type element_test
 * @tag_level0 数学运算
 * @tag_level1 坐标变换
 * @tag_level2 二维旋转
 * @tag_level3 主动旋转
 * @version 1.0
 * @date 2026-06-26
 * @author CBF Module Team
 */

#include "rotateVectorByMinusYaw.hpp"

#include <cmath>
#include <cstdio>

namespace {

struct TestCase {
  const char* id;
  double vxGlobal;
  double vyGlobal;
  double yaw;
  double vxEgoExpected;
  double vyEgoExpected;
  double tolerance;
};

constexpr TestCase kCases[] = {
    {"TC001_zero_yaw", 1.0, 2.0, 0.0, 1.0, 2.0, 1e-9},
    {"TC002_pi_over_2", 1.0, 0.0, 1.5707963267948966, 0.0, -1.0, 1e-9},
    {"TC003_minus_pi_over_2", 1.0, 0.0, -1.5707963267948966, 0.0, 1.0, 1e-9},
    {"TC004_pi", 3.0, -2.0, 3.141592653589793, -3.0, 2.0, 1e-9},
};

bool runOne(const TestCase& tc) {
  double vxEgo = 0.0;
  double vyEgo = 0.0;
  cbf::rotateVectorByMinusYaw(tc.vxGlobal, tc.vyGlobal, tc.yaw, vxEgo, vyEgo);
  const bool passed = (std::fabs(vxEgo - tc.vxEgoExpected) < tc.tolerance) &&
                      (std::fabs(vyEgo - tc.vyEgoExpected) < tc.tolerance);
  std::printf("[%s] %s expected=(%.6f,%.6f) got=(%.6f,%.6f)\n",
              passed ? "PASS" : "FAIL",
              tc.id,
              tc.vxEgoExpected,
              tc.vyEgoExpected,
              vxEgo,
              vyEgo);
  return passed;
}

}  // namespace

int main() {
  unsigned int passedCount = 0;
  const unsigned int totalCount =
      static_cast<unsigned int>(sizeof(kCases) / sizeof(kCases[0]));
  for (unsigned int i = 0; i < totalCount; ++i) {
    if (runOne(kCases[i])) {
      passedCount += 1U;
    }
  }
  std::printf("Passed %u/%u\n", passedCount, totalCount);
  const int exitCode = (passedCount == totalCount) ? 0 : 1;
  return exitCode;
}
