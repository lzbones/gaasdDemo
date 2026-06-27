/**
 * @file computeObstacleAffineConstraint_test.cpp
 * @brief HOCBF 仿射约束生成测试
 * @cn_name 障碍物 HOCBF 约束生成测试
 * @type element_test
 * @tag_level0 自动驾驶安全控制
 * @tag_level1 控制屏障函数
 * @tag_level2 高阶 CBF 约束装配
 * @tag_level3 障碍物子约束
 * @version 1.0
 * @date 2026-06-26
 * @author CBF Module Team
 */

#include "computeObstacleAffineConstraint.hpp"

#include <cmath>
#include <cstdio>

namespace {
struct TestCase {
  const char* id;
  double egoVelocity;
  double wheelBase;
  double alpha1;
  double alpha2;
  double dxEgo;
  double dyEgo;
  double vRxEgo;
  double vRyEgo;
  double axObEgo;
  double ayObEgo;
  double safetyRadius;
  double betaAExp;
  double betaTExp;
  double boundExp;
  double tolerance;
};

constexpr TestCase kCases[] = {
    {"TC001_static_far", 10.0, 2.7, 1.0, 1.0, 20.0, 0.0, -10.0, 0.0,
     0.0, 0.0, 4.0, -40.0, 0.0,
     -(200.0 + 2.0 * (-400.0) + 1.0 * (400.0 - 16.0)), 1e-6},
    {"TC002_zero_velocity_no_steer_coupling", 0.0, 2.7, 1.0, 1.0, 5.0, 3.0,
     -1.0, 0.0, 0.0, 0.0, 4.0, -10.0, 0.0,
     -(2.0 * 1.0 + 2.0 * (-10.0) + 1.0 * (5.0 * 5.0 + 3.0 * 3.0 - 16.0)), 1e-6},
};
}  // namespace

int main() {
  unsigned int passedCount = 0;
  const unsigned int totalCount =
      static_cast<unsigned int>(sizeof(kCases) / sizeof(kCases[0]));
  for (unsigned int i = 0; i < totalCount; ++i) {
    cbf_arbitration::ObstacleState ob;
    ob.dxEgo = kCases[i].dxEgo;
    ob.dyEgo = kCases[i].dyEgo;
    ob.vRxEgo = kCases[i].vRxEgo;
    ob.vRyEgo = kCases[i].vRyEgo;
    const cbf_arbitration::AffineConstraint c = cbf_arbitration::computeObstacleAffineConstraint(
        kCases[i].egoVelocity,
        kCases[i].wheelBase,
        kCases[i].alpha1,
        kCases[i].alpha2,
        ob,
        kCases[i].axObEgo,
        kCases[i].ayObEgo,
        kCases[i].safetyRadius);
    const bool passed =
        (std::fabs(c.betaA - kCases[i].betaAExp) < kCases[i].tolerance) &&
        (std::fabs(c.betaT - kCases[i].betaTExp) < kCases[i].tolerance) &&
        (std::fabs(c.bound - kCases[i].boundExp) < kCases[i].tolerance);
    std::printf("[%s] %s expected=(%.4f,%.4f,%.4f) got=(%.4f,%.4f,%.4f)\n",
                passed ? "PASS" : "FAIL",
                kCases[i].id,
                kCases[i].betaAExp,
                kCases[i].betaTExp,
                kCases[i].boundExp,
                c.betaA,
                c.betaT,
                c.bound);
    if (passed) {
      passedCount += 1U;
    }
  }
  std::printf("Passed %u/%u\n", passedCount, totalCount);
  const int exitCode = (passedCount == totalCount) ? 0 : 1;
  return exitCode;
}
