/**
 * @file clampControlInput_test.cpp
 * @brief 控制输入限幅单元测试
 * @cn_name 控制输入限幅测试
 * @type element_test
 * @tag_level0 自动驾驶安全控制
 * @tag_level1 信号处理
 * @tag_level2 饱和限幅
 * @tag_level3 双通道限幅
 * @version 1.0
 * @date 2026-06-26
 * @author CBF Module Team
 */

#include "clampControlInput.hpp"

#include <cmath>
#include <cstdio>

namespace {
struct TestCase {
  const char* id;
  double aRaw;
  double dfRaw;
  double aMin;
  double aMax;
  double dfMin;
  double dfMax;
  double aExp;
  double dfExp;
  double tolerance;
};

constexpr TestCase kCases[] = {
    {"TC001_in_range", 1.0, 0.1, -7.0, 7.0, -0.5, 0.5, 1.0, 0.1, 1e-12},
    {"TC002_above_max", 12.0, 1.0, -7.0, 7.0, -0.5, 0.5, 7.0, 0.5, 1e-12},
    {"TC003_below_min", -10.0, -1.0, -7.0, 7.0, -0.5, 0.5, -7.0, -0.5, 1e-12},
};
}  // namespace

int main() {
  unsigned int passedCount = 0;
  const unsigned int totalCount =
      static_cast<unsigned int>(sizeof(kCases) / sizeof(kCases[0]));
  for (unsigned int i = 0; i < totalCount; ++i) {
    double a = 0.0;
    double df = 0.0;
    cbf::clampControlInput(kCases[i].aRaw,
                           kCases[i].dfRaw,
                           kCases[i].aMin,
                           kCases[i].aMax,
                           kCases[i].dfMin,
                           kCases[i].dfMax,
                           a,
                           df);
    const bool passed = (std::fabs(a - kCases[i].aExp) < kCases[i].tolerance) &&
                        (std::fabs(df - kCases[i].dfExp) < kCases[i].tolerance);
    std::printf("[%s] %s expected=(%.4f,%.4f) got=(%.4f,%.4f)\n",
                passed ? "PASS" : "FAIL",
                kCases[i].id,
                kCases[i].aExp,
                kCases[i].dfExp,
                a,
                df);
    if (passed) {
      passedCount += 1U;
    }
  }
  std::printf("Passed %u/%u\n", passedCount, totalCount);
  const int exitCode = (passedCount == totalCount) ? 0 : 1;
  return exitCode;
}
