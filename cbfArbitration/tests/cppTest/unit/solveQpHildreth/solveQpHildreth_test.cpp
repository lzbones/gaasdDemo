/**
 * @file solveQpHildreth_test.cpp
 * @brief Hildreth QP 求解器单元测试
 * @cn_name Hildreth QP 测试
 * @type element_test
 * @tag_level0 数学运算
 * @tag_level1 优化求解
 * @tag_level2 二次规划
 * @tag_level3 对偶坐标下降
 * @version 1.0
 * @date 2026-06-26
 * @author CBF Module Team
 *
 * 用例 1：无约束情况下，最优解 u = -Qd^-1 c。
 * 用例 2：仅一条 box 约束 a <= 1，原始无约束最优 a*=2 被压回 1。
 * 用例 3：与原约束 -a <= -3 (即 a>=3)，无约束最优 a*=2 被推到 3。
 */

#include "solveQpHildreth.hpp"

#include <cmath>
#include <cstdio>

int main() {
  unsigned int passedCount = 0;
  unsigned int totalCount = 0;

  // ------------------- TC1 无约束 -------------------
  {
    totalCount += 1U;
    const double qa = 2.0;
    const double qt = 2.0;
    const double c[2] = {-4.0, -2.0};                   // → u* = (2, 1)
    const double aMat[2] = {0.0, 0.0};                   // 占位
    const double bVec[1] = {1e18};                       // 极大上界 → 失效
    double u[2] = {0.0, 0.0};
    std::size_t iter = 0;
    cbf_arbitration::solveQpHildreth(qa, qt, c, aMat, bVec, 1U, 200U, 1e-8, u, iter);
    const bool ok = (std::fabs(u[0] - 2.0) < 1e-4) && (std::fabs(u[1] - 1.0) < 1e-4);
    std::printf("[%s] TC1_unconstrained got=(%.4f,%.4f)\n",
                ok ? "PASS" : "FAIL", u[0], u[1]);
    if (ok) passedCount += 1U;
  }

  // ------------------- TC2 单约束压回 -------------------
  {
    totalCount += 1U;
    const double qa = 2.0;
    const double qt = 2.0;
    const double c[2] = {-4.0, -2.0};                    // 无约束 u*=(2,1)
    const double aMat[2] = {1.0, 0.0};                   // a <= 1
    const double bVec[1] = {1.0};
    double u[2] = {0.0, 0.0};
    std::size_t iter = 0;
    cbf_arbitration::solveQpHildreth(qa, qt, c, aMat, bVec, 1U, 200U, 1e-8, u, iter);
    const bool ok = (std::fabs(u[0] - 1.0) < 1e-3) && (std::fabs(u[1] - 1.0) < 1e-3);
    std::printf("[%s] TC2_clip_top got=(%.4f,%.4f)\n",
                ok ? "PASS" : "FAIL", u[0], u[1]);
    if (ok) passedCount += 1U;
  }

  // ------------------- TC3 单约束推到下界 -------------------
  {
    totalCount += 1U;
    const double qa = 2.0;
    const double qt = 2.0;
    const double c[2] = {-4.0, -2.0};                    // 无约束 u*=(2,1)
    const double aMat[2] = {-1.0, 0.0};                  // -a <= -3 → a >= 3
    const double bVec[1] = {-3.0};
    double u[2] = {0.0, 0.0};
    std::size_t iter = 0;
    cbf_arbitration::solveQpHildreth(qa, qt, c, aMat, bVec, 1U, 200U, 1e-8, u, iter);
    const bool ok = (std::fabs(u[0] - 3.0) < 1e-3) && (std::fabs(u[1] - 1.0) < 1e-3);
    std::printf("[%s] TC3_clip_bottom got=(%.4f,%.4f)\n",
                ok ? "PASS" : "FAIL", u[0], u[1]);
    if (ok) passedCount += 1U;
  }

  std::printf("Passed %u/%u\n", passedCount, totalCount);
  const int exitCode = (passedCount == totalCount) ? 0 : 1;
  return exitCode;
}
