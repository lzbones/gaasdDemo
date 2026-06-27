/**
 * @file cbfControlRevision_test.cpp
 * @brief CBF 顶层入口单元测试
 * @cn_name CBF 控制修正测试
 * @type component_test
 * @tag_level0 自动驾驶安全控制
 * @tag_level1 控制屏障函数
 * @tag_level2 顶层主入口
 * @tag_level3 障碍物-only 场景
 * @version 1.0
 * @date 2026-06-26
 * @author CBF Module Team
 *
 * 用例 1：障碍物在远处不威胁 → 安全输出应非常接近原始指令；
 * 用例 2：正前方近距离障碍物即将相撞 → 修正后加速度应明显小于 0；
 * 用例 3：无障碍物 → 输出 = 原始（限幅范围内）。
 */

#include "cbfControlRevision.hpp"

#include <cmath>
#include <cstdio>

namespace {
cbf::CbfParam makeDefaultParam() {
  cbf::CbfParam param;
  param.safetyRadius = 4.0;
  param.alpha1 = 1.5;
  param.alpha2 = 1.5;
  param.wheelBase = 2.7;
  param.aMin = -7.0;
  param.aMax = 7.0;
  param.deltaFMin = -0.5;
  param.deltaFMax = 0.5;
  param.qDiagAccel = 1.0;
  param.qDiagSteer = 5.0;
  param.qpMaxIter = 200U;
  param.qpTolerance = 1e-8;
  return param;
}
}  // namespace

int main() {
  unsigned int passedCount = 0;
  unsigned int totalCount = 0;
  const cbf::CbfParam param = makeDefaultParam();

  // TC1: 远处障碍物
  {
    totalCount += 1U;
    cbf::EgoState ego;
    ego.velocity = 5.0;
    ego.aOriginal = 1.0;
    ego.deltaFOriginal = 0.0;
    cbf::ObstacleState obs[1];
    obs[0].dxEgo = 80.0;
    obs[0].dyEgo = 0.0;
    obs[0].vRxEgo = 0.0;
    obs[0].vRyEgo = 0.0;
    const cbf::CbfOutput out = cbf::cbfControlRevision(ego, obs, 1U, param);
    const bool ok = (std::fabs(out.aSafe - 1.0) < 0.05) &&
                    (std::fabs(out.deltaFSafe) < 0.01) && out.feasible;
    std::printf("[%s] TC1_far_obstacle aSafe=%.4f dfSafe=%.4f feasible=%d\n",
                ok ? "PASS" : "FAIL", out.aSafe, out.deltaFSafe, out.feasible);
    if (ok) passedCount += 1U;
  }

  // TC2: 紧贴前方接近障碍
  {
    totalCount += 1U;
    cbf::EgoState ego;
    ego.velocity = 8.0;
    ego.aOriginal = 2.0;
    ego.deltaFOriginal = 0.0;
    cbf::ObstacleState obs[1];
    obs[0].dxEgo = 6.0;
    obs[0].dyEgo = 0.0;
    obs[0].vRxEgo = -8.0;  // 障碍物相对自车向后逼近
    obs[0].vRyEgo = 0.0;
    const cbf::CbfOutput out = cbf::cbfControlRevision(ego, obs, 1U, param);
    const bool ok = (out.aSafe < 0.0);
    std::printf("[%s] TC2_imminent_collision aSafe=%.4f dfSafe=%.4f\n",
                ok ? "PASS" : "FAIL", out.aSafe, out.deltaFSafe);
    if (ok) passedCount += 1U;
  }

  // TC3: 无障碍物
  {
    totalCount += 1U;
    cbf::EgoState ego;
    ego.velocity = 5.0;
    ego.aOriginal = 0.5;
    ego.deltaFOriginal = 0.05;
    const cbf::CbfOutput out = cbf::cbfControlRevision(ego, nullptr, 0U, param);
    const bool ok = (std::fabs(out.aSafe - 0.5) < 1e-3) &&
                    (std::fabs(out.deltaFSafe - 0.05) < 1e-3);
    std::printf("[%s] TC3_no_obstacle aSafe=%.4f dfSafe=%.4f\n",
                ok ? "PASS" : "FAIL", out.aSafe, out.deltaFSafe);
    if (ok) passedCount += 1U;
  }

  std::printf("Passed %u/%u\n", passedCount, totalCount);
  const int exitCode = (passedCount == totalCount) ? 0 : 1;
  return exitCode;
}
