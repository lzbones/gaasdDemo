/**
 * @file PredictBicycleState_test.cpp
 * @brief PredictBicycleState 元件 Traits 级单元测试
 */
#include <iostream>
#include <cmath>
#include "PredictBicycleState.hpp"

using namespace waypoint_follow_mbd;

struct TestCase {
  const char* id;
  double x, y, yaw, v, delta, accel;
  double dt, wb;
  double exX, exY, exYaw, exV;
  double tol;
};

static bool runTest(const TestCase& tc) {
  PredictBicycleState mod(PredictBicycleStateTraits::Param{tc.dt, tc.wb});
  PredictBicycleStateTraits::Input in{tc.x, tc.y, tc.yaw, tc.v, tc.delta, tc.accel};
  PredictBicycleStateTraits::Output out;
  mod.run(in, out);
  const bool p = std::fabs(out.xNext - tc.exX) < tc.tol
              && std::fabs(out.yNext - tc.exY) < tc.tol
              && std::fabs(out.yawNext - tc.exYaw) < tc.tol
              && std::fabs(out.vNext - tc.exV) < tc.tol;
  std::cout << (p ? "[PASS] " : "[FAIL] ") << tc.id;
  if (!p) std::cout << "  got=(" << out.xNext << "," << out.yNext << "," << out.yawNext << "," << out.vNext << ")";
  std::cout << std::endl;
  return p;
}

int main() {
  static const TestCase cases[] = {
    {"PBS_TC001", 0,0,0,10, 0,0, 0.1,2.85, 1.0,0.0,0.0,10.0, 1e-10},
    {"PBS_TC002", 0,0,0,0, 0,2.0, 0.1,2.85, 0.0,0.0,0.0,0.2, 1e-10},
    {"PBS_TC003", 0,0,0,10, 0.1,0, 0.1,2.85, 1.0,0.0,0.035128,10.0, 0.001},
  };
  unsigned int p = 0;
  std::cout << "=== MBD Unit Tests: PredictBicycleState ===" << std::endl;
  for (const auto& tc : cases) { if (runTest(tc)) ++p; }
  const auto total = sizeof(cases)/sizeof(cases[0]);
  std::cout << "\nPassed: " << p << "/" << total << std::endl;
  return (p == total) ? 0 : 1;
}
