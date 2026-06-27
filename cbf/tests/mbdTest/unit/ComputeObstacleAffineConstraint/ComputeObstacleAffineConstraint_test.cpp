
/**
 * @file ComputeObstacleAffineConstraint_test.cpp
 */
#include <cmath>
#include <cstdio>
#include "ComputeObstacleAffineConstraint.hpp"

int main() {
  using namespace cbf_mbd;
  ComputeObstacleAffineConstraint m;
  m.setParam({2.7, 1.5, 1.5});
  m.mutableSub().barrierB.setParam({4.0});

  ComputeObstacleAffineConstraint::Input in{};
  in.egoVelocity = 12.0;
  in.dxEgo = 10.0;
  in.dyEgo = 0.0;
  in.vRxEgo = -2.0;
  in.vRyEgo = 0.0;

  ComputeObstacleAffineConstraint::Output out{};
  m.run(in, out);
  if (std::fabs(out.betaA + 20.0) > 1e-6) return 1;
  if (std::fabs(out.betaT) > 1e-6) return 2;
  std::printf("ComputeObstacleAffineConstraint OK betaA=%.3f bound=%.3f\n",
              out.betaA, out.bound);
  return 0;
}
