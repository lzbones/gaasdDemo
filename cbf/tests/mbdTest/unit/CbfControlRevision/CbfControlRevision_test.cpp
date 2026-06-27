
/**
 * @file CbfControlRevision_test.cpp
 */
#include <cmath>
#include <cstdio>
#include "CbfControlRevision.hpp"

int main() {
  using namespace cbf_mbd;
  CbfControlRevision m;
  CbfControlRevision::Param p{};
  p.safetyRadius = 5.0;
  p.alpha1 = 1.2;
  p.alpha2 = 1.2;
  p.qDiagAccel = 1.0;
  p.qDiagSteer = 5.0;
  m.setParam(p);

  CbfControlRevision::Input in{};
  in.egoState.velocity = 12.0;
  in.egoState.aOriginal = 1.5;
  in.egoState.deltaFOriginal = 0.0;
  in.obstacles[0].dxEgo = 8.0;
  in.obstacles[0].dyEgo = 0.0;
  in.obstacles[0].vRxEgo = -3.0;
  in.obstacles[0].vRyEgo = 0.0;
  in.obstacleNum = 1;

  CbfControlRevision::Output out{};
  m.run(in, out);
  if (!out.feasible) return 1;
  if (out.aSafe >= in.egoState.aOriginal) {
    std::printf("WARN: aSafe %.3f not reduced from %.3f\n",
                out.aSafe, in.egoState.aOriginal);
  }
  std::printf("CbfControlRevision OK aSafe=%.3f deltaSafe=%.3f feasible=%d\n",
              out.aSafe, out.deltaFSafe, (int)out.feasible);
  return 0;
}
