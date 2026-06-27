
/**
 * @file SolveQpHildreth_test.cpp
 */
#include <cmath>
#include <cstdio>
#include "SolveQpHildreth.hpp"

int main() {
  using namespace cbf_mbd;
  SolveQpHildreth m;
  m.setParam({200, 1e-6});
  SolveQpHildreth::Input in{};
  in.qDiagA = 2.0;
  in.qDiagT = 2.0;
  in.cVec[0] = -2.0;
  in.cVec[1] = -2.0;
  in.aMatRowMajor[0] = 1.0;  in.aMatRowMajor[1] = 0.0;  in.bVec[0] = 0.5;
  in.aMatRowMajor[2] = 0.0;  in.aMatRowMajor[3] = 1.0;  in.bVec[1] = 0.5;
  in.rowCount = 2;
  SolveQpHildreth::Output out{};
  m.run(in, out);
  if (!out.feasible) return 1;
  if (std::fabs(out.uSolution[0] - 0.5) > 1e-3) return 2;
  if (std::fabs(out.uSolution[1] - 0.5) > 1e-3) return 3;
  std::printf("SolveQpHildreth OK iter=%zu u=(%.3f,%.3f)\n",
              out.iterUsed, out.uSolution[0], out.uSolution[1]);
  return 0;
}
