
/**
 * @file ComputeBarrierFirstDerivative_test.cpp
 */
#include <cmath>
#include <cstdio>
#include "ComputeBarrierFirstDerivative.hpp"

int main() {
  using namespace cbf_arbitration_mbd;
  ComputeBarrierFirstDerivative m;
  ComputeBarrierFirstDerivative::Input in{10.0, 0.0, -2.0, 0.0};
  ComputeBarrierFirstDerivative::Output out{};
  m.run(in, out);
  if (std::fabs(out.barrierDot + 40.0) > 1e-9) return 1;

  in = {3.0, 4.0, 1.0, -1.0};
  m.run(in, out);
  if (std::fabs(out.barrierDot - (-2.0)) > 1e-9) return 2;

  std::printf("ComputeBarrierFirstDerivative OK\n");
  return 0;
}
