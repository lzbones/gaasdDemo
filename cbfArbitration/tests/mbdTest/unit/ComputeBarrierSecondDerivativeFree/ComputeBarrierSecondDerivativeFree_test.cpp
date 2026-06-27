
/**
 * @file ComputeBarrierSecondDerivativeFree_test.cpp
 */
#include <cmath>
#include <cstdio>
#include "ComputeBarrierSecondDerivativeFree.hpp"

int main() {
  using namespace cbf_arbitration_mbd;
  ComputeBarrierSecondDerivativeFree m;
  ComputeBarrierSecondDerivativeFree::Input in{10.0, 0.0, -2.0, 0.0, 0.0, 0.0};
  ComputeBarrierSecondDerivativeFree::Output out{};
  m.run(in, out);
  if (std::fabs(out.barrierDdotFree - 8.0) > 1e-9) return 1;

  in = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  m.run(in, out);
  if (std::fabs(out.barrierDdotFree - 8.0) > 1e-9) return 2;

  std::printf("ComputeBarrierSecondDerivativeFree OK\n");
  return 0;
}
