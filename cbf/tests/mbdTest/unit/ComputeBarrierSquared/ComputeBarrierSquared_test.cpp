
/**
 * @file ComputeBarrierSquared_test.cpp
 */
#include <cmath>
#include <cstdio>
#include "ComputeBarrierSquared.hpp"

int main() {
  using namespace cbf_mbd;
  ComputeBarrierSquared m;
  m.setParam({4.0});
  ComputeBarrierSquared::Input in{10.0, 0.0};
  ComputeBarrierSquared::Output out{};
  m.run(in, out);
  if (std::fabs(out.barrier - 84.0) > 1e-9) return 1;

  in = {3.0, 4.0};
  m.run(in, out);
  if (std::fabs(out.barrier - 9.0) > 1e-9) return 2;

  in = {0.0, 0.0};
  m.run(in, out);
  if (std::fabs(out.barrier + 16.0) > 1e-9) return 3;

  std::printf("ComputeBarrierSquared OK\n");
  return 0;
}
