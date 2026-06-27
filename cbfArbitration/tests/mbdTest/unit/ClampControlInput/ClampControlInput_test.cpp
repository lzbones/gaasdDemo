
/**
 * @file ClampControlInput_test.cpp
 */
#include <cmath>
#include <cstdio>
#include "ClampControlInput.hpp"

int main() {
  using namespace cbf_arbitration_mbd;
  ClampControlInput m;
  m.setParam({-7.0, 7.0, -0.5, 0.5});
  ClampControlInput::Input in{15.0, 1.0};
  ClampControlInput::Output out{};
  m.run(in, out);
  if (std::fabs(out.aFinal - 7.0) > 1e-9) return 1;
  if (std::fabs(out.deltaFFinal - 0.5) > 1e-9) return 2;

  in = {-10.0, -1.0};
  m.run(in, out);
  if (std::fabs(out.aFinal + 7.0) > 1e-9) return 3;
  if (std::fabs(out.deltaFFinal + 0.5) > 1e-9) return 4;

  std::printf("ClampControlInput OK\n");
  return 0;
}
