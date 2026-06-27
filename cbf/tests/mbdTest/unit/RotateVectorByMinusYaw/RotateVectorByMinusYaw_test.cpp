
/**
 * @file RotateVectorByMinusYaw_test.cpp
 * @brief MBD RotateVectorByMinusYaw 元件 Traits 级单元测试
 */
#include <cmath>
#include <cstdio>
#include "RotateVectorByMinusYaw.hpp"

int main() {
  using namespace cbf_mbd;
  RotateVectorByMinusYaw m;
  RotateVectorByMinusYaw::Input in{1.0, 0.0, 0.0};
  RotateVectorByMinusYaw::Output out{};
  m.run(in, out);
  if (std::fabs(out.vxEgo - 1.0) > 1e-9 || std::fabs(out.vyEgo) > 1e-9) return 1;

  in = {0.0, 1.0, M_PI / 2.0};
  m.run(in, out);
  if (std::fabs(out.vxEgo - 1.0) > 1e-6 || std::fabs(out.vyEgo) > 1e-6) return 2;

  in = {1.0, 0.0, M_PI / 2.0};
  m.run(in, out);
  if (std::fabs(out.vxEgo) > 1e-6 || std::fabs(out.vyEgo + 1.0) > 1e-6) return 3;

  std::printf("RotateVectorByMinusYaw OK\n");
  return 0;
}
