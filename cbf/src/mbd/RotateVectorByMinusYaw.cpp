/**
 * @file RotateVectorByMinusYaw.cpp
 * @brief 元件实现：执行 R(-yaw) 旋转
 */

#include "RotateVectorByMinusYaw.hpp"

#include <cmath>

namespace cbf_mbd {

void RotateVectorByMinusYaw::run(const Input& input, Output& output) {
  const double cosYaw = std::cos(input.yaw);
  const double sinYaw = std::sin(input.yaw);
  output.vxEgo = cosYaw * input.vxGlobal + sinYaw * input.vyGlobal;
  output.vyEgo = -sinYaw * input.vxGlobal + cosYaw * input.vyGlobal;
}

}  // namespace cbf_mbd
