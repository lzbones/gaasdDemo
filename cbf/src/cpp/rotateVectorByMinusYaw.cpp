/**
 * @file rotateVectorByMinusYaw.cpp
 * @brief 全局坐标系到 ego 坐标系的二维向量主动旋转实现
 */

#include "rotateVectorByMinusYaw.hpp"

#include <cmath>

namespace cbf {

void rotateVectorByMinusYaw(Real vxGlobal,
                            Real vyGlobal,
                            Real yaw,
                            Real& vxEgo,
                            Real& vyEgo) {
  const Real cosYaw = std::cos(yaw);                                // 偏航角余弦
  const Real sinYaw = std::sin(yaw);                                // 偏航角正弦
  const Real vxEgoLocal = cosYaw * vxGlobal + sinYaw * vyGlobal;    // 旋转后 x 分量
  const Real vyEgoLocal = -sinYaw * vxGlobal + cosYaw * vyGlobal;   // 旋转后 y 分量
  vxEgo = vxEgoLocal;                                               // 单一出口，直接写回
  vyEgo = vyEgoLocal;                                               // 单一出口，直接写回
}

}  // namespace cbf
