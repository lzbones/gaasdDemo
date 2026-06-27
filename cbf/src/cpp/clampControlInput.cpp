/**
 * @file clampControlInput.cpp
 * @brief 控制输入双通道限幅实现
 */

#include "clampControlInput.hpp"

#include <algorithm>

namespace cbf {

void clampControlInput(Real aRaw,
                       Real deltaFRaw,
                       Real aMin,
                       Real aMax,
                       Real deltaFMin,
                       Real deltaFMax,
                       Real& aClamped,
                       Real& deltaFClamped) {
  const Real aResult = std::clamp(aRaw, aMin, aMax);                  // 加速度限幅
  const Real deltaFResult = std::clamp(deltaFRaw, deltaFMin, deltaFMax); // 转角限幅
  aClamped = aResult;                                                 // 单一出口
  deltaFClamped = deltaFResult;                                       // 单一出口
}

}  // namespace cbf
