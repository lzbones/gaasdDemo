/**
 * @file ClampControlInput.cpp
 * @brief 控制输入双轴 box 限幅
 */

#include "ClampControlInput.hpp"

#include <algorithm>

namespace cbf_arbitration_mbd {

void ClampControlInput::run(const Input& input, Output& output) {
  output.aFinal = std::clamp(input.aRaw, param_.aMin, param_.aMax);
  output.deltaFFinal = std::clamp(input.deltaFRaw, param_.deltaFMin, param_.deltaFMax);
}

}  // namespace cbf_arbitration_mbd
