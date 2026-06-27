#include "ClampValue.hpp"

namespace waypoint_follow_mbd {

void ClampValue::run(const Input& input, Output& output) {
  const double clampedLow = (input.value < input.minVal) ? input.minVal : input.value;
  output.result = (clampedLow > input.maxVal) ? input.maxVal : clampedLow;
}

} // namespace waypoint_follow_mbd
