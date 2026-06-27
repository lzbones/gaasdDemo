#include "NormalizeAngle.hpp"
#include <cmath>

namespace waypoint_follow_mbd {

void NormalizeAngle::run(const Input& input, Output& output) {
  output.result = std::atan2(std::sin(input.angle), std::cos(input.angle));
}

} // namespace waypoint_follow_mbd
