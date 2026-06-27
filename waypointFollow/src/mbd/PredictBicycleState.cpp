#include "PredictBicycleState.hpp"
#include <cmath>

namespace waypoint_follow_mbd {

void PredictBicycleState::run(const Input& input, Output& output) {
  const double cosYaw = std::cos(input.yaw);
  const double sinYaw = std::sin(input.yaw);
  const double tanDelta = std::tan(input.steeringAngle);
  const double yawRate = input.v * tanDelta / param_.wheelbase;

  output.xNext   = input.x + input.v * cosYaw * param_.dt;
  output.yNext   = input.y + input.v * sinYaw * param_.dt;
  output.yawNext = input.yaw + yawRate * param_.dt;
  output.vNext   = input.v + input.acceleration * param_.dt;
}

} // namespace waypoint_follow_mbd
