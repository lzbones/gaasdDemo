#include "EvaluatePolynomialSlope.hpp"

namespace waypoint_follow_mbd {

void EvaluatePolynomialSlope::run(const Input& input, Output& output) {
  const double x = input.x;
  const auto& c = input.coeffs.c;
  output.result = c[1] + x * (2.0 * c[2] + 3.0 * c[3] * x);
}

} // namespace waypoint_follow_mbd
