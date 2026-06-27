#include "EvaluatePolynomial.hpp"

namespace waypoint_follow_mbd {

void EvaluatePolynomial::run(const Input& input, Output& output) {
  const double x = input.x;
  const auto& c = input.coeffs.c;
  output.result = c[0] + x * (c[1] + x * (c[2] + x * c[3]));
}

} // namespace waypoint_follow_mbd
