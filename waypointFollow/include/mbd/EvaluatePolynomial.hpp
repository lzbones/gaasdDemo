#pragma once
#include "FuncModule.hpp"
#include "waypointFollowTypes.hpp"

namespace waypoint_follow_mbd {

/// 多项式求值元件 Traits
struct EvaluatePolynomialTraits {
  struct Input  { waypoint_follow::PolynomialCoeffs coeffs; double x = 0.0; };
  struct Output { double result = 0.0; };
  struct Param  {};
  struct State  {};
  struct Sub    {};
};

/// Horner法三次多项式求值元件
class EvaluatePolynomial : public FuncModule<EvaluatePolynomialTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

} // namespace waypoint_follow_mbd
