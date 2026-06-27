#pragma once
#include "FuncModule.hpp"
#include "waypointFollowTypes.hpp"

namespace waypoint_follow_mbd {

/// 多项式斜率求值元件 Traits
struct EvaluatePolynomialSlopeTraits {
  struct Input  { waypoint_follow::PolynomialCoeffs coeffs; double x = 0.0; };
  struct Output { double result = 0.0; };
  struct Param  {};
  struct State  {};
  struct Sub    {};
};

/// 三次多项式一阶导数求值元件
class EvaluatePolynomialSlope : public FuncModule<EvaluatePolynomialSlopeTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

} // namespace waypoint_follow_mbd
