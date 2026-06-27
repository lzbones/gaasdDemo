#pragma once
#include "FuncModule.hpp"

namespace waypoint_follow_mbd {

/// 角度归一化元件 Traits
struct NormalizeAngleTraits {
  struct Input  { double angle = 0.0; };
  struct Output { double result = 0.0; };
  struct Param  {};
  struct State  {};
  struct Sub    {};
};

/// 角度归一化到 [-π, π] 元件
class NormalizeAngle : public FuncModule<NormalizeAngleTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

} // namespace waypoint_follow_mbd
