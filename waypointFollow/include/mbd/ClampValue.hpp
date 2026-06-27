#pragma once
#include "FuncModule.hpp"

namespace waypoint_follow_mbd {

/// 数值限幅截断元件 Traits
struct ClampValueTraits {
  struct Input  { double value = 0.0; double minVal = 0.0; double maxVal = 0.0; };
  struct Output { double result = 0.0; };
  struct Param  {};
  struct State  {};
  struct Sub    {};
};

/// 数值限幅截断元件
class ClampValue : public FuncModule<ClampValueTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

} // namespace waypoint_follow_mbd
