#pragma once
/**
 * @file ClampControlInput.hpp
 * @brief 控制输入双轴 box 限幅元件
 */

#include "FuncModule.hpp"

namespace cbf_mbd {

struct ClampControlInputTraits {
  struct Input {
    double aRaw = 0.0;
    double deltaFRaw = 0.0;
  };
  struct Output {
    double aFinal = 0.0;
    double deltaFFinal = 0.0;
  };
  struct Param {
    double aMin = -7.0;
    double aMax = 7.0;
    double deltaFMin = -0.5;
    double deltaFMax = 0.5;
  };
  struct State {};
  struct Sub {};
};

class ClampControlInput : public FuncModule<ClampControlInputTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

}  // namespace cbf_mbd
