#pragma once
/**
 * @file RotateVectorByMinusYaw.hpp
 * @brief 全局向量绕 -yaw 旋转到 ego frame 的元件（叶子节点）
 *
 * Traits 五元：
 *   Input  : 全局 x/y 分量及 yaw；
 *   Output : ego frame 下的 x/y 分量；
 *   Param  : 无；
 *   State  : 无；
 *   Sub    : 无。
 */

#include "FuncModule.hpp"

namespace cbf_mbd {

struct RotateVectorByMinusYawTraits {
  struct Input {
    double vxGlobal = 0.0;  ///< 全局横向分量
    double vyGlobal = 0.0;  ///< 全局纵向分量
    double yaw = 0.0;       ///< 自车朝向角 (rad)
  };
  struct Output {
    double vxEgo = 0.0;  ///< ego 系下纵向分量
    double vyEgo = 0.0;  ///< ego 系下横向分量
  };
  struct Param {};
  struct State {};
  struct Sub {};
};

class RotateVectorByMinusYaw : public FuncModule<RotateVectorByMinusYawTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

}  // namespace cbf_mbd
