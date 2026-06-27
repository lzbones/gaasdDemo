#pragma once
#include "FuncModule.hpp"
#include "MpcComputeCost.hpp"
#include "ClampValue.hpp"
#include "waypointFollowTypes.hpp"

namespace waypoint_follow_mbd {

/// MPC优化器组件 Traits
struct MpcOptimizeTraits {
  struct Input {
    waypoint_follow::VehicleState initialState;    ///< 初始状态
    waypoint_follow::FittedTrajectory trajectory;  ///< 参考轨迹
    waypoint_follow::ControlInput warmStartSequence[waypoint_follow::MAX_CONTROL_HORIZON]; ///< 暖启动序列
    waypoint_follow::ControlInput prevAppliedControl; ///< 上一周期控制量
  };
  struct Output {
    waypoint_follow::ControlInput optimalSequence[waypoint_follow::MAX_CONTROL_HORIZON]; ///< 最优控制序列
    double finalCost = 0.0; ///< 最终代价
  };
  struct Param {
    waypoint_follow::MpcParam mpcParam; ///< MPC参数
  };
  struct State {};
  struct Sub {
    MpcComputeCost computeCost; ///< 代价函数计算
    ClampValue clampVal;        ///< 数值限幅
  };
};

/// MPC梯度下降优化器组件
class MpcOptimize : public FuncModule<MpcOptimizeTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

} // namespace waypoint_follow_mbd
