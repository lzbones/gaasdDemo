#pragma once
#include "FuncModule.hpp"
#include "PredictBicycleState.hpp"
#include "EvaluatePolynomial.hpp"
#include "EvaluatePolynomialSlope.hpp"
#include "NormalizeAngle.hpp"
#include "waypointFollowTypes.hpp"

namespace waypoint_follow_mbd {

/// MPC代价函数组件 Traits
struct MpcComputeCostTraits {
  struct Input {
    waypoint_follow::ControlInput controlSequence[waypoint_follow::MAX_CONTROL_HORIZON]; ///< 控制序列
    waypoint_follow::VehicleState initialState;    ///< 初始状态
    waypoint_follow::FittedTrajectory trajectory;  ///< 参考轨迹
    waypoint_follow::ControlInput prevAppliedControl; ///< 上一周期控制量
  };
  struct Output {
    double totalCost = 0.0; ///< MPC总代价值
  };
  struct Param {
    waypoint_follow::MpcParam mpcParam; ///< MPC参数
  };
  struct State {};
  struct Sub {
    PredictBicycleState predictState;     ///< 自行车模型预测
    EvaluatePolynomial evalPoly;          ///< 多项式求值
    EvaluatePolynomialSlope evalSlope;    ///< 多项式斜率求值
    NormalizeAngle normAngle;             ///< 角度归一化
  };
};

/// MPC代价函数计算组件
class MpcComputeCost : public FuncModule<MpcComputeCostTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

} // namespace waypoint_follow_mbd
