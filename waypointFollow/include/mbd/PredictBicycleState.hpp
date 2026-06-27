#pragma once
#include "FuncModule.hpp"

namespace waypoint_follow_mbd {

/// 运动学自行车模型预测元件 Traits
struct PredictBicycleStateTraits {
  struct Input {
    double x = 0.0;              ///< 纵向位置 (m)
    double y = 0.0;              ///< 横向位置 (m)
    double yaw = 0.0;            ///< 航向角 (rad)
    double v = 0.0;              ///< 纵向速度 (m/s)
    double steeringAngle = 0.0;  ///< 前轮转向角 (rad)
    double acceleration = 0.0;   ///< 纵向加速度 (m/s^2)
  };
  struct Output {
    double xNext = 0.0;   ///< 下一时刻纵向位置 (m)
    double yNext = 0.0;   ///< 下一时刻横向位置 (m)
    double yawNext = 0.0; ///< 下一时刻航向角 (rad)
    double vNext = 0.0;   ///< 下一时刻纵向速度 (m/s)
  };
  struct Param {
    double dt = 0.1;        ///< 时间步长 (s)
    double wheelbase = 2.85;///< 前后轴距 (m)
  };
  struct State {};
  struct Sub   {};
};

/// 运动学自行车模型单步状态预测元件
class PredictBicycleState : public FuncModule<PredictBicycleStateTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

} // namespace waypoint_follow_mbd
