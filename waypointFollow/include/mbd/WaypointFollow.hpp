#pragma once
#include "FuncModule.hpp"
#include "FitWaypointTrajectory.hpp"
#include "MpcOptimize.hpp"
#include "ClampValue.hpp"
#include "waypointFollowTypes.hpp"

namespace waypoint_follow_mbd {

/// 路点跟踪MPC控制器顶层组件 Traits
struct WaypointFollowTraits {
  struct Input {
    double egoSpeed = 0.0;                  ///< 自车车速 (m/s)
    double currentSteeringWheelAngle = 0.0; ///< 当前方向盘转角 (rad)
    waypoint_follow::Waypoint waypoints[waypoint_follow::NUM_WAYPOINTS]; ///< 参考路点
  };
  struct Output {
    double steeringWheelAngle = 0.0; ///< 目标方向盘转角 (rad)
    double frontWheelAngle = 0.0;    ///< 目标前轮转角 (rad)
    double acceleration = 0.0;       ///< 目标加速度 (m/s^2)
  };
  struct Param {
    waypoint_follow::MpcParam mpcParam;    ///< MPC参数
    double steeringRatio = 15.0;           ///< 方向盘传动比
  };
  struct State {
    waypoint_follow::ControlInput prevControlSequence[waypoint_follow::MAX_CONTROL_HORIZON]; ///< 暖启动
    waypoint_follow::ControlInput lastAppliedControl; ///< 上一周期控制量
    bool initialized = false;                         ///< 是否已初始化
  };
  struct Sub {
    FitWaypointTrajectory fitTrajectory; ///< 轨迹拟合子模块
    MpcOptimize mpcOptimize;             ///< MPC优化子模块
    ClampValue clampVal;                 ///< 限幅子模块
  };
};

/// 路点跟踪MPC控制器顶层组件
class WaypointFollow : public FuncModule<WaypointFollowTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

} // namespace waypoint_follow_mbd
