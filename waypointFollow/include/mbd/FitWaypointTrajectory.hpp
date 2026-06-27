#pragma once
#include "FuncModule.hpp"
#include "FitCubicPolynomial.hpp"
#include "waypointFollowTypes.hpp"

namespace waypoint_follow_mbd {

/// 路点轨迹拟合组件 Traits
struct FitWaypointTrajectoryTraits {
  struct Input {
    waypoint_follow::Waypoint waypoints[waypoint_follow::NUM_WAYPOINTS]; ///< 参考路点
    int numWaypoints = waypoint_follow::NUM_WAYPOINTS;                   ///< 路点数量
    double egoSpeed = 0.0;                                               ///< 自车车速 (m/s)
  };
  struct Output {
    waypoint_follow::FittedTrajectory trajectory; ///< 拟合轨迹
  };
  struct Param {};
  struct State {};
  struct Sub {
    FitCubicPolynomial fitPoly; ///< 多项式拟合子模块
  };
};

/// 路点轨迹拟合组件
class FitWaypointTrajectory : public FuncModule<FitWaypointTrajectoryTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

} // namespace waypoint_follow_mbd
