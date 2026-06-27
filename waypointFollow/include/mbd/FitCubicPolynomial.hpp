#pragma once
#include "FuncModule.hpp"
#include "waypointFollowTypes.hpp"

namespace waypoint_follow_mbd {

constexpr int MAX_FIT_POINTS = waypoint_follow::NUM_WAYPOINTS + 1;

/// 三次多项式拟合元件 Traits (含内部高斯消元)
struct FitCubicPolynomialTraits {
  struct Input {
    double xData[MAX_FIT_POINTS] = {}; ///< X坐标数组
    double yData[MAX_FIT_POINTS] = {}; ///< Y坐标数组
    int numPoints = 0;                 ///< 数据点数量
  };
  struct Output {
    waypoint_follow::PolynomialCoeffs coeffs; ///< 拟合系数
    bool success = false;                     ///< 拟合是否成功
  };
  struct Param  {};
  struct State  {};
  struct Sub    {};
};

/// 三次多项式最小二乘拟合元件
class FitCubicPolynomial : public FuncModule<FitCubicPolynomialTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

} // namespace waypoint_follow_mbd
