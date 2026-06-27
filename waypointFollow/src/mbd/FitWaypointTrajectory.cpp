#include "FitWaypointTrajectory.hpp"
#include <cmath>

namespace waypoint_follow_mbd {

void FitWaypointTrajectory::run(const Input& input, Output& output) {
  // === MBD_AUTO_GEN_BEGIN [FitWaypointTrajectory] ===

  constexpr int MAX_PTS = MAX_FIT_POINTS;
  double xAll[MAX_PTS] = {};
  double yAll[MAX_PTS] = {};
  xAll[0] = 0.0;
  yAll[0] = 0.0;
  for (int i = 0; i < input.numWaypoints; ++i) {
    xAll[i + 1] = input.waypoints[i].x;
    yAll[i + 1] = input.waypoints[i].y;
  }
  const int totalPts = input.numWaypoints + 1;

  double xMin = xAll[0], xMax = xAll[0];
  for (int i = 1; i < totalPts; ++i) {
    xMin = (xAll[i] < xMin) ? xAll[i] : xMin;
    xMax = (xAll[i] > xMax) ? xAll[i] : xMax;
  }
  const double xRange = xMax - xMin;
  constexpr double MIN_RANGE = 0.5;
  const bool valid = (xRange >= MIN_RANGE);

  // 构造归一化拟合输入
  FitCubicPolynomialTraits::Input fitIn;
  fitIn.numPoints = totalPts;
  for (int i = 0; i < totalPts; ++i) {
    fitIn.xData[i] = valid ? ((xAll[i] - xMin) / xRange) : 0.0;
    fitIn.yData[i] = yAll[i];
  }

  // 调用多项式拟合子模块
  FitCubicPolynomialTraits::Output fitOut;
  sub_.fitPoly.run(fitIn, fitOut);

  // 计算参考车速
  const double d0 = std::sqrt(input.waypoints[0].x * input.waypoints[0].x
                               + input.waypoints[0].y * input.waypoints[0].y);
  double totalDist = d0;
  for (int i = 1; i < input.numWaypoints; ++i) {
    const double dx = input.waypoints[i].x - input.waypoints[i - 1].x;
    const double dy = input.waypoints[i].y - input.waypoints[i - 1].y;
    totalDist += std::sqrt(dx * dx + dy * dy);
  }

  output.trajectory.poly = fitOut.coeffs;
  output.trajectory.xOffset = valid ? xMin : 0.0;
  output.trajectory.xScale = valid ? xRange : 1.0;
  output.trajectory.refSpeed = valid ? (totalDist / 3.0) : input.egoSpeed;
  output.trajectory.valid = valid && fitOut.success;

  // === MBD_AUTO_GEN_END [FitWaypointTrajectory] ===
}

} // namespace waypoint_follow_mbd
