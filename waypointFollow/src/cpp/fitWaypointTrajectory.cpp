#include "fitWaypointTrajectory.hpp"
#include "fitCubicPolynomial.hpp"
#include <cmath>

namespace waypoint_follow {

/**
 * @brief 将参考路点拟合为三次多项式轨迹并估算参考车速
 * @cn_name 路点轨迹拟合
 * @type block
 * @tag_level0 需确认
 * @tag_level1 控制
 * @tag_level2 轨迹跟踪
 * @tag_level3 轨迹拟合
 * @version 2.0
 * @date 2026-06-26
 * @author 许庆
 *
 * 将自车当前位置(0,0)和6个参考路点共7个数据点的X坐标归一化到 [0, 1] 后
 * 进行三次多项式拟合, 避免大数值导致法方程条件数过大。
 * 包含原点确保MPC预测初始阶段的参考轨迹有效。
 */
FittedTrajectory fitWaypointTrajectory(const Waypoint* waypoints,
                                        const int numWaypoints,
                                        const double egoSpeed) {
  // 拟合数据包含原点 (0,0) 和给定路点, 共 numWaypoints+1 个点
  constexpr int MAX_FIT_POINTS = NUM_WAYPOINTS + 1;
  double xAll[MAX_FIT_POINTS] = {};
  double yAll[MAX_FIT_POINTS] = {};

  // 第一个点为自车当前位置 (原点)
  xAll[0] = 0.0;
  yAll[0] = 0.0;
  for (int i = 0; i < numWaypoints; ++i) {
    xAll[i + 1] = waypoints[i].x;
    yAll[i + 1] = waypoints[i].y;
  }
  const int totalPoints = numWaypoints + 1;

  // 计算X坐标范围, 用于归一化
  double xMin = xAll[0];
  double xMax = xAll[0];
  for (int i = 1; i < totalPoints; ++i) {
    xMin = (xAll[i] < xMin) ? xAll[i] : xMin;
    xMax = (xAll[i] > xMax) ? xAll[i] : xMax;
  }
  const double xRange = xMax - xMin;

  // X方向跨度过小时无法有效拟合
  constexpr double MIN_X_RANGE = 0.5; // (m)
  const bool isValidRange = (xRange >= MIN_X_RANGE);
  const double xOffset = isValidRange ? xMin : 0.0;
  const double xScale = isValidRange ? xRange : 1.0;

  // 对X坐标归一化
  double xNorm[MAX_FIT_POINTS] = {};
  for (int i = 0; i < totalPoints; ++i) {
    xNorm[i] = isValidRange ? ((xAll[i] - xMin) / xRange) : 0.0;
  }

  // 在归一化坐标系下拟合三次多项式
  const PolynomialCoeffs poly = isValidRange
    ? fitCubicPolynomial(xNorm, yAll, totalPoints)
    : PolynomialCoeffs{};

  // 根据路点间距估算参考车速
  // 第一段: 原点到第一个路点; 后续: 相邻路点间距
  const double d0 = std::sqrt(waypoints[0].x * waypoints[0].x
                               + waypoints[0].y * waypoints[0].y);
  double totalDist = d0;
  for (int i = 1; i < numWaypoints; ++i) {
    const double dx = waypoints[i].x - waypoints[i - 1].x;
    const double dy = waypoints[i].y - waypoints[i - 1].y;
    totalDist += std::sqrt(dx * dx + dy * dy);
  }
  constexpr double TOTAL_TIME = 3.0; // 6个路点跨越0.5~3.0秒 (s)
  const double refSpeed = isValidRange ? (totalDist / TOTAL_TIME) : egoSpeed;

  FittedTrajectory result;
  result.poly = poly;
  result.xOffset = xOffset;
  result.xScale = xScale;
  result.refSpeed = refSpeed;
  result.valid = isValidRange;

  return result;
}

} // namespace waypoint_follow
