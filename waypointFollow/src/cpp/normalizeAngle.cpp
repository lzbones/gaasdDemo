#include "normalizeAngle.hpp"
#include <cmath>

namespace waypoint_follow {

/**
 * @brief 将角度归一化到 [-π, π] 区间
 * @cn_name 角度归一化
 * @type block
 * @tag_level0 需确认
 * @tag_level1 控制
 * @tag_level2 轨迹跟踪
 * @tag_level3 角度处理
 * @version 2.0
 * @date 2026-06-26
 * @author 许庆
 *
 * 利用 atan2(sin(θ), cos(θ)) 的性质，将任意角度映射到 [-π, π] 区间。
 */
double normalizeAngle(const double angle) {
  const double result = std::atan2(std::sin(angle), std::cos(angle));
  return result;
}

} // namespace waypoint_follow
