#include "clampValue.hpp"

namespace waypoint_follow {

/**
 * @brief 将数值截断到指定闭区间
 * @cn_name 数值限幅截断
 * @type block
 * @tag_level0 需确认
 * @tag_level1 控制
 * @tag_level2 轨迹跟踪
 * @tag_level3 安全限幅
 * @version 2.0
 * @date 2026-06-26
 * @author 许庆
 */
double clampValue(const double value, const double minVal, const double maxVal) {
  const double clampedLow = (value < minVal) ? minVal : value;
  const double result = (clampedLow > maxVal) ? maxVal : clampedLow;
  return result;
}

} // namespace waypoint_follow
