#pragma once
/**
 * @file computeBarrierSquared.hpp
 * @brief 计算针对单个障碍物的平方型 Barrier Function 值 B = dx^2 + dy^2 - r^2
 */

#include "cbfTypes.hpp"

namespace cbf {

/**
 * @brief 计算单个障碍物的平方型 CBF h 值
 * @cn_name 平方距离障碍物 Barrier
 * @type element
 * @tag_level0 自动驾驶安全控制
 * @tag_level1 控制屏障函数
 * @tag_level2 距离类 Barrier
 * @tag_level3 平方距离形式
 * @version 1.0
 * @date 2026-06-26
 * @author CBF Module Team
 *
 * 公式：B = dxEgo^2 + dyEgo^2 - safetyRadius^2
 * 当 B >= 0 时表示自车在该障碍物对应的安全集内。
 *
 * @param[in] dxEgo        ego frame 纵向相对位置 (m)
 * @param[in] dyEgo        ego frame 横向相对位置 (m)
 * @param[in] safetyRadius 安全圆半径 (m)
 * @return B 值（无量纲长度平方差）
 */
Real computeBarrierSquared(Real dxEgo, Real dyEgo, Real safetyRadius);

}  // namespace cbf
