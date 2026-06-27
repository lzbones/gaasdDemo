#pragma once
/**
 * @file computeBarrierFirstDerivative.hpp
 * @brief 计算 Barrier 一阶时间导数 dot(B)
 */

#include "cbfTypes.hpp"

namespace cbf {

/**
 * @brief 平方型 Barrier 的一阶时间导数 dB/dt
 * @cn_name Barrier 一阶导数
 * @type element
 * @tag_level0 自动驾驶安全控制
 * @tag_level1 控制屏障函数
 * @tag_level2 高阶导数
 * @tag_level3 一阶时间导数
 * @version 1.0
 * @date 2026-06-26
 * @author CBF Module Team
 *
 * 公式：dotB = 2 * (dxEgo * vRxEgo + dyEgo * vRyEgo)
 * 推导：B = dx^2+dy^2-r^2，时间导数等于 2 倍位置矢量与相对速度矢量之内积，
 * 由于内积在旋转下不变，可直接使用 ego frame 的相对位置与相对速度。
 *
 * @param[in] dxEgo  ego frame 纵向相对位置 (m)
 * @param[in] dyEgo  ego frame 横向相对位置 (m)
 * @param[in] vRxEgo ego frame 纵向相对速度 (m/s)
 * @param[in] vRyEgo ego frame 横向相对速度 (m/s)
 * @return dot(B) (m^2/s)
 */
Real computeBarrierFirstDerivative(Real dxEgo,
                                   Real dyEgo,
                                   Real vRxEgo,
                                   Real vRyEgo);

}  // namespace cbf
