#pragma once
/**
 * @file computeBarrierSecondDerivativeFree.hpp
 * @brief 计算 Barrier 二阶导数中与控制输入 u 无关的部分（D-CBF 自然项）
 */

#include "cbfTypes.hpp"

namespace cbf_arbitration {

/**
 * @brief 平方型 Barrier 二阶时间导数中不依赖 u 的部分
 * @cn_name Barrier 二阶导自由项
 * @type element
 * @tag_level0 自动驾驶安全控制
 * @tag_level1 控制屏障函数
 * @tag_level2 高阶导数
 * @tag_level3 二阶导自由项
 * @version 1.0
 * @date 2026-06-26
 * @author CBF Module Team
 *
 * 公式：ddBFree = 2*(vRxEgo^2 + vRyEgo^2) + 2*(dxEgo*axObEgo + dyEgo*ayObEgo)
 * 该项对应论文 (8) 中 2*∂L_f1 h / ∂x_tp · dot(x_tp) + dot(x_tp)^T ∂²h/∂x_tp² · dot(x_tp)
 * 在平方型 Barrier 简化下的解析形式。
 *
 * @param[in] dxEgo    ego frame 纵向相对位置 (m)
 * @param[in] dyEgo    ego frame 横向相对位置 (m)
 * @param[in] vRxEgo   ego frame 纵向相对速度 (m/s)
 * @param[in] vRyEgo   ego frame 横向相对速度 (m/s)
 * @param[in] axObEgo  障碍物绝对加速度 ego frame x 分量 (m/s^2)
 * @param[in] ayObEgo  障碍物绝对加速度 ego frame y 分量 (m/s^2)
 * @return d²B/dt² 中不含 u 的部分 (m^2/s^2)
 */
Real computeBarrierSecondDerivativeFree(Real dxEgo,
                                        Real dyEgo,
                                        Real vRxEgo,
                                        Real vRyEgo,
                                        Real axObEgo,
                                        Real ayObEgo);

}  // namespace cbf_arbitration
