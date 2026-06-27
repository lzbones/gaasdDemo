#pragma once
/**
 * @file clampControlInput.hpp
 * @brief 将控制输入 (a, deltaF) 限制在物理可行范围内
 */

#include "cbfTypes.hpp"

namespace cbf_arbitration {

/**
 * @brief 控制输入物理饱和限幅
 * @cn_name 控制输入限幅
 * @type element
 * @tag_level0 自动驾驶安全控制
 * @tag_level1 信号处理
 * @tag_level2 饱和限幅
 * @tag_level3 双通道限幅
 * @version 1.0
 * @date 2026-06-26
 * @author CBF Module Team
 *
 * 使用 std::clamp 进行加速度与前轮转角的双通道限幅，符合“优先复用 STL”规范。
 *
 * @param[in]  aRaw      待限幅加速度 (m/s^2)
 * @param[in]  deltaFRaw 待限幅前轮转角 (rad)
 * @param[in]  aMin      加速度下界 (m/s^2)
 * @param[in]  aMax      加速度上界 (m/s^2)
 * @param[in]  deltaFMin 前轮转角下界 (rad)
 * @param[in]  deltaFMax 前轮转角上界 (rad)
 * @param[out] aClamped      限幅后的加速度
 * @param[out] deltaFClamped 限幅后的前轮转角
 */
void clampControlInput(Real aRaw,
                       Real deltaFRaw,
                       Real aMin,
                       Real aMax,
                       Real deltaFMin,
                       Real deltaFMax,
                       Real& aClamped,
                       Real& deltaFClamped);

}  // namespace cbf_arbitration
