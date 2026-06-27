#pragma once
/**
 * @file cbfControlRevision.hpp
 * @brief CBF 安全控制修正模块顶层入口（仅考虑周围障碍物，不考虑车道线）
 */

#include "cbfTypes.hpp"

namespace cbf {

/**
 * @brief CBF 安全控制修正模块顶层组件
 * @cn_name CBF 控制修正主入口
 * @type component
 * @tag_level0 自动驾驶安全控制
 * @tag_level1 控制屏障函数
 * @tag_level2 顶层主入口
 * @tag_level3 障碍物-only 场景
 * @version 1.0
 * @date 2026-06-26
 * @author CBF Module Team
 *
 * 功能：基于论文 (12) 在仅有障碍物的场景下，将 EgoState 中给定的原始控制
 * 指令 (aOriginal, deltaFOriginal) 修正为满足所有 HOCBF 约束以及物理饱和的
 * 安全控制指令 (aSafe, deltaFSafe)。
 *
 * 输入 (input)：
 *   - egoState：自车当前运动状态与原始控制指令；
 *   - obstacles：周围障碍物状态数组（ego frame）；
 *   - obstacleNum：实际障碍物数量。
 * 参数 (param)：
 *   - param：CBF 算法参数；
 * 输出 (output)：
 *   - 返回 CbfOutput，包含 aSafe / deltaFSafe / 可行性标志。
 *
 * @param[in] egoState     自车状态与原始控制
 * @param[in] obstacles    障碍物状态数组指针
 * @param[in] obstacleNum  实际障碍物数量（应 <= MAX_OBSTACLE_NUM）
 * @param[in] param        CBF 参数
 * @return CbfOutput 修正后的安全控制
 */
CbfOutput cbfControlRevision(const EgoState& egoState,
                             const ObstacleState* obstacles,
                             std::size_t obstacleNum,
                             const CbfParam& param);

}  // namespace cbf
