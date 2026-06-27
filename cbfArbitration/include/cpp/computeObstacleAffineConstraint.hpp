#pragma once
/**
 * @file computeObstacleAffineConstraint.hpp
 * @brief 基于 HOCBF + D-CBF 推导单个障碍物对应的关于控制输入 u=[a, tan(δf)]^T 的仿射约束
 */

#include "cbfTypes.hpp"

namespace cbf_arbitration {

/**
 * @brief 针对单个障碍物计算 HOCBF 仿射约束
 * @cn_name 障碍物 HOCBF 仿射约束生成
 * @type element
 * @tag_level0 自动驾驶安全控制
 * @tag_level1 控制屏障函数
 * @tag_level2 高阶 CBF 约束装配
 * @tag_level3 障碍物子约束
 * @version 1.0
 * @date 2026-06-26
 * @author CBF Module Team
 *
 * 推导基于论文 (8) 的 HOCBF + D-CBF 表达，并结合自车控制仿射模型 (6)：
 *   B  = dxEgo^2 + dyEgo^2 - r_safe^2
 *   dotB = 2(dxEgo*vRxEgo + dyEgo*vRyEgo)
 *   ddBFree = 2(vRxEgo^2+vRyEgo^2) + 2(dxEgo*axObEgo + dyEgo*ayObEgo)
 * 控制输入贡献：
 *   ddB = ddBFree - 2*dxEgo*a - 2*(v^2/L)*dyEgo*tan(δf)
 * HOCBF：
 *   ddB + (α1+α2)*dotB + α1*α2*B >= 0
 * 写成 betaA*a + betaT*tanδ >= bound 的形式。
 *
 * @param[in]  egoVelocity 自车速度 v (m/s)
 * @param[in]  wheelBase   自车轴距 L (m)
 * @param[in]  alpha1      HOCBF 系数 α1
 * @param[in]  alpha2      HOCBF 系数 α2
 * @param[in]  obstacle    障碍物状态（ego frame 相对位置/速度，全局加速度需调用方先转 ego frame）
 * @param[in]  axObEgo     障碍物加速度 ego frame x 分量 (m/s^2)
 * @param[in]  ayObEgo     障碍物加速度 ego frame y 分量 (m/s^2)
 * @param[in]  safetyRadius 安全圆半径 (m)
 * @return 单条仿射约束 (betaA, betaT, bound)
 */
AffineConstraint computeObstacleAffineConstraint(Real egoVelocity,
                                                 Real wheelBase,
                                                 Real alpha1,
                                                 Real alpha2,
                                                 const ObstacleState& obstacle,
                                                 Real axObEgo,
                                                 Real ayObEgo,
                                                 Real safetyRadius);

}  // namespace cbf_arbitration
