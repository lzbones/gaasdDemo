#pragma once
/**
 * @file rotateVectorByMinusYaw.hpp
 * @brief 将一个全局坐标系下的二维向量旋转 -yaw 角到 ego 坐标系
 */

#include "cbfTypes.hpp"

namespace cbf {

/**
 * @brief 将全局坐标系下的二维向量 (vxGlobal, vyGlobal) 投影到 ego 坐标系
 * @cn_name 全局向量到自车坐标系投影
 * @type element
 * @tag_level0 数学运算
 * @tag_level1 坐标变换
 * @tag_level2 二维旋转
 * @tag_level3 主动旋转
 * @version 1.0
 * @date 2026-06-26
 * @author CBF Module Team
 *
 * 等价于左乘旋转矩阵 R(-yaw)：
 *   vxEgo =  cos(yaw) * vxGlobal + sin(yaw) * vyGlobal
 *   vyEgo = -sin(yaw) * vxGlobal + cos(yaw) * vyGlobal
 *
 * @param[in]  vxGlobal 输入向量全局 x 分量
 * @param[in]  vyGlobal 输入向量全局 y 分量
 * @param[in]  yaw      自车朝向角 (rad)
 * @param[out] vxEgo    输出向量在 ego 坐标系下 x 分量
 * @param[out] vyEgo    输出向量在 ego 坐标系下 y 分量
 */
void rotateVectorByMinusYaw(Real vxGlobal,
                            Real vyGlobal,
                            Real yaw,
                            Real& vxEgo,
                            Real& vyEgo);

}  // namespace cbf
