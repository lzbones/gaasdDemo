#pragma once
/**
 * @file solveQpHildreth.hpp
 * @brief 使用 Hildreth 对偶坐标下降法求解小规模带不等式约束的二次规划
 */

#include "cbfTypes.hpp"

#include <cstddef>

namespace cbf {

/**
 * @brief Hildreth 算法求解 QP：min 0.5 u^T Qd u + c^T u  s.t.  A u <= b
 * @cn_name Hildreth 二次规划
 * @type element
 * @tag_level0 数学运算
 * @tag_level1 优化求解
 * @tag_level2 二次规划
 * @tag_level3 对偶坐标下降
 * @version 1.0
 * @date 2026-06-26
 * @author CBF Module Team
 *
 * 仅针对决策变量维度 2 (a, tan(δf)) 的小规模问题，约束矩阵 A 为 numConstraints×2。
 * Q 假定为对角阵：Qd = diag(qa, qt)。
 *
 * @param[in]  qa             Q 矩阵第 1 个对角元（加速度通道）
 * @param[in]  qt             Q 矩阵第 2 个对角元（转向通道）
 * @param[in]  cVec           线性项 c (size 2)
 * @param[in]  aMatRowMajor   约束矩阵 A，按行优先存储，长度 = 2*numConstraints
 * @param[in]  bVec           约束右端 b，长度 = numConstraints
 * @param[in]  numConstraints 约束行数
 * @param[in]  maxIter        最大迭代步数
 * @param[in]  tolerance      收敛容差
 * @param[out] uOut           输出解 u (size 2)
 * @param[out] iterUsedOut    实际迭代次数
 * @return true 表示找到可行解；false 表示超出迭代仍违反约束
 */
bool solveQpHildreth(Real qa,
                     Real qt,
                     const Real cVec[2],
                     const Real* aMatRowMajor,
                     const Real* bVec,
                     std::size_t numConstraints,
                     std::size_t maxIter,
                     Real tolerance,
                     Real uOut[2],
                     std::size_t& iterUsedOut);

}  // namespace cbf
