#pragma once

/// @file solveLinearSystem.hpp
/// @brief 高斯消元法线性方程组求解器声明

namespace waypoint_follow {

/// 使用高斯消元法（部分主元选取）求解 n×n 线性方程组 Ax = b
/// @param matrix 行主序矩阵 A (n×maxDim), 求解过程中被修改
/// @param rhs 右端向量 b (n), 求解完成后存放解向量 x
/// @param n 方程组维数
/// @param maxDim 矩阵行步长（第二维大小）
/// @return 求解是否成功 (false 表示矩阵奇异)
bool solveLinearSystem(double* matrix, double* rhs, int n, int maxDim);

} // namespace waypoint_follow
