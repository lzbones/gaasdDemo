#pragma once
/**
 * @file SolveQpHildreth.hpp
 * @brief 二维 QP Hildreth 对偶坐标下降求解器元件
 *
 * 标准形式： min 0.5 u^T Q u + c^T u  s.t.  A u <= b
 *   - Q 为 2x2 对角阵 diag(qDiagA, qDiagT)；
 *   - 约束矩阵 A 行优先存储于 aMatRowMajor，长度 = rowCount * 2；
 *   - 约束右端 bVec 长度 = rowCount。
 */

#include <cstddef>

#include "FuncModule.hpp"

namespace cbf_mbd {

constexpr std::size_t MBD_QP_MAX_ROWS = 64U;

struct SolveQpHildrethTraits {
  struct Input {
    double qDiagA = 1.0;                          ///< Q 加速度通道权重
    double qDiagT = 1.0;                          ///< Q 转向通道权重
    double cVec[2] = {0.0, 0.0};                  ///< 线性项 c
    double aMatRowMajor[MBD_QP_MAX_ROWS * 2] = {0};
    double bVec[MBD_QP_MAX_ROWS] = {0};
    std::size_t rowCount = 0;                     ///< 实际约束行数
  };
  struct Output {
    double uSolution[2] = {0.0, 0.0};
    bool feasible = true;
    std::size_t iterUsed = 0;
  };
  struct Param {
    std::size_t maxIter = 200U;
    double tolerance = 1e-6;
  };
  struct State {};
  struct Sub {};
};

class SolveQpHildreth : public FuncModule<SolveQpHildrethTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

}  // namespace cbf_mbd
