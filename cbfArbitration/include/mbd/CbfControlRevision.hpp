#pragma once
/**
 * @file CbfControlRevision.hpp
 * @brief CBF 安全控制修正模块顶层（复合模块）
 *
 * Sub 中包含四个模块：
 *   - RotateVectorByMinusYaw      （元件，全局加速度 → ego frame）
 *   - ComputeObstacleAffineConstraint （组件，单条 HOCBF 约束装配）
 *   - SolveQpHildreth            （元件，QP 求解器）
 *   - ClampControlInput           （元件，控制输入饱和限幅）
 *
 * 输入：自车快照 + 障碍物数组；输出：安全控制 (a*, δ_f*) 及诊断量。
 */

#include "CbfMbdTypes.hpp"
#include "ClampControlInput.hpp"
#include "ComputeObstacleAffineConstraint.hpp"
#include "FuncModule.hpp"
#include "RotateVectorByMinusYaw.hpp"
#include "SolveQpHildreth.hpp"

namespace cbf_arbitration_mbd {

struct CbfControlRevisionTraits {
  struct Input {
    EgoSnapshot egoState{};
    ObstacleSnapshot obstacles[MAX_MBD_OBSTACLES] = {};
    std::size_t obstacleNum = 0;
  };
  struct Output {
    double aSafe = 0.0;
    double deltaFSafe = 0.0;
    bool feasible = true;
    std::size_t activeNum = 0;
    std::size_t iterUsed = 0;
  };
  struct Param {
    double safetyRadius = 5.0;
    double alpha1 = 1.2;
    double alpha2 = 1.2;
    double wheelBase = 2.7;
    double aMin = -7.0;
    double aMax = 7.0;
    double deltaFMin = -0.5;
    double deltaFMax = 0.5;
    double qDiagAccel = 1.0;
    double qDiagSteer = 5.0;
    std::size_t qpMaxIter = 200U;
    double qpTolerance = 1e-6;
  };
  struct State {};
  struct Sub {
    RotateVectorByMinusYaw rotator{};
    ComputeObstacleAffineConstraint constraintGen{};
    SolveQpHildreth qpSolver{};
    ClampControlInput clamper{};
  };
};

class CbfControlRevision : public FuncModule<CbfControlRevisionTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

}  // namespace cbf_arbitration_mbd
