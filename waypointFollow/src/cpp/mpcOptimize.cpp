#include "mpcOptimize.hpp"
#include "mpcComputeCost.hpp"
#include "clampValue.hpp"
#include <cmath>

namespace waypoint_follow {

/**
 * @brief 使用数值梯度下降法优化MPC控制序列
 * @cn_name MPC梯度下降优化器
 * @type block
 * @tag_level0 需确认
 * @tag_level1 控制
 * @tag_level2 轨迹跟踪
 * @tag_level3 模型预测控制
 * @version 2.0
 * @date 2026-06-26
 * @author 许庆
 *
 * 采用中心差分计算数值梯度, 自适应步长梯度下降法迭代优化。
 * 梯度计算与参数更新严格分离, 每步更新后施加幅值约束和变化率约束。
 * 当代价函数值上升时自动缩减学习率以保证收敛。
 */
MpcOptimizeOutput mpcOptimize(const VehicleState& initialState,
                               const FittedTrajectory& trajectory,
                               const ControlInput* warmStartSequence,
                               const ControlInput& prevAppliedControl,
                               const MpcParam& param) {
  const int nc = param.controlHorizon;
  const double eps = param.finiteDiffEps;

  // 初始化控制序列 (从暖启动序列复制)
  ControlInput controls[MAX_CONTROL_HORIZON] = {};
  for (int i = 0; i < nc; ++i) {
    controls[i] = warmStartSequence[i];
  }

  double currentCost = mpcComputeCost(
    controls, initialState, trajectory, prevAppliedControl, param);

  double adaptiveLr = param.learningRate;

  // 梯度下降迭代
  for (int iter = 0; iter < param.maxOptIterations; ++iter) {
    // 先计算所有梯度 (中心差分), 再统一更新
    double gradDelta[MAX_CONTROL_HORIZON] = {};
    double gradAccel[MAX_CONTROL_HORIZON] = {};

    for (int i = 0; i < nc; ++i) {
      // ---- 转向角梯度 (中心差分) ----
      ControlInput pertPlus[MAX_CONTROL_HORIZON] = {};
      ControlInput pertMinus[MAX_CONTROL_HORIZON] = {};
      for (int j = 0; j < nc; ++j) {
        pertPlus[j] = controls[j];
        pertMinus[j] = controls[j];
      }
      pertPlus[i].steeringAngle += eps;
      pertMinus[i].steeringAngle -= eps;
      const double costDeltaPlus = mpcComputeCost(
        pertPlus, initialState, trajectory, prevAppliedControl, param);
      const double costDeltaMinus = mpcComputeCost(
        pertMinus, initialState, trajectory, prevAppliedControl, param);
      gradDelta[i] = (costDeltaPlus - costDeltaMinus) / (2.0 * eps);

      // ---- 加速度梯度 (中心差分) ----
      for (int j = 0; j < nc; ++j) {
        pertPlus[j] = controls[j];
        pertMinus[j] = controls[j];
      }
      pertPlus[i].acceleration += eps;
      pertMinus[i].acceleration -= eps;
      const double costAccelPlus = mpcComputeCost(
        pertPlus, initialState, trajectory, prevAppliedControl, param);
      const double costAccelMinus = mpcComputeCost(
        pertMinus, initialState, trajectory, prevAppliedControl, param);
      gradAccel[i] = (costAccelPlus - costAccelMinus) / (2.0 * eps);
    }

    // 梯度范数裁剪: 梯度模超过 1 时缩放至单位长度, 否则保留原始梯度
    double gradNormSq = 0.0;
    for (int i = 0; i < nc; ++i) {
      gradNormSq += gradDelta[i] * gradDelta[i];
      gradNormSq += gradAccel[i] * gradAccel[i];
    }
    constexpr double MIN_GRAD_NORM_SQ = 1e-16;
    const double gradNorm = std::sqrt(gradNormSq);
    const double gradScale = (gradNormSq < MIN_GRAD_NORM_SQ) ? 0.0
                           : (gradNorm > 1.0) ? (1.0 / gradNorm)
                           : 1.0;

    // 尝试更新: 保存旧控制序列
    ControlInput candidateControls[MAX_CONTROL_HORIZON] = {};
    for (int i = 0; i < nc; ++i) {
      candidateControls[i].steeringAngle =
        controls[i].steeringAngle - adaptiveLr * gradDelta[i] * gradScale;
      candidateControls[i].acceleration =
        controls[i].acceleration - adaptiveLr * gradAccel[i] * gradScale;

      // 施加幅值约束
      candidateControls[i].steeringAngle = clampValue(
        candidateControls[i].steeringAngle,
        -param.maxSteeringAngle, param.maxSteeringAngle);
      candidateControls[i].acceleration = clampValue(
        candidateControls[i].acceleration,
        param.minAcceleration, param.maxAcceleration);
    }

    // 施加转向角变化率约束
    const double prevDelta0 = prevAppliedControl.steeringAngle;
    candidateControls[0].steeringAngle = clampValue(
      candidateControls[0].steeringAngle,
      prevDelta0 - param.maxSteeringRate,
      prevDelta0 + param.maxSteeringRate);
    for (int i = 1; i < nc; ++i) {
      candidateControls[i].steeringAngle = clampValue(
        candidateControls[i].steeringAngle,
        candidateControls[i - 1].steeringAngle - param.maxSteeringRate,
        candidateControls[i - 1].steeringAngle + param.maxSteeringRate);
    }

    // 评估候选控制序列的代价
    const double candidateCost = mpcComputeCost(
      candidateControls, initialState, trajectory, prevAppliedControl, param);

    // 自适应步长: 代价下降则接受更新, 否则缩减学习率
    const bool costDecreased = (candidateCost < currentCost);
    if (costDecreased) {
      for (int i = 0; i < nc; ++i) {
        controls[i] = candidateControls[i];
      }
      currentCost = candidateCost;
    } else {
      constexpr double LR_DECAY_FACTOR = 0.5;
      adaptiveLr *= LR_DECAY_FACTOR;
    }
  }

  MpcOptimizeOutput output;
  for (int i = 0; i < nc; ++i) {
    output.optimalSequence[i] = controls[i];
  }
  output.finalCost = currentCost;

  return output;
}

} // namespace waypoint_follow
