#include "mpcComputeCost.hpp"
#include "predictBicycleState.hpp"
#include "evaluatePolynomial.hpp"
#include "evaluatePolynomialSlope.hpp"
#include "normalizeAngle.hpp"
#include <cmath>

namespace waypoint_follow {

/**
 * @brief 计算给定控制序列下的MPC总代价
 * @cn_name MPC代价函数
 * @type block
 * @tag_level0 需确认
 * @tag_level1 控制
 * @tag_level2 轨迹跟踪
 * @tag_level3 模型预测控制
 * @version 2.0
 * @date 2026-06-26
 * @author 许庆
 *
 * 代价由三部分构成:
 *   (1) 轨迹跟踪误差: 横向偏差 + 航向偏差 + 速度偏差
 *   (2) 控制量幅值: 转向角 + 加速度
 *   (3) 控制量变化率: 转向角变化率 + 加速度变化率
 */
double mpcComputeCost(const ControlInput* controlSequence,
                      const VehicleState& initialState,
                      const FittedTrajectory& trajectory,
                      const ControlInput& prevAppliedControl,
                      const MpcParam& param) {
  double totalCost = 0.0;
  VehicleState currentState = initialState;

  const int np = param.predictionHorizon;
  const int nc = param.controlHorizon;

  for (int k = 0; k < np; ++k) {
    // 控制步超出控制时域后保持最后一个控制量
    const int controlIdx = (k < nc) ? k : (nc - 1);
    const ControlInput& control = controlSequence[controlIdx];

    // 预测下一时刻状态
    const VehicleState nextState = predictBicycleState(
      currentState, control, param.dt, param.wheelbase);

    // 计算参考轨迹上的目标位置和航向
    const double xNorm = (nextState.x - trajectory.xOffset) / trajectory.xScale;
    const double refY = evaluatePolynomial(trajectory.poly, xNorm);
    const double slopeNorm = evaluatePolynomialSlope(trajectory.poly, xNorm);
    const double refHeading = std::atan2(slopeNorm, trajectory.xScale);

    // 跟踪误差
    const double lateralError = nextState.y - refY;
    const double headingError = normalizeAngle(nextState.yaw - refHeading);
    const double velocityError = nextState.v - trajectory.refSpeed;

    // 跟踪误差代价
    totalCost += param.wLateral * lateralError * lateralError;
    totalCost += param.wHeading * headingError * headingError;
    totalCost += param.wVelocity * velocityError * velocityError;

    // 控制量幅值代价 (仅在控制时域内)
    if (k < nc) {
      totalCost += param.wSteering * control.steeringAngle * control.steeringAngle;
      totalCost += param.wAccel * control.acceleration * control.acceleration;
    }

    // 控制量变化率代价
    if (k < nc) {
      const double prevDelta = (k == 0)
        ? prevAppliedControl.steeringAngle
        : controlSequence[k - 1].steeringAngle;
      const double prevAccel = (k == 0)
        ? prevAppliedControl.acceleration
        : controlSequence[k - 1].acceleration;
      const double dDelta = control.steeringAngle - prevDelta;
      const double dAccel = control.acceleration - prevAccel;
      totalCost += param.wSteeringRate * dDelta * dDelta;
      totalCost += param.wAccelRate * dAccel * dAccel;
    }

    currentState = nextState;
  }

  return totalCost;
}

} // namespace waypoint_follow
