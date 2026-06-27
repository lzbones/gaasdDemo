#include "predictBicycleState.hpp"
#include <cmath>

namespace waypoint_follow {

/**
 * @brief 基于运动学自行车模型预测下一时刻车辆状态
 * @cn_name 自行车模型状态预测
 * @type block
 * @tag_level0 需确认
 * @tag_level1 控制
 * @tag_level2 轨迹跟踪
 * @tag_level3 车辆模型
 * @version 2.0
 * @date 2026-06-26
 * @author 许庆
 *
 * 运动学自行车模型离散方程:
 *   x_{k+1} = x_k + v_k * cos(ψ_k) * Δt
 *   y_{k+1} = y_k + v_k * sin(ψ_k) * Δt
 *   ψ_{k+1} = ψ_k + v_k * tan(δ_k) / L * Δt
 *   v_{k+1} = v_k + a_k * Δt
 * 其中 L 为轴距, δ 为前轮转向角, a 为纵向加速度。
 */
VehicleState predictBicycleState(const VehicleState& state,
                                  const ControlInput& control,
                                  const double dt,
                                  const double wheelbase) {
  const double cosYaw = std::cos(state.yaw);
  const double sinYaw = std::sin(state.yaw);
  const double tanDelta = std::tan(control.steeringAngle);
  const double yawRate = state.v * tanDelta / wheelbase;

  VehicleState nextState;
  nextState.x = state.x + state.v * cosYaw * dt;
  nextState.y = state.y + state.v * sinYaw * dt;
  nextState.yaw = state.yaw + yawRate * dt;
  nextState.v = state.v + control.acceleration * dt;

  return nextState;
}

} // namespace waypoint_follow
