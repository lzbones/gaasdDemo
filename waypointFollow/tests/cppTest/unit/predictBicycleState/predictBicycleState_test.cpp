/**
 * @file predictBicycleState_test.cpp
 * @brief 运动学自行车模型单步预测单元测试
 * @cn_name 自行车模型预测测试
 * @type block
 * @tag_level0 需确认
 * @tag_level1 控制
 * @tag_level2 轨迹跟踪
 * @tag_level3 车辆模型
 * @version 2.0
 * @date 2026-06-26
 * @author 许庆
 */

#include <iostream>
#include <cmath>
#include "predictBicycleState.hpp"

/// 测试用例结构体
struct BicycleTestCase {
  const char* id;          ///< 测试编号
  const char* description; ///< 测试意图
  double x0, y0, yaw0, v0; ///< 初始状态
  double delta, accel;     ///< 控制输入
  double dt, wheelbase;    ///< 时间步长和轴距
  double expX, expY, expYaw, expV; ///< 期望状态
  double tolerance;        ///< 容差
};

static const BicycleTestCase g_bicycleTestCases[] = {
  {
    "BM_TC001",
    "直线匀速: v=10m/s, delta=0, a=0, dt=0.1s → x前进1m",
    0.0, 0.0, 0.0, 10.0,
    0.0, 0.0,
    0.1, 2.85,
    1.0, 0.0, 0.0, 10.0,
    1e-6
  },
  {
    "BM_TC002",
    "直线加速: v=10m/s, a=2m/s^2, dt=0.1s → v增至10.2m/s",
    0.0, 0.0, 0.0, 10.0,
    0.0, 2.0,
    0.1, 2.85,
    1.0, 0.0, 0.0, 10.2,
    1e-6
  },
  {
    "BM_TC003",
    "左转弯: v=10m/s, delta=0.1rad → yaw增加",
    0.0, 0.0, 0.0, 10.0,
    0.1, 0.0,
    0.1, 2.85,
    0.0, 0.0, 0.0, 10.0,
    0.0
  },
  {
    "BM_TC004",
    "静止状态: v=0 → 位置和航向不变",
    5.0, 3.0, 0.5, 0.0,
    0.2, 0.0,
    0.1, 2.85,
    5.0, 3.0, 0.5, 0.0,
    1e-10
  },
};

bool runBicycleTest(const BicycleTestCase& tc) {
  using namespace waypoint_follow;

  VehicleState state;
  state.x = tc.x0;
  state.y = tc.y0;
  state.yaw = tc.yaw0;
  state.v = tc.v0;

  ControlInput control;
  control.steeringAngle = tc.delta;
  control.acceleration = tc.accel;

  const VehicleState next = predictBicycleState(state, control, tc.dt, tc.wheelbase);

  // TC003 特殊判定: 左转弯时 yaw 应增加
  bool passed = true;
  if (tc.tolerance == 0.0) {
    // 仅检查方向性: 左转时yaw增加
    const double yawChange = next.yaw - tc.yaw0;
    passed = (yawChange > 0.0);
    const double expectedX = tc.x0 + tc.v0 * std::cos(tc.yaw0) * tc.dt;
    const double expectedY = tc.y0 + tc.v0 * std::sin(tc.yaw0) * tc.dt;
    passed = passed && (std::fabs(next.x - expectedX) < 1e-6);
    passed = passed && (std::fabs(next.y - expectedY) < 1e-6);
  } else {
    passed = (std::fabs(next.x - tc.expX) < tc.tolerance)
          && (std::fabs(next.y - tc.expY) < tc.tolerance)
          && (std::fabs(next.yaw - tc.expYaw) < tc.tolerance)
          && (std::fabs(next.v - tc.expV) < tc.tolerance);
  }

  if (passed) {
    std::cout << "[PASS] " << tc.id << ": " << tc.description
              << " | x=" << next.x << " y=" << next.y
              << " yaw=" << next.yaw << " v=" << next.v << std::endl;
  } else {
    std::cerr << "[FAIL] " << tc.id << ": " << tc.description
              << " | x=" << next.x << " y=" << next.y
              << " yaw=" << next.yaw << " v=" << next.v << std::endl;
  }

  return passed;
}

int main() {
  unsigned int passed = 0;
  constexpr unsigned int total = sizeof(g_bicycleTestCases) / sizeof(g_bicycleTestCases[0]);

  std::cout << "=== PredictBicycleState Unit Tests ===" << std::endl;

  for (const auto& tc : g_bicycleTestCases) {
    const bool ok = runBicycleTest(tc);
    passed += ok ? 1 : 0;
  }

  std::cout << "\n=== Test Summary ===" << std::endl;
  std::cout << "Passed: " << passed << "/" << total << std::endl;

  const int exitCode = (passed == total) ? 0 : 1;
  return exitCode;
}
