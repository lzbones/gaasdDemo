/**
 * @file waypointFollow_test.cpp
 * @brief 路点跟踪MPC控制器集成测试
 * @cn_name 路点跟踪控制集成测试
 * @type block
 * @tag_level0 需确认
 * @tag_level1 控制
 * @tag_level2 轨迹跟踪
 * @tag_level3 路点跟踪MPC
 * @version 3.0
 * @date 2026-06-26
 * @author 许庆
 *
 * 测试用例覆盖:
 *   TC001: 直线行驶 → 转向角接近零
 *   TC002: 左弯道 → 转向角为正
 *   TC003: 右弯道 → 转向角为负
 *   TC004: 低速保护 → 保持当前转角
 *   TC005: 多步闭环仿真 → 跟踪固定圆弧轨迹, 输出 CSV 供可视化
 */

#include <iostream>
#include <fstream>
#include <cmath>
#include <cstring>
#include "waypointFollow.hpp"

/// 测试用例结构体
struct TestCase {
  const char* id;          ///< 测试用例编号
  const char* description; ///< 测试意图说明
  double egoSpeed;         ///< 自车车速 (m/s)
  double steeringWheel;    ///< 当前方向盘转角 (rad)
  double wpX[6];           ///< 路点X坐标数组 (m)
  double wpY[6];           ///< 路点Y坐标数组 (m)
  bool expectNearZeroSteer;   ///< 是否期望转向角接近零
  bool expectPositiveSteer;   ///< 是否期望正转向角 (左转)
  bool expectNegativeSteer;   ///< 是否期望负转向角 (右转)
  bool expectHoldSteering;    ///< 是否期望保持当前转角 (低速)
  double steerTolerance;      ///< 转向角判定容差 (rad)
};

/// 测试用例数据
static const TestCase g_testCases[] = {
  {
    "TC001_straight",
    "直线行驶: 路点沿X轴等距分布, 期望转向角接近零",
    20.0, 0.0,
    {10.0, 20.0, 30.0, 40.0, 50.0, 60.0},
    {0.0,  0.0,  0.0,  0.0,  0.0,  0.0},
    true, false, false, false, 0.05
  },
  {
    "TC002_left_curve",
    "左弯道: 路点向左偏移, 期望产生正转向角 (左转)",
    15.0, 0.0,
    {7.5, 15.0, 22.5, 30.0, 37.5, 45.0},
    {0.2,  0.8,  1.8,  3.2,  5.0,  7.2},
    false, true, false, false, 0.0
  },
  {
    "TC003_right_curve",
    "右弯道: 路点向右偏移, 期望产生负转向角 (右转)",
    15.0, 0.0,
    {7.5, 15.0, 22.5, 30.0, 37.5, 45.0},
    {-0.2, -0.8, -1.8, -3.2, -5.0, -7.2},
    false, false, true, false, 0.0
  },
  {
    "TC004_low_speed",
    "低速保护: 车速低于阈值, 期望保持当前方向盘转角",
    0.3, 0.1,
    {0.1, 0.2, 0.3, 0.4, 0.5, 0.6},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    false, false, false, true, 0.001
  },
};

/// 执行单个测试用例
bool runTest(const TestCase& tc) {
  using namespace waypoint_follow;

  // 构造输入
  WaypointFollowInput input;
  input.egoSpeed = tc.egoSpeed;
  input.currentSteeringWheelAngle = tc.steeringWheel;
  for (int i = 0; i < NUM_WAYPOINTS; ++i) {
    input.waypoints[i].x = tc.wpX[i];
    input.waypoints[i].y = tc.wpY[i];
  }

  // 使用默认参数
  WaypointFollowParam param;
  WaypointFollowState state;

  // 执行控制器
  const WaypointFollowOutput output = waypointFollow(input, param, state);

  // 判定结果
  bool passed = true;
  const double steerAngle = output.steeringWheelAngle;

  if (tc.expectNearZeroSteer) {
    passed = (std::fabs(steerAngle) < tc.steerTolerance);
  }
  if (tc.expectPositiveSteer) {
    passed = (steerAngle > 0.0);
  }
  if (tc.expectNegativeSteer) {
    passed = (steerAngle < 0.0);
  }
  if (tc.expectHoldSteering) {
    passed = (std::fabs(steerAngle - tc.steeringWheel) < tc.steerTolerance);
  }

  if (!passed) {
    std::cerr << "[FAIL] " << tc.id << ": " << tc.description
              << " | steer=" << steerAngle
              << " accel=" << output.acceleration << std::endl;
  } else {
    std::cout << "[PASS] " << tc.id << ": " << tc.description
              << " | steer=" << steerAngle
              << " accel=" << output.acceleration << std::endl;
  }

  return passed;
}

/// 多步闭环仿真测试: 跟踪固定圆弧参考轨迹, 输出 CSV 供可视化
bool runClosedLoopTest() {
  using namespace waypoint_follow;
  std::cout << "\n--- TC005: Multi-step closed-loop simulation ---" << std::endl;

  WaypointFollowParam param;
  WaypointFollowState state;

  constexpr double RADIUS = 100.0;
  constexpr double SPEED = 10.0;
  constexpr int NUM_STEPS = 80;
  constexpr double DT = 0.1;
  constexpr double OMEGA = SPEED / RADIUS;

  double egoX = 0.0;
  double egoY = 0.0;
  double egoYaw = 0.0;
  double egoV = SPEED;
  double currentSteeringWheel = 0.0;
  double maxLateralError = 0.0;

  std::ofstream csv("waypointFollow_sim.csv");
  csv << "step,time,egoX,egoY,egoYaw,egoV,refX,refY,steer,accel,latErr\n";

  for (int step = 0; step < NUM_STEPS; ++step) {
    const double currentTime = step * DT;

    WaypointFollowInput input;
    input.egoSpeed = egoV;
    input.currentSteeringWheelAngle = currentSteeringWheel;

    for (int i = 0; i < NUM_WAYPOINTS; ++i) {
      const double futureTime = currentTime + 0.5 * (i + 1);
      const double theta = OMEGA * futureTime;
      const double gx = RADIUS * std::sin(theta);
      const double gy = RADIUS * (1.0 - std::cos(theta));
      const double dx = gx - egoX;
      const double dy = gy - egoY;
      input.waypoints[i].x =  std::cos(egoYaw) * dx + std::sin(egoYaw) * dy;
      input.waypoints[i].y = -std::sin(egoYaw) * dx + std::cos(egoYaw) * dy;
    }

    const WaypointFollowOutput output = waypointFollow(input, param, state);
    currentSteeringWheel = output.steeringWheelAngle;
    const double frontWheelAngle = output.frontWheelAngle;

    egoX += egoV * std::cos(egoYaw) * DT;
    egoY += egoV * std::sin(egoYaw) * DT;
    egoYaw += egoV * std::tan(frontWheelAngle) / param.mpcParam.wheelbase * DT;
    egoV += output.acceleration * DT;

    const double refTheta = OMEGA * (step + 1) * DT;
    const double refX = RADIUS * std::sin(refTheta);
    const double refY = RADIUS * (1.0 - std::cos(refTheta));
    const double distToCenter = std::sqrt(egoX * egoX + (egoY - RADIUS) * (egoY - RADIUS));
    const double lateralError = std::fabs(distToCenter - RADIUS);
    maxLateralError = (lateralError > maxLateralError) ? lateralError : maxLateralError;

    csv << step << "," << currentTime << "," << egoX << "," << egoY << ","
        << egoYaw << "," << egoV << "," << refX << "," << refY << ","
        << output.steeringWheelAngle << "," << output.acceleration << ","
        << lateralError << "\n";
  }
  csv.close();

  constexpr double THRESHOLD = 2.0;
  const bool passed = (maxLateralError < THRESHOLD);
  if (passed) {
    std::cout << "[PASS] TC005_closed_loop: max_lateral_error="
              << maxLateralError << "m (threshold=" << THRESHOLD << "m)" << std::endl;
  } else {
    std::cerr << "[FAIL] TC005_closed_loop: max_lateral_error="
              << maxLateralError << "m (threshold=" << THRESHOLD << "m)" << std::endl;
  }

  return passed;
}

int main() {
  unsigned int passed = 0;
  constexpr unsigned int numCases = sizeof(g_testCases) / sizeof(g_testCases[0]);

  std::cout << "=== WaypointFollow Unit/Integration Tests ===" << std::endl;

  // 运行参数化测试用例
  for (const auto& tc : g_testCases) {
    const bool ok = runTest(tc);
    passed += ok ? 1 : 0;
  }

  // 运行闭环仿真测试
  const bool closedLoopOk = runClosedLoopTest();
  const unsigned int totalTests = numCases + 1;
  passed += closedLoopOk ? 1 : 0;

  std::cout << "\n=== Test Summary ===" << std::endl;
  std::cout << "Passed: " << passed << "/" << totalTests << std::endl;

  const int exitCode = (passed == totalTests) ? 0 : 1;
  return exitCode;
}
