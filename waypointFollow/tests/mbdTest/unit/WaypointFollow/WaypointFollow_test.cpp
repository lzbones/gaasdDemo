/**
 * @file WaypointFollow_test.cpp
 * @brief WaypointFollow MBD 组件集成测试 (含闭环仿真)
 *
 * 与 Step 01 cppTest 用例完全对齐, 验证 MBD 重构后的正确性
 */
#include <iostream>
#include <fstream>
#include <cmath>
#include <utility>
#include "WaypointFollow.hpp"

using namespace waypoint_follow_mbd;
using namespace waypoint_follow;

static WaypointFollow buildController() {
  PredictBicycleState pbs(PredictBicycleStateTraits::Param{0.1, 2.85});
  EvaluatePolynomial ep;
  EvaluatePolynomialSlope es;
  NormalizeAngle na;
  ClampValue cv1;
  FitCubicPolynomial fp;
  ClampValue cv2;

  MpcComputeCost mcc(
    MpcComputeCostTraits::Param{MpcParam{}},
    MpcComputeCostTraits::Sub{std::move(pbs), std::move(ep), std::move(es), std::move(na)}
  );

  MpcOptimize mpcOpt(
    MpcOptimizeTraits::Param{MpcParam{}},
    MpcOptimizeTraits::Sub{std::move(mcc), std::move(cv1)}
  );

  FitWaypointTrajectory fit(
    FitWaypointTrajectoryTraits::Param{},
    FitWaypointTrajectoryTraits::Sub{std::move(fp)}
  );

  ClampValue cvTop;

  WaypointFollow ctrl(
    WaypointFollowTraits::Param{MpcParam{}, 15.0},
    WaypointFollowTraits::Sub{std::move(fit), std::move(mpcOpt), std::move(cvTop)}
  );
  return ctrl;
}

static bool testStraight() {
  auto ctrl = buildController();
  WaypointFollowTraits::Input in;
  in.egoSpeed = 10.0;
  in.currentSteeringWheelAngle = 0.0;
  for (int i = 0; i < NUM_WAYPOINTS; ++i) {
    in.waypoints[i].x = 5.0 * (i + 1);
    in.waypoints[i].y = 0.0;
  }
  WaypointFollowTraits::Output out;
  ctrl.run(in, out);
  const bool ok = std::fabs(out.steeringWheelAngle) < 0.5;
  std::cout << (ok ? "[PASS]" : "[FAIL]") << " WF_TC001 straight: steer=" << out.steeringWheelAngle << std::endl;
  return ok;
}

static bool testLeftCurve() {
  auto ctrl = buildController();
  WaypointFollowTraits::Input in;
  in.egoSpeed = 10.0;
  in.currentSteeringWheelAngle = 0.0;
  const double xs[] = {5,10,15,20,25,30};
  const double ys[] = {0.5,2.0,4.5,8.0,12.5,18.0};
  for (int i = 0; i < NUM_WAYPOINTS; ++i) { in.waypoints[i].x = xs[i]; in.waypoints[i].y = ys[i]; }
  WaypointFollowTraits::Output out;
  ctrl.run(in, out);
  const bool ok = out.steeringWheelAngle > 0.0;
  std::cout << (ok ? "[PASS]" : "[FAIL]") << " WF_TC002 left: steer=" << out.steeringWheelAngle << std::endl;
  return ok;
}

static bool testLowSpeed() {
  auto ctrl = buildController();
  WaypointFollowTraits::Input in;
  in.egoSpeed = 0.3;
  in.currentSteeringWheelAngle = 0.1;
  for (int i = 0; i < NUM_WAYPOINTS; ++i) {
    in.waypoints[i].x = 5.0 * (i + 1);
    in.waypoints[i].y = 0.0;
  }
  WaypointFollowTraits::Output out;
  ctrl.run(in, out);
  const bool ok = (std::fabs(out.steeringWheelAngle - 0.1) < 1e-6) && (std::fabs(out.acceleration) < 1e-6);
  std::cout << (ok ? "[PASS]" : "[FAIL]") << " WF_TC003 low_speed: steer=" << out.steeringWheelAngle
            << " accel=" << out.acceleration << std::endl;
  return ok;
}

static bool testClosedLoop() {
  auto ctrl = buildController();
  constexpr double R = 100.0;
  constexpr double speed = 10.0;
  constexpr int STEPS = 80;
  constexpr double DT = 0.1;
  constexpr double omega = speed / R;

  double egoX = 0.0, egoY = 0.0, egoYaw = 0.0, egoV = speed;
  double currentSteerWheel = 0.0;

  std::ofstream csv("WaypointFollow_mbd_sim.csv");
  csv << "step,egoX,egoY,egoYaw,egoV,refX,refY,steer,accel,latErr\n";

  double maxLatErr = 0.0;
  for (int step = 0; step < STEPS; ++step) {
    const double currentTime = step * DT;

    WaypointFollowTraits::Input in;
    in.egoSpeed = egoV;
    in.currentSteeringWheelAngle = currentSteerWheel;

    for (int i = 0; i < NUM_WAYPOINTS; ++i) {
      const double futureTime = currentTime + 0.5 * (i + 1);
      const double theta = omega * futureTime;
      const double gx = R * std::sin(theta);
      const double gy = R * (1.0 - std::cos(theta));
      const double dx = gx - egoX;
      const double dy = gy - egoY;
      in.waypoints[i].x =  std::cos(egoYaw) * dx + std::sin(egoYaw) * dy;
      in.waypoints[i].y = -std::sin(egoYaw) * dx + std::cos(egoYaw) * dy;
    }

    WaypointFollowTraits::Output out;
    ctrl.run(in, out);
    currentSteerWheel = out.steeringWheelAngle;

    const double frontAngle = out.steeringWheelAngle / 15.0;
    egoX += egoV * std::cos(egoYaw) * DT;
    egoY += egoV * std::sin(egoYaw) * DT;
    egoYaw += egoV * std::tan(frontAngle) / 2.85 * DT;
    egoV += out.acceleration * DT;

    const double refX = R * std::sin(omega * (step + 1) * DT);
    const double refY = R * (1.0 - std::cos(omega * (step + 1) * DT));
    const double distToCenter = std::sqrt(egoX * egoX + (egoY - R) * (egoY - R));
    const double le = std::fabs(distToCenter - R);
    maxLatErr = (le > maxLatErr) ? le : maxLatErr;
    csv << step << "," << egoX << "," << egoY << "," << egoYaw << ","
        << egoV << "," << refX << "," << refY << ","
        << out.steeringWheelAngle << "," << out.acceleration << "," << le << "\n";
  }
  csv.close();

  constexpr double THRESHOLD = 2.0;
  const bool ok = maxLatErr < THRESHOLD;
  std::cout << (ok ? "[PASS]" : "[FAIL]") << " WF_TC004 closed_loop: maxLatErr=" << maxLatErr
            << "m (threshold=" << THRESHOLD << "m)" << std::endl;
  return ok;
}

int main() {
  unsigned int passed = 0;
  std::cout << "=== MBD Integration Tests: WaypointFollow ===" << std::endl;
  if (testStraight()) ++passed;
  if (testLeftCurve()) ++passed;
  if (testLowSpeed()) ++passed;
  if (testClosedLoop()) ++passed;
  std::cout << "\nPassed: " << passed << "/4" << std::endl;
  return (passed == 4) ? 0 : 1;
}
