/**
 * @file integration_sim.cpp
 * @brief waypointFollow + cbfArbitration 联合集成仿真
 *
 * 在四个典型交通场景下，将端到端输出的 6 个未来路点喂给 waypointFollow，
 * 得到原始控制 (steeringWheelAngle, acceleration) 后，换算为前轮转角作为
 * cbfArbitration 的原始输入；cbfArbitration 结合感知障碍物信息输出安全控制 (aSafe, deltaFSafe)。
 * 最终使用单车自行车模型推进自车与障碍物状态，逐周期记录 CSV。
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "cbfControlRevision.hpp"
#include "cbfTypes.hpp"
#include "waypointFollow.hpp"
#include "waypointFollowTypes.hpp"

namespace {

// ---------------------------------------------------------------------------
// 仿真常量
// ---------------------------------------------------------------------------
constexpr double DT = 0.05;          ///< 仿真步长 (s)
constexpr double T_END = 8.0;        ///< 总仿真时长 (s)
constexpr std::size_t NUM_STEPS =
    static_cast<std::size_t>(T_END / DT) + 1U;

// ---------------------------------------------------------------------------
// 全局坐标系下的自车与障碍物状态
// ---------------------------------------------------------------------------
struct GlobalEgo {
  double x = 0.0;    ///< 全局 x (m)
  double y = 0.0;    ///< 全局 y (m)
  double v = 0.0;    ///< 纵向速度 (m/s)
  double phi = 0.0;  ///< 航向角 (rad)
};

struct GlobalObstacle {
  double x = 0.0;   ///< 全局 x (m)
  double y = 0.0;   ///< 全局 y (m)
  double vx = 0.0;  ///< 全局 vx (m/s)
  double vy = 0.0;  ///< 全局 vy (m/s)
  double ax = 0.0;  ///< 全局 ax (m/s^2)
  double ay = 0.0;  ///< 全局 ay (m/s^2)
};

// ---------------------------------------------------------------------------
// 工具函数
// ---------------------------------------------------------------------------

/// 将障碍物全局状态转换为 cbfArbitration 要求的 ego frame 状态
cbf_arbitration::ObstacleState toEgoFrame(const GlobalEgo& ego, const GlobalObstacle& ob) {
  const double dxGlobal = ob.x - ego.x;
  const double dyGlobal = ob.y - ego.y;
  const double cosPhi = std::cos(ego.phi);
  const double sinPhi = std::sin(ego.phi);

  cbf_arbitration::ObstacleState s;
  s.dxEgo = cosPhi * dxGlobal + sinPhi * dyGlobal;
  s.dyEgo = -sinPhi * dxGlobal + cosPhi * dyGlobal;

  const double vRxGlobal = ob.vx - ego.v * cosPhi;
  const double vRyGlobal = ob.vy - ego.v * sinPhi;
  s.vRxEgo = cosPhi * vRxGlobal + sinPhi * vRyGlobal;
  s.vRyEgo = -sinPhi * vRxGlobal + cosPhi * vRyGlobal;

  s.axGlobal = ob.ax;
  s.ayGlobal = ob.ay;
  return s;
}

/// 单车自行车模型前向积分（输入为前轮转角 delta_f）
void integrateEgo(GlobalEgo& ego,
                  double accel,
                  double deltaF,
                  double dt,
                  double wheelbase) {
  ego.x += ego.v * std::cos(ego.phi) * dt;
  ego.y += ego.v * std::sin(ego.phi) * dt;
  ego.v += accel * dt;
  if (ego.v < 0.0) {
    ego.v = 0.0;
  }
  ego.phi += (ego.v / wheelbase) * std::tan(deltaF) * dt;
}

/// 障碍物匀加速运动模型前向积分
void integrateObstacle(GlobalObstacle& ob, double dt) {
  ob.x += ob.vx * dt;
  ob.y += ob.vy * dt;
  ob.vx += ob.ax * dt;
  ob.vy += ob.ay * dt;
}

/// 生成沿直道的 6 个参考路点（自车坐标系）
void generateStraightWaypoints(waypoint_follow::WaypointFollowInput& input,
                               double egoSpeed) {
  constexpr double TIME_STEP = 0.5;  // 路点时间间隔：0.5 ~ 3.0 s
  for (int i = 0; i < waypoint_follow::NUM_WAYPOINTS; ++i) {
    const double futureTime = TIME_STEP * (i + 1);
    input.waypoints[i].x = egoSpeed * futureTime;
    input.waypoints[i].y = 0.0;
  }
}

/// 生成带横向偏移的直道参考路点，用于诱导 gentle 转向
void generateOffsetWaypoints(waypoint_follow::WaypointFollowInput& input,
                             double egoSpeed,
                             double lateralOffsetAt3s) {
  constexpr double TIME_STEP = 0.5;
  for (int i = 0; i < waypoint_follow::NUM_WAYPOINTS; ++i) {
    const double futureTime = TIME_STEP * (i + 1);
    const double ratio = futureTime / 3.0;
    input.waypoints[i].x = egoSpeed * futureTime;
    input.waypoints[i].y = lateralOffsetAt3s * ratio * ratio;
  }
}

/// 构造 CBF 参数，保持与 waypointFollow 车辆参数一致
/// @param wheelbase 车辆轴距
/// @param qDiagSteer 转向权重，较小的值鼓励 CBF 使用转向规避
/// @note 默认使用较大转向权重使 CBF 优先通过纵向减速保持安全
cbf_arbitration::CbfParam makeCbfParam(double wheelbase, double qDiagSteer = 200.0) {
  cbf_arbitration::CbfParam p;
  p.safetyRadius = 5.0;
  p.alpha1 = 1.2;
  p.alpha2 = 1.2;
  p.wheelBase = wheelbase;
  p.aMin = -7.0;
  p.aMax = 7.0;
  p.deltaFMin = -0.3;
  p.deltaFMax = 0.3;
  p.qDiagAccel = 1.0;
  p.qDiagSteer = qDiagSteer;
  p.qpMaxIter = 300U;
  p.qpTolerance = 1e-7;
  return p;
}

/// 工具：对障碍物施加场景相关事件
using ObstacleEvent = void (*)(double tCurrent, GlobalObstacle& ob);

void eventLeadBrake(double t, GlobalObstacle& ob) {
  if (t >= 2.0 && t < 4.5) {
    ob.ax = -4.0;
  } else {
    ob.ax = 0.0;
  }
}

void eventCutIn(double t, GlobalObstacle& ob) {
  if (t < 1.0) {
    ob.ax = 0.0;
    ob.ay = 0.0;
  } else if (t < 3.0) {
    ob.ax = 1.5;
    ob.ay = 1.0;  // 向左切入（+y 方向）
  } else {
    ob.ax = 0.0;
    ob.ay = 0.0;
  }
}

void eventCrossPedestrian(double /*t*/, GlobalObstacle& ob) {
  ob.ax = 0.0;
  ob.ay = 0.0;
}

// ---------------------------------------------------------------------------
// 场景配置
// ---------------------------------------------------------------------------
struct ScenarioConfig {
  std::string name;
  std::string description;
  GlobalEgo ego;
  std::vector<GlobalObstacle> obstacles;
  std::vector<ObstacleEvent> events;
  double lateralOffsetAt3s = 0.0;  // 路点诱导横向偏移，0 表示直道
  double qDiagSteer = 200.0;       // CBF 转向权重，小值鼓励转向规避
};

ScenarioConfig makeLeadBrakeScenario() {
  ScenarioConfig cfg;
  cfg.name = "lead_brake";
  cfg.description = "前车急刹：同车道前方车辆 t=2s 开始以 -4 m/s^2 制动";
  cfg.ego = {0.0, 0.0, 12.0, 0.0};

  GlobalObstacle lead;
  lead.x = 40.0;
  lead.y = 0.0;
  lead.vx = 12.0;
  lead.vy = 0.0;
  cfg.obstacles.push_back(lead);
  cfg.events.push_back(&eventLeadBrake);
  cfg.lateralOffsetAt3s = 0.0;
  return cfg;
}

ScenarioConfig makeCutInScenario() {
  ScenarioConfig cfg;
  cfg.name = "cut_in";
  cfg.description = "右侧车辆切入：右后方车辆 t=1~3s 向左切入，自车更快将接近";
  cfg.ego = {0.0, 0.0, 12.0, 0.0};

  GlobalObstacle cutter;
  cutter.x = 7.0;    // 保持一定初始距离，给 CBF 反应空间
  cutter.y = -2.5;   // 靠近本车道
  cutter.vx = 10.0;  // 比自车慢，便于形成碰撞风险
  cutter.vy = 0.0;
  cfg.obstacles.push_back(cutter);
  cfg.events.push_back(&eventCutIn);
  cfg.lateralOffsetAt3s = 0.0;
  return cfg;
}

ScenarioConfig makeCrossPedestrianScenario() {
  ScenarioConfig cfg;
  cfg.name = "cross_pedestrian";
  cfg.description = "行人横穿：行人从右侧匀速横穿车道，CBF 应通过减速让行";
  cfg.ego = {0.0, 0.0, 10.0, 0.0};

  GlobalObstacle ped;
  ped.x = 35.0;   // 足够远的行人，使 CBF 可通过减速保持安全
  ped.y = -4.0;   // 靠近车道边缘
  ped.vx = 0.0;
  ped.vy = 1.5;   // 从右向左
  cfg.obstacles.push_back(ped);
  cfg.events.push_back(&eventCrossPedestrian);
  cfg.lateralOffsetAt3s = 0.0;
  return cfg;
}

ScenarioConfig makeSwerveScenario() {
  ScenarioConfig cfg;
  cfg.name = "swerve_obstacle";
  cfg.description = "转向规避：车道中心偏右静止障碍物，CBF 以转向为主绕行";
  cfg.ego = {0.0, 0.0, 15.0, 0.0};

  GlobalObstacle obs;
  obs.x = 35.0;  // 中距静止障碍物，给 CBF 足够时间转向
  obs.y = 0.5;   // 稍微偏向车道中心右侧，鼓励 CBF 向左绕行
  obs.vx = 0.0;
  obs.vy = 0.0;
  cfg.obstacles.push_back(obs);
  cfg.events.push_back(&eventCrossPedestrian);  // 静止，无需动态事件
  cfg.lateralOffsetAt3s = 0.0;
  cfg.qDiagSteer = 10.0;  // 降低转向权重，鼓励 CBF 使用转向规避，同时避免过度偏离
  return cfg;
}

// ---------------------------------------------------------------------------
// 单个场景闭环运行
// ---------------------------------------------------------------------------
void runScenario(const ScenarioConfig& cfg) {
  using namespace waypoint_follow;

  const std::string outDir = "output";
  std::string mkdirCmd = "mkdir -p " + outDir;
  std::system(mkdirCmd.c_str());

  const std::string csvPath = outDir + "/" + cfg.name + ".csv";
  std::ofstream csv(csvPath);
  if (!csv.is_open()) {
    std::cerr << "[ERROR] Failed to open " << csvPath << std::endl;
    return;
  }

  csv << "t,ego_x,ego_y,ego_v,ego_phi,"
      << "sw_original,delta_f_original,accel_original,"
      << "a_safe,delta_f_safe,feasible,"
      << "min_dist";
  for (std::size_t i = 0; i < cfg.obstacles.size(); ++i) {
    csv << ",ob" << i << "_x,ob" << i << "_y,ob" << i << "_vx,ob" << i << "_vy";
  }
  csv << "\n";

  // 车辆与 waypointFollow 参数
  WaypointFollowParam wfParam;
  WaypointFollowState wfState;
  const double wheelbase = wfParam.mpcParam.wheelbase;
  const double steeringRatio = wfParam.steeringRatio;

  // CBF 参数（每个场景可独立配置转向权重）
  const cbf_arbitration::CbfParam cbfParam = makeCbfParam(wheelbase, cfg.qDiagSteer);

  GlobalEgo ego = cfg.ego;
  std::vector<GlobalObstacle> obs = cfg.obstacles;

  double currentSteeringWheelAngle = 0.0;  // 初始方向盘转角

  for (std::size_t step = 0; step < NUM_STEPS; ++step) {
    const double t = static_cast<double>(step) * DT;

    // 1) 触发场景事件，更新障碍物加速度
    for (std::size_t i = 0; i < obs.size(); ++i) {
      cfg.events[i](t, obs[i]);
    }

    // 2) 生成端到端输出的 6 个未来路点
    WaypointFollowInput wfInput;
    wfInput.egoSpeed = ego.v;
    wfInput.currentSteeringWheelAngle = currentSteeringWheelAngle;
    if (cfg.lateralOffsetAt3s == 0.0) {
      generateStraightWaypoints(wfInput, ego.v);
    } else {
      generateOffsetWaypoints(wfInput, ego.v, cfg.lateralOffsetAt3s);
    }

    // 3) waypointFollow：路点 → 原始控制
    const WaypointFollowOutput wfOutput =
        waypointFollow(wfInput, wfParam, wfState);

    // 4) waypointFollow 已直接输出前轮转角，作为 cbfArbitration 原始输入
    const double deltaFOriginal = wfOutput.frontWheelAngle;

    // 5) 构造 cbfArbitration 自车状态
    cbf_arbitration::EgoState egoState;
    egoState.xg = ego.x;
    egoState.yg = ego.y;
    egoState.velocity = ego.v;
    egoState.phi = ego.phi;
    egoState.aOriginal = wfOutput.acceleration;
    egoState.deltaFOriginal = deltaFOriginal;

    // 6) 障碍物状态转换到 ego frame
    std::vector<cbf_arbitration::ObstacleState> obstacleStates;
    obstacleStates.reserve(obs.size());
    for (const auto& ob : obs) {
      obstacleStates.push_back(toEgoFrame(ego, ob));
    }

    // 7) cbfArbitration 安全仲裁
    const cbf_arbitration::CbfOutput cbfOutput = cbf_arbitration::cbfControlRevision(
        egoState, obstacleStates.data(), obstacleStates.size(), cbfParam);

    // 8) 计算最小障碍物距离
    double minDist = std::numeric_limits<double>::max();
    for (const auto& s : obstacleStates) {
      const double dist = std::sqrt(s.dxEgo * s.dxEgo + s.dyEgo * s.dyEgo);
      if (dist < minDist) {
        minDist = dist;
      }
    }

    // 9) 记录 CSV
    csv << t << "," << ego.x << "," << ego.y << "," << ego.v << ","
        << ego.phi << "," << wfOutput.steeringWheelAngle << ","
        << deltaFOriginal << ","
        << wfOutput.acceleration << "," << cbfOutput.aSafe << ","
        << cbfOutput.deltaFSafe << "," << (cbfOutput.feasible ? 1 : 0)
        << "," << minDist;
    for (const auto& ob : obs) {
      csv << "," << ob.x << "," << ob.y << "," << ob.vx << "," << ob.vy;
    }
    csv << "\n";

    // 10) 使用安全控制推进自车与障碍物
    integrateEgo(ego, cbfOutput.aSafe, cbfOutput.deltaFSafe, DT, wheelbase);
    for (auto& ob : obs) {
      integrateObstacle(ob, DT);
    }

    // 11) 更新下一周期方向盘转角（用于 waypointFollow 的当前方向盘输入）
    currentSteeringWheelAngle = cbfOutput.deltaFSafe * steeringRatio;
    (void)steeringRatio;  // 其余地方已改用 frontWheelAngle
  }

  csv.close();
  std::printf("[Scenario %s] %s\n", cfg.name.c_str(), cfg.description.c_str());
  std::printf("  CSV written to %s\n", csvPath.c_str());
}

}  // namespace

int main() {
  std::cout << "=== waypointFollow + cbfArbitration Integration Simulation ===" << std::endl;

  runScenario(makeLeadBrakeScenario());
  runScenario(makeCutInScenario());
  runScenario(makeCrossPedestrianScenario());
  runScenario(makeSwerveScenario());

  std::cout << "=== All scenarios completed ===" << std::endl;
  return 0;
}
