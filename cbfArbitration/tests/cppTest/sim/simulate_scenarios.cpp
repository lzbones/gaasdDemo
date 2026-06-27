/**
 * @file simulate_scenarios.cpp
 * @brief CBF 控制修正模块的闭环仿真测试场景
 * @cn_name CBF 闭环仿真
 * @type scenario_test
 * @tag_level0 自动驾驶安全控制
 * @tag_level1 控制屏障函数
 * @tag_level2 闭环仿真
 * @tag_level3 障碍物-only 场景
 * @version 1.0
 * @date 2026-06-26
 * @author CBF Module Team
 *
 * 三个场景：
 *   1) lead_brake     ：前车在 t=2s 处急刹车（纯跟驰）
 *   2) cut_in         ：右侧后方车辆斜向切入
 *   3) cross_pedestrian：行人从右侧匀速横穿
 *
 * 自车采用「公式 (13)」式样的 IDM-like 期望加速度 + 简化预瞄横向控制：
 *   aOriginal     = aIdmExp * (1 - (v / vExp)^4)
 *   deltaFOriginal= 0  （直道上无横向预瞄）
 * 然后调用 cbfControlRevision 修正，使用单车自行车模型 (6) 推进。
 *
 * 输出：每个场景生成 CSV 文件到 tests/cppTest/Integration/<scenario>/<scenario>.csv，
 * 包含逐时刻自车与障碍物状态、原始与安全控制。
 */

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include "cbfControlRevision.hpp"
#include "cbfTypes.hpp"

namespace {

constexpr double DT = 0.05;             ///< 仿真步长 (s)
constexpr double T_END = 8.0;           ///< 总仿真时长 (s)
constexpr double EGO_V_EXP = 12.0;      ///< 自车期望速度 (m/s)
constexpr double EGO_A_IDM_EXP = 2.2;   ///< IDM 期望加速度 (m/s^2)
constexpr double EGO_A_IDM_GAMMA = 4.0; ///< IDM γ 指数

struct GlobalObstacle {
  double x = 0.0;       ///< 全局 x
  double y = 0.0;       ///< 全局 y
  double vx = 0.0;      ///< 全局 vx
  double vy = 0.0;      ///< 全局 vy
  double ax = 0.0;      ///< 全局 ax
  double ay = 0.0;      ///< 全局 ay
};

struct GlobalEgo {
  double x = 0.0;
  double y = 0.0;
  double v = 0.0;
  double phi = 0.0;
};

/// 由全局 obstacle 计算 ego frame 下的 ObstacleState
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

/// 单车运动模型 (6) 一步推进
void integrateEgo(GlobalEgo& ego, double accel, double deltaF, double dt, double L) {
  ego.x += ego.v * std::cos(ego.phi) * dt;
  ego.y += ego.v * std::sin(ego.phi) * dt;
  ego.v += accel * dt;
  if (ego.v < 0.0) {
    ego.v = 0.0;
  }
  ego.phi += (ego.v / L) * std::tan(deltaF) * dt;
}

/// 障碍物简单匀加速模型推进
void integrateObstacle(GlobalObstacle& ob, double dt) {
  ob.x += ob.vx * dt;
  ob.y += ob.vy * dt;
  ob.vx += ob.ax * dt;
  ob.vy += ob.ay * dt;
}

/// IDM 简化期望加速度（公式 (13a)）
double idmExpectedAccel(double v) {
  const double ratio = v / EGO_V_EXP;
  const double accel = EGO_A_IDM_EXP * (1.0 - std::pow(ratio, EGO_A_IDM_GAMMA));
  return accel;
}

cbf_arbitration::CbfParam makeParam() {
  cbf_arbitration::CbfParam p;
  p.safetyRadius = 5.0;
  p.alpha1 = 1.2;
  p.alpha2 = 1.2;
  p.wheelBase = 2.7;
  p.aMin = -7.0;
  p.aMax = 7.0;
  p.deltaFMin = -0.3;
  p.deltaFMax = 0.3;
  p.qDiagAccel = 1.0;
  p.qDiagSteer = 5.0;
  p.qpMaxIter = 300U;
  p.qpTolerance = 1e-7;
  return p;
}

/// 工具：在 step 处对障碍物 i 执行预设事件
using ObstacleEvent = void (*)(double tCurrent, GlobalObstacle& ob);

void eventLeadBrake(double t, GlobalObstacle& ob) {
  if (t >= 2.0 && t < 4.5) {
    ob.ax = -3.5;
  } else {
    ob.ax = 0.0;
  }
}

void eventCutIn(double t, GlobalObstacle& ob) {
  // 右后方车辆向左前切入：t<1 静默，1<=t<3 持续向左 + 加速，t>=3 平稳
  if (t < 1.0) {
    ob.ax = 0.0;
    ob.ay = 0.0;
  } else if (t < 3.0) {
    ob.ax = 1.5;
    ob.ay = 1.0;
  } else {
    ob.ax = 0.0;
    ob.ay = 0.0;
  }
}

void eventCrossPed(double t, GlobalObstacle& ob) {
  (void)t;
  ob.ax = 0.0;
  ob.ay = 0.0;
}

struct ScenarioConfig {
  std::string name;
  GlobalEgo ego;
  std::vector<GlobalObstacle> obs;
  std::vector<ObstacleEvent> events;
};

void runOne(const ScenarioConfig& cfg) {
  const cbf_arbitration::CbfParam param = makeParam();
  const std::string outDir = "tests/cppTest/Integration/" + cfg.name;
  std::string mkdirCmd = "mkdir -p " + outDir;
  std::system(mkdirCmd.c_str());
  std::ofstream csv(outDir + "/" + cfg.name + ".csv");
  csv << "t,ego_x,ego_y,ego_v,ego_phi,a_orig,df_orig,a_safe,df_safe,feasible";
  for (std::size_t i = 0; i < cfg.obs.size(); ++i) {
    csv << ",ob" << i << "_x,ob" << i << "_y,ob" << i << "_dx,ob" << i << "_dy,ob" << i << "_dist";
  }
  csv << "\n";

  GlobalEgo ego = cfg.ego;
  std::vector<GlobalObstacle> obs = cfg.obs;
  const std::size_t numSteps = static_cast<std::size_t>(T_END / DT) + 1U;

  for (std::size_t step = 0; step < numSteps; ++step) {
    const double t = static_cast<double>(step) * DT;
    for (std::size_t i = 0; i < obs.size(); ++i) {
      cfg.events[i](t, obs[i]);
    }

    cbf_arbitration::EgoState egoState;
    egoState.xg = ego.x;
    egoState.yg = ego.y;
    egoState.velocity = ego.v;
    egoState.phi = ego.phi;
    egoState.aOriginal = idmExpectedAccel(ego.v);
    egoState.deltaFOriginal = 0.0;

    std::vector<cbf_arbitration::ObstacleState> ovec;
    ovec.reserve(obs.size());
    for (const auto& ob : obs) {
      ovec.push_back(toEgoFrame(ego, ob));
    }
    const cbf_arbitration::CbfOutput out = cbf_arbitration::cbfControlRevision(
        egoState, ovec.data(), ovec.size(), param);

    csv << t << "," << ego.x << "," << ego.y << "," << ego.v << "," << ego.phi
        << "," << egoState.aOriginal << "," << egoState.deltaFOriginal
        << "," << out.aSafe << "," << out.deltaFSafe << "," << (out.feasible ? 1 : 0);
    for (std::size_t i = 0; i < obs.size(); ++i) {
      const double dx = ovec[i].dxEgo;
      const double dy = ovec[i].dyEgo;
      const double dist = std::sqrt(dx * dx + dy * dy);
      csv << "," << obs[i].x << "," << obs[i].y << "," << dx << "," << dy << "," << dist;
    }
    csv << "\n";

    integrateEgo(ego, out.aSafe, out.deltaFSafe, DT, param.wheelBase);
    for (auto& ob : obs) {
      integrateObstacle(ob, DT);
    }
  }
  std::printf("[Scenario %s] CSV written to %s/%s.csv\n",
              cfg.name.c_str(), outDir.c_str(), cfg.name.c_str());
}

}  // namespace

int main() {
  // 场景 1：lead_brake
  {
    ScenarioConfig cfg;
    cfg.name = "lead_brake";
    cfg.ego.x = 0.0;
    cfg.ego.y = 0.0;
    cfg.ego.v = 12.0;
    cfg.ego.phi = 0.0;
    GlobalObstacle ob;
    ob.x = 35.0;
    ob.y = 0.0;
    ob.vx = 10.0;
    ob.vy = 0.0;
    cfg.obs.push_back(ob);
    cfg.events.push_back(&eventLeadBrake);
    runOne(cfg);
  }
  // 场景 2：cut_in
  {
    ScenarioConfig cfg;
    cfg.name = "cut_in";
    cfg.ego.x = 0.0;
    cfg.ego.y = 0.0;
    cfg.ego.v = 12.0;
    cfg.ego.phi = 0.0;
    GlobalObstacle ob;
    ob.x = 6.0;
    ob.y = -3.5;  // 右后方
    ob.vx = 11.5;
    ob.vy = 0.0;
    cfg.obs.push_back(ob);
    cfg.events.push_back(&eventCutIn);
    runOne(cfg);
  }
  // 场景 3：cross_pedestrian
  {
    ScenarioConfig cfg;
    cfg.name = "cross_pedestrian";
    cfg.ego.x = 0.0;
    cfg.ego.y = 0.0;
    cfg.ego.v = 10.0;
    cfg.ego.phi = 0.0;
    GlobalObstacle ob;
    ob.x = 30.0;
    ob.y = -8.0;
    ob.vx = 0.0;
    ob.vy = 1.5;  // 从右往左横穿（朝 +y 方向走）
    cfg.obs.push_back(ob);
    cfg.events.push_back(&eventCrossPed);
    runOne(cfg);
  }
  return 0;
}
