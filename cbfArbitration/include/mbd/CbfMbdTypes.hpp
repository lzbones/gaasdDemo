#pragma once
/**
 * @file CbfMbdTypes.hpp
 * @brief MBD 顶层模块公用快照结构（自车与障碍物输入快照）
 *
 * 命名为 *Snapshot 以强调"周期性输入快照"语义；纯结构体，无逻辑。
 */

#include <cstddef>

namespace cbf_arbitration_mbd {

constexpr std::size_t MAX_MBD_OBSTACLES = 32U;

struct EgoSnapshot {
  double xg = 0.0;
  double yg = 0.0;
  double velocity = 0.0;
  double phi = 0.0;
  double aOriginal = 0.0;
  double deltaFOriginal = 0.0;
};

struct ObstacleSnapshot {
  double dxEgo = 0.0;
  double dyEgo = 0.0;
  double vRxEgo = 0.0;
  double vRyEgo = 0.0;
  double axGlobal = 0.0;
  double ayGlobal = 0.0;
};

}  // namespace cbf_arbitration_mbd
