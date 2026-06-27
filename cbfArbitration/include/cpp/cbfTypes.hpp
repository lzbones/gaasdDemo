#pragma once
/**
 * @file cbfTypes.hpp
 * @brief CBF 模块通用数据结构、参数与常量定义
 *
 * 本文件用于定义 CBF (Control Barrier Function) 自动驾驶安全控制修正模块的
 * 输入、输出、参数与状态结构体；不包含任何函数实现。
 *
 * 算法参考：
 *   "A Control Barrier Function-Based Driving Safety Control Revision Method for
 *    Autonomous Vehicles Plus Applicability Analysis", IEEE TIE.
 *   Section II (Preliminaries) 与 Section III-A (Constraints for Traffic Participants).
 */

#include <array>
#include <cstddef>
#include <vector>

namespace cbf_arbitration {

/// 模块允许的最大障碍物数量上限（用于固定容量缓存，避免动态分配）
constexpr std::size_t MAX_OBSTACLE_NUM = 32U;

/// 浮点类型别名（统一精度，便于将来替换）
using Real = double;

/**
 * @brief 自车（EV）当前运动状态与原始控制指令
 *
 * 字段语义：
 *   - 自车位置坐标系：全局笛卡尔坐标系 (xg, yg)；
 *   - 自车朝向 phi：以全局 +x 轴为基准的偏航角 (rad)；
 *   - 原始控制指令 (aOriginal, deltaFOriginal) 为上层控制器输出的「期望」加速度
 *     与前轮转角，CBF 模块会基于安全约束将其修正为安全输出。
 */
struct EgoState {
  Real xg = 0.0;             ///< 自车质心全局横坐标 (m)
  Real yg = 0.0;             ///< 自车质心全局纵坐标 (m)
  Real velocity = 0.0;       ///< 自车纵向速度 v (m/s)
  Real phi = 0.0;            ///< 自车朝向角 (rad)
  Real aOriginal = 0.0;      ///< 原始期望加速度 (m/s^2)
  Real deltaFOriginal = 0.0; ///< 原始期望前轮转角 delta_f (rad)
};

/**
 * @brief 单个障碍物状态（ego frame 视角）
 *
 * 输入接口约定：相对位置与相对速度都给在自车坐标系（ego frame）下。
 *   - dxEgo > 0 表示该障碍物位于自车前方；
 *   - dyEgo > 0 表示该障碍物位于自车左侧；
 *   - vRxEgo 表示障碍物相对自车的纵向速度分量（沿自车朝向方向，m/s）；
 *   - vRyEgo 表示障碍物相对自车的横向速度分量（垂直自车朝向方向，m/s）。
 *
 * 障碍物的绝对加速度 (axGlobal, ayGlobal) 一般由感知模块输出，若不可获取，
 * 上层应填 0；该值仅用于 D-CBF 中的 \f$ \dot{x}_{tp} \f$ 项。
 */
struct ObstacleState {
  Real dxEgo = 0.0;     ///< ego frame 纵向相对位置 (m)，正为前方
  Real dyEgo = 0.0;     ///< ego frame 横向相对位置 (m)，正为左侧
  Real vRxEgo = 0.0;    ///< ego frame 纵向相对速度 (m/s)
  Real vRyEgo = 0.0;    ///< ego frame 横向相对速度 (m/s)
  Real axGlobal = 0.0;  ///< 障碍物全局横向加速度 ax (m/s^2)
  Real ayGlobal = 0.0;  ///< 障碍物全局纵向加速度 ay (m/s^2)
};

/**
 * @brief CBF 算法参数集合
 *
 * 工程要求：本结构体中的所有数值都属于「参数 (param)」语义，
 * 在初始化阶段一次性赋值，运行期间不得修改。
 */
struct CbfParam {
  Real safetyRadius = 4.0;      ///< 安全圆半径 r_safe (m)，对应论文中的 c_safe
  Real alpha1 = 1.5;            ///< HOCBF 一阶 K_inf 系数
  Real alpha2 = 1.5;            ///< HOCBF 二阶 K_inf 系数
  Real wheelBase = 2.7;         ///< 自车轴距 L (m)
  Real aMin = -7.0;             ///< 加速度下界 (m/s^2)
  Real aMax = 7.0;              ///< 加速度上界 (m/s^2)
  Real deltaFMin = -0.5;        ///< 前轮转角下界 (rad)
  Real deltaFMax = 0.5;         ///< 前轮转角上界 (rad)
  Real qDiagAccel = 1.0;        ///< QP 目标 Q 矩阵中加速度通道权重
  Real qDiagSteer = 5.0;        ///< QP 目标 Q 矩阵中转向通道权重
  std::size_t qpMaxIter = 200U; ///< QP 最大迭代步数
  Real qpTolerance = 1e-6;      ///< QP 收敛容差
};

/**
 * @brief 单条 CBF 仿射约束 \f$ \beta_a \cdot a + \beta_t \cdot \tan(\delta_f) \geq b \f$
 *
 * 约束方向约定：betaA * a + betaT * tanDeltaF >= bound
 */
struct AffineConstraint {
  Real betaA = 0.0;  ///< 加速度通道系数
  Real betaT = 0.0;  ///< 转向通道系数（作用于 tan(delta_f)）
  Real bound = 0.0;  ///< 不等式右端
};

/**
 * @brief CBF 安全控制修正模块的输出
 */
struct CbfOutput {
  Real aSafe = 0.0;          ///< 修正后的安全加速度 (m/s^2)
  Real deltaFSafe = 0.0;     ///< 修正后的安全前轮转角 (rad)
  bool feasible = true;      ///< QP 是否找到可行解（true 为可行）
  std::size_t activeNum = 0; ///< 实际生效的 CBF 约束数（用于诊断）
  std::size_t iterUsed = 0;  ///< QP 迭代实际使用步数
};

}  // namespace cbf_arbitration
