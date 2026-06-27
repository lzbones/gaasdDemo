#pragma once

/// @file waypointFollowTypes.hpp
/// @brief 路点跟踪控制器的共用类型定义

namespace waypoint_follow {

constexpr int NUM_WAYPOINTS = 6;           ///< 参考路点数量
constexpr int POLY_COEFFS_NUM = 4;         ///< 三次多项式系数数量 (阶数+1)
constexpr int MAX_PREDICTION_HORIZON = 30; ///< MPC最大预测步数
constexpr int MAX_CONTROL_HORIZON = 15;    ///< MPC最大控制步数
constexpr int MAX_LINEAR_SYSTEM_DIM = 8;   ///< 线性方程组求解器最大维数
constexpr double MIN_SPEED_THRESHOLD = 0.5;///< 控制器生效最低车速 (m/s)

/// 二维路点坐标
struct Waypoint {
  double x = 0.0; ///< 纵向坐标 (m), 自车坐标系前方为正
  double y = 0.0; ///< 横向坐标 (m), 自车坐标系左方为正
};

/// 三次多项式系数: y = c[0] + c[1]*x + c[2]*x^2 + c[3]*x^3
struct PolynomialCoeffs {
  double c[POLY_COEFFS_NUM] = {0.0, 0.0, 0.0, 0.0};
};

/// 拟合后的参考轨迹（含归一化信息）
struct FittedTrajectory {
  PolynomialCoeffs poly; ///< 归一化坐标系下的多项式系数
  double xOffset = 0.0;  ///< X坐标归一化偏移量 (m)
  double xScale = 1.0;   ///< X坐标归一化缩放因子 (m)
  double refSpeed = 0.0; ///< 路点推算的参考车速 (m/s)
  bool valid = false;    ///< 轨迹拟合是否有效
};

/// 车辆运动状态
struct VehicleState {
  double x = 0.0;   ///< 纵向位置 (m)
  double y = 0.0;   ///< 横向位置 (m)
  double yaw = 0.0; ///< 航向角 (rad), 逆时针为正
  double v = 0.0;   ///< 纵向速度 (m/s)
};

/// 车辆控制输入量
struct ControlInput {
  double steeringAngle = 0.0; ///< 前轮转向角 (rad), 左转为正
  double acceleration = 0.0;  ///< 纵向加速度 (m/s^2), 加速为正
};

/// MPC控制参数集
struct MpcParam {
  int predictionHorizon = 15;       ///< 预测步数
  int controlHorizon = 5;          ///< 控制步数
  double dt = 0.1;                  ///< 预测时间步长 (s)
  double wheelbase = 2.85;          ///< 前后轴距 (m)
  double maxSteeringAngle = 0.5236; ///< 前轮最大转向角 (rad), 约30度
  double maxAcceleration = 3.0;     ///< 最大纵向加速度 (m/s^2)
  double minAcceleration = -5.0;    ///< 最大纵向减速度 (m/s^2)
  double maxSteeringRate = 0.1;     ///< 每步最大转向角变化量 (rad/step)
  double wLateral = 100.0;         ///< 横向偏差代价权重
  double wHeading = 50.0;          ///< 航向偏差代价权重
  double wVelocity = 5.0;          ///< 速度偏差代价权重
  double wSteering = 1.0;          ///< 转向角控制量代价权重
  double wAccel = 0.5;             ///< 加速度控制量代价权重
  double wSteeringRate = 10.0;     ///< 转向角变化率代价权重
  double wAccelRate = 5.0;         ///< 加速度变化率代价权重
  int maxOptIterations = 80;       ///< 梯度下降最大迭代次数
  double learningRate = 0.1;       ///< 梯度下降初始学习率 (配合梯度范数裁剪使用)
  double finiteDiffEps = 1e-4;    ///< 数值梯度中心差分步长
};

/// 路点跟踪控制器输入结构体
struct WaypointFollowInput {
  double egoSpeed = 0.0;                  ///< 自车车速 (m/s)
  double currentSteeringWheelAngle = 0.0; ///< 当前方向盘转角 (rad)
  Waypoint waypoints[NUM_WAYPOINTS];      ///< 6个参考路点, 对应未来0.5/1.0/1.5/2.0/2.5/3.0秒位置
};

/// 路点跟踪控制器输出结构体
struct WaypointFollowOutput {
  double steeringWheelAngle = 0.0; ///< 目标方向盘转角 (rad)
  double frontWheelAngle = 0.0;    ///< 目标前轮转角 (rad)，方向盘转角 / 传动比
  double acceleration = 0.0;       ///< 目标纵向加速度 (m/s^2)
};

/// 路点跟踪控制器参数结构体
struct WaypointFollowParam {
  MpcParam mpcParam;           ///< MPC控制参数集
  double steeringRatio = 15.0; ///< 方向盘到前轮转角传动比 (无量纲)
};

/// 路点跟踪控制器状态结构体（跨周期持久化）
struct WaypointFollowState {
  ControlInput prevControlSequence[MAX_CONTROL_HORIZON]; ///< 上一周期MPC控制序列 (暖启动用)
  ControlInput lastAppliedControl;                       ///< 上一周期实际施加的控制量
  bool initialized = false;                              ///< 暖启动状态是否已初始化
};

} // namespace waypoint_follow
