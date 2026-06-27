# waypointFollow + cbf 集成仿真测试

本目录存放 `waypointFollow`（端到端路点跟踪 MPC 控制器）与 `cbf`（基于控制障碍函数的安全控制修正模块）的联合闭环仿真测试。

## 目录结构

```
test/integration_sim/
├── CMakeLists.txt              # 集成测试构建配置
├── src/
│   └── integration_sim.cpp     # 联合仿真主程序
├── scripts/
│   └── plot_scenarios.py       # 结果可视化脚本
├── output/                     # 仿真输出 CSV 与 PNG
│   ├── lead_brake.csv
│   ├── cut_in.csv
│   ├── cross_pedestrian.csv
│   └── *.png
└── build/                      # CMake 构建目录
```

## 构建与运行

```bash
cd test/integration_sim
cmake -B build
cmake --build build
./build/integration_sim
```

## 可视化

```bash
python3 scripts/plot_scenarios.py
```

## 测试场景

1. **lead_brake**：同车道前车 t=2s 开始急刹，验证 CBF 纵向制动能力。
2. **cut_in**：右后方车辆向左切入，验证 CBF 减速 + 横向避让能力。
3. **cross_pedestrian**：行人从右侧横穿，验证 CBF 减速让行能力。
4. **swerve_obstacle**：车道中心偏右静止障碍物，降低 CBF 转向权重，验证 CBF 以转向为主的规避能力。

## 关键接口转换

- `waypointFollow` 输出**方向盘转角** `steeringWheelAngle`、**前轮转角** `frontWheelAngle` 与**加速度** `acceleration`。
- `frontWheelAngle` 直接作为 `cbf` 的 `deltaFOriginal`，无需再除以 `steeringRatio`。
- `acceleration` 直接作为 `cbf` 的 `aOriginal`。
- `cbf` 输出安全前轮转角 `deltaFSafe`，直接用于单车自行车模型推进。
- 障碍物从全局坐标系通过 `toEgoFrame()` 转换到 ego frame 后输入 `cbf`。

## 成功标准

- 所有场景仿真成功生成 CSV。
- 任意时刻自车与最近障碍物的距离 ≥ 安全半径 5 m。
- 当存在碰撞风险时，`cbf` 输出的 `a_safe` 应小于 `waypointFollow` 原始 `acceleration`。
- 所有 QP 求解步骤 `feasible = true`。

## 仿真结果

| 场景 | 策略 | 最小距离 | 最终速度 |
|---|---|---|---|
| `lead_brake` | 纯纵向制动 | 7.40 m | 4.06 m/s |
| `cut_in` | 减速 + 轻微转向 | 5.55 m | 11.81 m/s |
| `cross_pedestrian` | 减速为主 | 5.17 m | 0.45 m/s |
| `swerve_obstacle` | 转向为主 | 16.35 m | 14.60 m/s |

**典型场景轨迹：**

| 前车急刹 `lead_brake` | 切入 `cut_in` |
|---|---|
| ![lead_brake](output/lead_brake.png) | ![cut_in](output/cut_in.png) |

| 行人横穿 `cross_pedestrian` | 转向规避 `swerve_obstacle` |
|---|---|
| ![cross_pedestrian](output/cross_pedestrian.png) | ![swerve_obstacle](output/swerve_obstacle.png) |
