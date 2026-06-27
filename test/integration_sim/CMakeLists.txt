cmake_minimum_required(VERSION 3.16)
project(IntegrationSim CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---------------------------------------------------------------------------
# 路径约定：本 CMake 位于 <repo-root>/test/integration_sim
# ---------------------------------------------------------------------------
set(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(CBF_ROOT ${REPO_ROOT}/cbf)
set(WF_ROOT ${REPO_ROOT}/waypointFollow)

# ---------------------------------------------------------------------------
# CBF 核心静态库
# ---------------------------------------------------------------------------
set(CBF_SOURCES
  ${CBF_ROOT}/src/cpp/rotateVectorByMinusYaw.cpp
  ${CBF_ROOT}/src/cpp/computeBarrierSquared.cpp
  ${CBF_ROOT}/src/cpp/computeBarrierFirstDerivative.cpp
  ${CBF_ROOT}/src/cpp/computeBarrierSecondDerivativeFree.cpp
  ${CBF_ROOT}/src/cpp/computeObstacleAffineConstraint.cpp
  ${CBF_ROOT}/src/cpp/clampControlInput.cpp
  ${CBF_ROOT}/src/cpp/solveQpHildreth.cpp
  ${CBF_ROOT}/src/cpp/cbfControlRevision.cpp
)

add_library(cbf_core STATIC ${CBF_SOURCES})
target_include_directories(cbf_core PUBLIC ${CBF_ROOT}/include/cpp)
target_compile_options(cbf_core PRIVATE -Wall -Wextra -Wno-unused-parameter)

# ---------------------------------------------------------------------------
# WaypointFollow 静态库
# ---------------------------------------------------------------------------
set(WF_SOURCES
  ${WF_ROOT}/src/cpp/solveLinearSystem.cpp
  ${WF_ROOT}/src/cpp/fitCubicPolynomial.cpp
  ${WF_ROOT}/src/cpp/evaluatePolynomial.cpp
  ${WF_ROOT}/src/cpp/evaluatePolynomialSlope.cpp
  ${WF_ROOT}/src/cpp/predictBicycleState.cpp
  ${WF_ROOT}/src/cpp/clampValue.cpp
  ${WF_ROOT}/src/cpp/normalizeAngle.cpp
  ${WF_ROOT}/src/cpp/fitWaypointTrajectory.cpp
  ${WF_ROOT}/src/cpp/mpcComputeCost.cpp
  ${WF_ROOT}/src/cpp/mpcOptimize.cpp
  ${WF_ROOT}/src/cpp/waypointFollow.cpp
)

add_library(waypointfollow_lib STATIC ${WF_SOURCES})
target_include_directories(waypointfollow_lib PUBLIC ${WF_ROOT}/include/cpp)
target_compile_options(waypointfollow_lib PRIVATE -Wall -Wextra)

# ---------------------------------------------------------------------------
# 集成仿真可执行文件
# ---------------------------------------------------------------------------
add_executable(integration_sim src/integration_sim.cpp)
target_link_libraries(integration_sim PRIVATE waypointfollow_lib cbf_core)

# ---------------------------------------------------------------------------
# 自定义运行目标
# ---------------------------------------------------------------------------
add_custom_target(run_integration_sim
  COMMAND integration_sim
  DEPENDS integration_sim
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Running waypointFollow + cbf integration simulation..."
)
