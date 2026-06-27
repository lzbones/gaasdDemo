cmake_minimum_required(VERSION 3.14)
project(WaypointFollow LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 收集所有源文件
set(LIB_SOURCES
  src/cpp/solveLinearSystem.cpp
  src/cpp/fitCubicPolynomial.cpp
  src/cpp/evaluatePolynomial.cpp
  src/cpp/evaluatePolynomialSlope.cpp
  src/cpp/predictBicycleState.cpp
  src/cpp/clampValue.cpp
  src/cpp/normalizeAngle.cpp
  src/cpp/fitWaypointTrajectory.cpp
  src/cpp/mpcComputeCost.cpp
  src/cpp/mpcOptimize.cpp
  src/cpp/waypointFollow.cpp
)

# 静态库
add_library(waypointfollow_lib STATIC ${LIB_SOURCES})
target_include_directories(waypointfollow_lib PUBLIC ${CMAKE_SOURCE_DIR}/include/cpp)

# 测试
enable_testing()

# 路点跟踪集成测试
add_executable(test_waypointFollow
  tests/cppTest/unit/waypointFollow/waypointFollow_test.cpp
)
target_link_libraries(test_waypointFollow PRIVATE waypointfollow_lib)
add_test(NAME Unit_WaypointFollow COMMAND test_waypointFollow)

# 多项式拟合单元测试
add_executable(test_fitCubicPolynomial
  tests/cppTest/unit/fitCubicPolynomial/fitCubicPolynomial_test.cpp
)
target_link_libraries(test_fitCubicPolynomial PRIVATE waypointfollow_lib)
add_test(NAME Unit_FitCubicPolynomial COMMAND test_fitCubicPolynomial)

# 自行车模型单元测试
add_executable(test_predictBicycleState
  tests/cppTest/unit/predictBicycleState/predictBicycleState_test.cpp
)
target_link_libraries(test_predictBicycleState PRIVATE waypointfollow_lib)
add_test(NAME Unit_PredictBicycleState COMMAND test_predictBicycleState)

# =========================================
# MBD FuncModule 架构库
# =========================================
# MBD 头文件搜索路径由 target_include_directories 管理

set(MBD_SOURCES
  src/mbd/ClampValue.cpp
  src/mbd/NormalizeAngle.cpp
  src/mbd/EvaluatePolynomial.cpp
  src/mbd/EvaluatePolynomialSlope.cpp
  src/mbd/PredictBicycleState.cpp
  src/mbd/FitCubicPolynomial.cpp
  src/mbd/FitWaypointTrajectory.cpp
  src/mbd/MpcComputeCost.cpp
  src/mbd/MpcOptimize.cpp
  src/mbd/WaypointFollow.cpp
)

add_library(waypointfollow_mbd_lib STATIC ${MBD_SOURCES})
target_include_directories(waypointfollow_mbd_lib PUBLIC
  ${CMAKE_SOURCE_DIR}/include/mbd   # MBD 头文件优先，避免 macOS 大小写不敏感导致 cpp 目录同名文件被误匹配
  ${CMAKE_SOURCE_DIR}/include/cpp   # MBD 头文件内部需要 waypointFollowTypes.hpp
)
target_compile_options(waypointfollow_mbd_lib PRIVATE -Wno-nonportable-include-path)

# MBD 元件单元测试: ClampValue
add_executable(mbd_test_ClampValue tests/mbdTest/unit/ClampValue/ClampValue_test.cpp)
target_link_libraries(mbd_test_ClampValue PRIVATE waypointfollow_mbd_lib)
add_test(NAME MBD_Unit_ClampValue COMMAND mbd_test_ClampValue)

# MBD 元件单元测试: NormalizeAngle
add_executable(mbd_test_NormalizeAngle tests/mbdTest/unit/NormalizeAngle/NormalizeAngle_test.cpp)
target_link_libraries(mbd_test_NormalizeAngle PRIVATE waypointfollow_mbd_lib)
add_test(NAME MBD_Unit_NormalizeAngle COMMAND mbd_test_NormalizeAngle)

# MBD 元件单元测试: EvaluatePolynomial
add_executable(mbd_test_EvaluatePolynomial tests/mbdTest/unit/EvaluatePolynomial/EvaluatePolynomial_test.cpp)
target_link_libraries(mbd_test_EvaluatePolynomial PRIVATE waypointfollow_mbd_lib)
add_test(NAME MBD_Unit_EvaluatePolynomial COMMAND mbd_test_EvaluatePolynomial)

# MBD 元件单元测试: EvaluatePolynomialSlope
add_executable(mbd_test_EvaluatePolynomialSlope tests/mbdTest/unit/EvaluatePolynomialSlope/EvaluatePolynomialSlope_test.cpp)
target_link_libraries(mbd_test_EvaluatePolynomialSlope PRIVATE waypointfollow_mbd_lib)
add_test(NAME MBD_Unit_EvaluatePolynomialSlope COMMAND mbd_test_EvaluatePolynomialSlope)

# MBD 元件单元测试: PredictBicycleState
add_executable(mbd_test_PredictBicycleState tests/mbdTest/unit/PredictBicycleState/PredictBicycleState_test.cpp)
target_link_libraries(mbd_test_PredictBicycleState PRIVATE waypointfollow_mbd_lib)
add_test(NAME MBD_Unit_PredictBicycleState COMMAND mbd_test_PredictBicycleState)

# MBD 元件单元测试: FitCubicPolynomial
add_executable(mbd_test_FitCubicPolynomial tests/mbdTest/unit/FitCubicPolynomial/FitCubicPolynomial_test.cpp)
target_link_libraries(mbd_test_FitCubicPolynomial PRIVATE waypointfollow_mbd_lib)
add_test(NAME MBD_Unit_FitCubicPolynomial COMMAND mbd_test_FitCubicPolynomial)

# MBD 组件集成测试: WaypointFollow
add_executable(mbd_test_WaypointFollow tests/mbdTest/unit/WaypointFollow/WaypointFollow_test.cpp)
target_link_libraries(mbd_test_WaypointFollow PRIVATE waypointfollow_mbd_lib)
add_test(NAME MBD_Integration_WaypointFollow COMMAND mbd_test_WaypointFollow)

# 运行所有 C++ 测试
add_custom_target(run_all_tests
  COMMAND ${CMAKE_CTEST_COMMAND} -R "Unit_" --output-on-failure
  DEPENDS test_waypointFollow test_fitCubicPolynomial test_predictBicycleState
  COMMENT "Running all C++ unit tests..."
)

# 运行所有 MBD 测试
add_custom_target(mbd_all_tests
  COMMAND ${CMAKE_CTEST_COMMAND} -R "MBD_" --output-on-failure
  DEPENDS mbd_test_ClampValue mbd_test_NormalizeAngle mbd_test_EvaluatePolynomial
          mbd_test_EvaluatePolynomialSlope mbd_test_PredictBicycleState
          mbd_test_FitCubicPolynomial mbd_test_WaypointFollow
  COMMENT "Running all MBD FuncModule tests..."
)
