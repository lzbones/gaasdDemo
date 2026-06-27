cmake_minimum_required(VERSION 3.16)
project(cbfControlRevision CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wno-unused-parameter)

# ---------------------------------------------------------------------------
# 核心算法静态库（One Function Per File 汇总）
# ---------------------------------------------------------------------------
set(CBF_SOURCES
  src/cpp/rotateVectorByMinusYaw.cpp
  src/cpp/computeBarrierSquared.cpp
  src/cpp/computeBarrierFirstDerivative.cpp
  src/cpp/computeBarrierSecondDerivativeFree.cpp
  src/cpp/computeObstacleAffineConstraint.cpp
  src/cpp/clampControlInput.cpp
  src/cpp/solveQpHildreth.cpp
  src/cpp/cbfControlRevision.cpp
)

add_library(cbf_arbitration_core STATIC ${CBF_SOURCES})
target_include_directories(cbf_arbitration_core PUBLIC ${CMAKE_SOURCE_DIR}/include/cpp)

# ---------------------------------------------------------------------------
# 单元测试
# ---------------------------------------------------------------------------
enable_testing()

set(UNIT_TESTS
  rotateVectorByMinusYaw
  computeBarrierSquared
  computeBarrierFirstDerivative
  computeBarrierSecondDerivativeFree
  computeObstacleAffineConstraint
  clampControlInput
  solveQpHildreth
  cbfControlRevision
)

foreach(name ${UNIT_TESTS})
  set(target_name test_${name})
  add_executable(${target_name} tests/cppTest/unit/${name}/${name}_test.cpp)
  target_link_libraries(${target_name} PRIVATE cbf_arbitration_core)
  add_test(NAME Unit_${name} COMMAND ${target_name})
endforeach()

# ---------------------------------------------------------------------------
# 仿真测试场景可执行文件
# ---------------------------------------------------------------------------
add_executable(simulate_scenarios tests/cppTest/sim/simulate_scenarios.cpp)
target_link_libraries(simulate_scenarios PRIVATE cbf_arbitration_core)

# ---------------------------------------------------------------------------
# MBD FuncModule 静态库
# ---------------------------------------------------------------------------
set(CBF_MBD_SOURCES
  src/mbd/RotateVectorByMinusYaw.cpp
  src/mbd/ComputeBarrierSquared.cpp
  src/mbd/ComputeBarrierFirstDerivative.cpp
  src/mbd/ComputeBarrierSecondDerivativeFree.cpp
  src/mbd/ComputeObstacleAffineConstraint.cpp
  src/mbd/ClampControlInput.cpp
  src/mbd/SolveQpHildreth.cpp
  src/mbd/CbfControlRevision.cpp
)

add_library(cbf_arbitration_mbd STATIC ${CBF_MBD_SOURCES})
target_include_directories(cbf_arbitration_mbd PUBLIC ${CMAKE_SOURCE_DIR}/include/mbd)

# MBD 单元测试
set(MBD_UNIT_TESTS
  RotateVectorByMinusYaw
  ComputeBarrierSquared
  ComputeBarrierFirstDerivative
  ComputeBarrierSecondDerivativeFree
  ComputeObstacleAffineConstraint
  ClampControlInput
  SolveQpHildreth
  CbfControlRevision
)

foreach(name ${MBD_UNIT_TESTS})
  set(target_name mbd_test_${name})
  add_executable(${target_name} tests/mbdTest/unit/${name}/${name}_test.cpp)
  target_link_libraries(${target_name} PRIVATE cbf_arbitration_mbd)
  add_test(NAME MbdUnit_${name} COMMAND ${target_name})
endforeach()
