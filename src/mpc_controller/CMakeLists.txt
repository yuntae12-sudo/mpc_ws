cmake_minimum_required(VERSION 3.0.2)
project(mpc_node)

# C++14
add_compile_options(-std=c++14 -O2)

# ========================================
# 의존성
# ========================================
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  visualization_msgs
  morai_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    nav_msgs
    visualization_msgs
    morai_msgs
    tf
)

# ========================================
# include 경로
# ========================================
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  Global
  model
  cost
  constraints
  solver
  node
  ${catkin_INCLUDE_DIRS}
)

# ========================================
# 단일 실행 파일로 빌드 (PlanningControl 스타일)
# ========================================
add_executable(mpc_node
  src/main.cpp
  src/global/global.cpp
  src/global/parameter_loader.cpp
  src/model/vehicle_model.cpp
  src/cost/cost_function.cpp
  src/constraints/constraints.cpp
  src/solver/mpc_solver.cpp
  src/node/mpc_node.cpp
  src/planner/path_planner.cpp
)

target_link_libraries(mpc_node
  ${catkin_LIBRARIES}
)

add_dependencies(mpc_node
  ${catkin_EXPORTED_TARGETS}
  morai_msgs_generate_messages_cpp
)

# ========================================
# Install (선택)
# ========================================
install(TARGETS mpc_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)