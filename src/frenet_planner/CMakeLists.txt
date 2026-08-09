cmake_minimum_required(VERSION 3.10)
project(frenet_planner_node)

# 순수 CMake 패키지 (catkin plain-cmake package). ROS/catkin 의존성 없음.
# package.xml의 <export><build_type>cmake</build_type></export>로 catkin에게 알림.
# mpc_controller/CMakeLists.txt와 동일한 스타일 - 대회 규정상 ROS 금지라
# 두 패키지 모두 단일 비-ROS 바이너리로 빌드된다.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O2)

find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

# ========================================
# include 경로
# ========================================
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${YAML_CPP_INCLUDE_DIR}
)

# ========================================
# 단일 실행 파일로 빌드
# ========================================
add_executable(frenet_planner_node
  src/main.cpp
  src/frenet_planner/frenet_planner.cpp
  src/frenet_planner/global/data_logger.cpp
  src/frenet_planner/visualization/planner_debug_writer.cpp
  src/frenet_planner/frenet/ref_line.cpp
  src/frenet_planner/frenet/path_generator.cpp
  src/frenet_planner/frenet/cost.cpp
  src/frenet_planner/frenet/collision_checker.cpp
  src/frenet_planner/frenet/leader_selector.cpp
  src/frenet_planner/frenet/avoidance_selector.cpp
  src/frenet_planner/frenet/merge_selector.cpp
  src/frenet_planner/math/polynomial.cpp
  src/frenet_planner/math/frenet_converter.cpp
  src/udp_network/udp_manager.cpp
  src/udp_network/sender/sender.cpp
  src/udp_network/sender/planned_path_sender.cpp
  src/udp_network/receiver/receiver.cpp
  src/udp_network/receiver/ego_info_receiver.cpp
  src/udp_network/receiver/object_info_receiver.cpp
)

target_link_libraries(frenet_planner_node
  yaml-cpp
  Threads::Threads
)

# 실행 위치(cwd)와 무관하게 config를 찾도록 절대경로를 컴파일 타임에 주입
target_compile_definitions(frenet_planner_node PRIVATE
  FRENET_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src"
)

# ========================================
# Install
# ========================================
install(TARGETS frenet_planner_node
  RUNTIME DESTINATION bin
)

install(PROGRAMS tools/planner_visualizer.py
  DESTINATION bin
)

install(DIRECTORY src/frenet_planner/config/
  DESTINATION share/${PROJECT_NAME}/frenet_planner/config
)

install(DIRECTORY src/udp_network/
  DESTINATION share/${PROJECT_NAME}/udp_network
  FILES_MATCHING PATTERN "*.yaml"
)
