cmake_minimum_required(VERSION 3.0.2)
project(behavior_planner)

add_compile_options(-std=c++17 -Wall -Wextra)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs visualization_msgs morai_msgs message_generation)
find_package(yaml-cpp REQUIRED)

add_message_files(FILES BehaviorContext.msg UrbanFeatureDebug.msg PlanFeedback.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS roscpp std_msgs geometry_msgs visualization_msgs morai_msgs message_runtime)
# Preserve the requested src/<area> layout while exposing conventional
# behavior_planner/<area>/... include paths to this target.
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/include)
execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_BINARY_DIR}/include/behavior_planner)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include ${catkin_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS})

set(BEHAVIOR_NODE_SOURCES
  src/main.cpp src/node/behavior_node.cpp src/node/morai_msg_converter.cpp
  src/map/mgeo_loader.cpp src/map/route_manager.cpp
  src/feature/feature_builder.cpp src/feature/object_projector.cpp
  src/rule/hard_rule_filter.cpp
  src/decision/behavior_decision.cpp src/decision/behavior_scorer.cpp
  src/decision/fsm.cpp src/decision/transition_condition.cpp
  src/context/msg_converter.cpp
  src/signal/mock_traffic_light_provider.cpp
  src/global/utils.cpp src/global/data_logger.cpp
  src/visualization/behavior_visualization.cpp)
file(GLOB_RECURSE BEHAVIOR_INTERNAL_HEADERS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp")
# The requested source layout is exposed through a generated include symlink.
# Explicit object dependencies ensure every translation unit is rebuilt when an
# internal struct header changes, preventing stale ABI layouts in incremental builds.
set_property(SOURCE ${BEHAVIOR_NODE_SOURCES} APPEND PROPERTY
  OBJECT_DEPENDS "${BEHAVIOR_INTERNAL_HEADERS}")
add_executable(behavior_node ${BEHAVIOR_NODE_SOURCES} ${BEHAVIOR_INTERNAL_HEADERS})
add_dependencies(behavior_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(behavior_node ${catkin_LIBRARIES} yaml-cpp)

install(TARGETS behavior_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS
  scripts/behavior_dashboard_server.py
  scripts/open_dashboard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch src/config web DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(behavior_logic_test
    test/behavior_logic_test.cpp
    src/rule/hard_rule_filter.cpp
    src/decision/behavior_scorer.cpp
    src/decision/behavior_decision.cpp)
  if(TARGET behavior_logic_test)
    add_dependencies(behavior_logic_test
      ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_include_directories(behavior_logic_test PRIVATE
      ${CMAKE_CURRENT_BINARY_DIR}/include ${catkin_INCLUDE_DIRS})
    target_link_libraries(behavior_logic_test ${catkin_LIBRARIES})
  endif()
endif()
