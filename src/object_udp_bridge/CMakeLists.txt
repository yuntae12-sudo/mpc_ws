cmake_minimum_required(VERSION 3.0.2)
project(object_udp_bridge)

add_compile_options(-std=c++17)
find_package(catkin REQUIRED COMPONENTS roscpp morai_msgs)

catkin_package(CATKIN_DEPENDS roscpp morai_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(object_udp_bridge_node src/bridge_node.cpp)
target_link_libraries(object_udp_bridge_node ${catkin_LIBRARIES})
add_dependencies(object_udp_bridge_node ${catkin_EXPORTED_TARGETS})
