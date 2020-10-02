cmake_minimum_required(VERSION 2.8.3)
project(ras)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  interactive_markers
  tf
  visualization_msgs
  message_generation
  std_msgs
  derived_object_msgs
  dynamic_reconfigure
  carla_msgs
  jsk_rviz_plugins
  radar_msgs
  shape_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
	FILES
    RasObject.msg
    RasObjectArray.msg
)

generate_messages(
    DEPENDENCIES
   	std_msgs derived_object_msgs geometry_msgs
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
cfg/ras.cfg
)

catkin_package(
INCLUDE_DIRS include
LIBRARIES ras_lib
CATKIN_DEPENDS roscpp
std_msgs
autoware_msgs
interactive_markers
tf
visualization_msgs
message_runtime
derived_object_msgs
 # DEPENDS system_lib eigen opencv
)

include_directories(
include
${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(ras_lib lib/ras_lib.cpp)
add_dependencies(ras_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(ras_lib ${catkin_LIBRARIES})

add_executable(ras_core_node src/ras_core/ras_core_node.cpp src/ras_core/ras_core.cpp)
target_link_libraries(ras_core_node ras_lib ${catkin_LIBRARIES})
add_dependencies(ras_core_node ${catkin_EXPORTED_TARGETS} ras_gencfg)

add_executable(ras_visualizer_node src/ras_visualizer/ras_visualizer_node.cpp src/ras_visualizer/ras_visualizer.cpp)
target_link_libraries(ras_visualizer_node ras_lib ${catkin_LIBRARIES})
add_dependencies(ras_visualizer_node ${catkin_EXPORTED_TARGETS})

add_executable(ras_autoware_connector_node src/ras_autoware_connector/ras_autoware_connector_node.cpp src/ras_autoware_connector/ras_autoware_connector.cpp)
target_link_libraries(ras_autoware_connector_node ras_lib ${catkin_LIBRARIES})
add_dependencies(ras_autoware_connector_node ${catkin_EXPORTED_TARGETS})
