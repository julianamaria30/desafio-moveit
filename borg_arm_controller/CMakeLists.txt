cmake_minimum_required(VERSION 3.0.2)
project(borg_arm_controller)

add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  actionlib
  actionlib_msgs
  open_manipulator_msgs
  moveit_msgs
  moveit_core
  moveit_ros_perception
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  dynamixel_workbench_msgs
  dynamixel_workbench_toolbox
  message_generation
  cmake_modules
  geometric_shapes
  interactive_markers
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs moveit_msgs trajectory_msgs open_manipulator_msgs moveit_core moveit_ros_planning moveit_ros_planning_interface angles
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(moveit_bridge src/moveit_bridge.cpp)
add_dependencies(moveit_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(moveit_bridge ${catkin_LIBRARIES})

# add_executable(move_group_interface src/move_group_interface.cpp)
# target_link_libraries(move_group_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# install(TARGETS move_group_interface DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


add_executable(borg_arm_controller src/borg_arm_controller.cpp)
target_link_libraries(borg_arm_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS borg_arm_controller DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(borg_arm_controller_simulation src/borg_arm_controller_simulation.cpp)
target_link_libraries(borg_arm_controller_simulation ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS borg_arm_controller_simulation DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

################################################################################
# Install
################################################################################
install(TARGETS moveit_bridge
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
