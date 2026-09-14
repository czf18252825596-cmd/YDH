cmake_minimum_required(VERSION 3.0.2)
project(usvc_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  usvc_navi
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common)

add_message_files(
  FILES
  GuidanceCmd.msg
  LosPath.msg
  OtterCtrl.msg
  OtterStatus.msg
  APFUpdate.msg
  TunableParams.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp std_msgs sensor_msgs geometry_msgs nav_msgs
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/util.cpp
  src/geo_util.cpp
  src/apf_avoid.cpp
  src/mode_manager.cpp
  src/los_tracker.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(planner_node src/planner_node.cpp)
target_link_libraries(planner_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

# 安装 Python 脚本（fake_ship.py 等）
catkin_install_python(PROGRAMS scripts/fake_ship.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})