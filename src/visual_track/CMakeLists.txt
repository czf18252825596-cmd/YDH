cmake_minimum_required(VERSION 3.0.2)
project(visual_track)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  usvc_planner
  yolo_rtsp
)

add_message_files(
  FILES
  VisualTrack.msg

)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp std_msgs
  # INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(visual_track_node src/visual_track_node.cpp)
target_link_libraries(visual_track_node ${catkin_LIBRARIES})
add_dependencies(visual_track_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})