cmake_minimum_required(VERSION 3.0.2)
project(rnrt_robot_controllers)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  moveit_core
  moveit_ros_planning_interface
  rnrt_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    ${PROJECT_NAME} 
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} 
    src/effort_ff_controller.cpp
    src/gravity_compensation_controller.cpp
    )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# add_library(effort_ff_controller src/effort_ff_controller.cpp)
# target_link_libraries(effort_ff_controller ${catkin_LIBRARIES})

# add_library(gravity_compensation_controller src/gravity_compensation_controller.cpp)
# target_link_libraries(gravity_compensation_controller ${catkin_LIBRARIES})

add_executable(gravity_compensation_controller_node src/gravity_compensation_node.cpp)

target_link_libraries(gravity_compensation_controller_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
