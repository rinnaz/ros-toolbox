cmake_minimum_required(VERSION 3.0.2)
project(rnrt_aruco_detector)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  geometry_msgs
  tf2
  tf2_geometry_msgs
  rnrt_msgs
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_converter
#  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs geometry_msgs message_runtime
#  DEPENDS system_lib
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${rrt_cv_gibrid_INCLUDE_DIRS} ##gibrid
)


add_executable(rnrt_aruco_detector src/main.cpp src/rnrt_aruco_detector.cpp)

add_dependencies(${PROJECT_NAME} rnrt_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(rnrt_aruco_detector
  ${OpenCV_LIBRARIES} 
  ${catkin_LIBRARIES}
)



