cmake_minimum_required(VERSION 3.0.2)
project(rnrt_control_tools)

## Compile as C++17
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
)

find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
    pm_motor
    transfer_fcn
    state_space_model

  CATKIN_DEPENDS 
    roscpp 

  DEPENDS
    Eigen3
)

# Libs

add_library(pm_motor src/pm_motor.cpp)
target_link_libraries(pm_motor ${catkin_LIBRARIES})

add_library(transfer_fcn src/transfer_fcn.cpp)
target_link_libraries(transfer_fcn ${catkin_LIBRARIES})

add_library(state_space_model src/state_space_model.cpp)
target_link_libraries(state_space_model ${catkin_LIBRARIES})

# if(CATKIN_ENABLE_TESTING)
  # Tests
  catkin_add_gtest(transfer_fcn_tests test/transfer_fcn_tests.cpp)
  target_link_libraries(transfer_fcn_tests ${catkin_LIBRARIES} transfer_fcn)

# endif()


install(TARGETS 
  pm_motor
  transfer_fcn
  state_space_model

  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  # FILES_MATCHING PATTERN "*.h"
)
