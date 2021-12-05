cmake_minimum_required(VERSION 3.0.2)
project(rnrt_control_tools)

## Compile as C++17
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  control_toolbox 
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
    ${PROJECT_NAME}

  CATKIN_DEPENDS 
    roscpp
    control_toolbox 

  DEPENDS
    Eigen3
)

# Libs

add_library(${PROJECT_NAME} 
    src/transfer_function_info.cpp
    src/state_space_model.cpp
    src/pm_motor.cpp
    src/current_servo.cpp
    src/linear_system.cpp
    src/butterworth_filter.cpp
    src/mass_spring_damper.cpp
    )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(test_node src/main.cpp)

target_link_libraries(test_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


if(CATKIN_ENABLE_TESTING)
#   Tests
  catkin_add_gtest(transfer_function_info_tests test/transfer_function_info_tests.cpp)
  target_link_libraries(transfer_function_info_tests ${catkin_LIBRARIES} ${PROJECT_NAME})

  catkin_add_gtest(state_space_model_tests test/state_space_model_tests.cpp)
  target_link_libraries(state_space_model_tests ${catkin_LIBRARIES} ${PROJECT_NAME})

  catkin_add_gtest(pm_motor_tests test/pm_motor_tests.cpp)
  target_link_libraries(pm_motor_tests ${catkin_LIBRARIES} ${PROJECT_NAME})
  
  catkin_add_gtest(linear_system_tests test/linear_system_tests.cpp)
  target_link_libraries(linear_system_tests ${catkin_LIBRARIES} ${PROJECT_NAME})

endif()


install(TARGETS 
  ${PROJECT_NAME}

  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  # FILES_MATCHING PATTERN "*.h"
)
