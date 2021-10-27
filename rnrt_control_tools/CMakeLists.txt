cmake_minimum_required(VERSION 3.0.2)
project(rnrt_control_tools)

## Compile as C++17
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
    ode_solver

  CATKIN_DEPENDS 
    roscpp 

  DEPENDS
)

# Libs

add_library(ode_solver src/ode_solver.cpp)
target_link_libraries(ode_solver ${catkin_LIBRARIES})

install(TARGETS 
  ode_solver

  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  # FILES_MATCHING PATTERN "*.h"
)
