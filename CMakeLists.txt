cmake_minimum_required(VERSION 2.8.3)
project(mrs_mpc_solvers)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES MpcTrackerSolver MpcControllerSolver
  DEPENDS Eigen
  )

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

# CVXgen wrapper for MPC tracker

add_library(MpcTrackerSolver src/mpc_tracker/mpc_tracker_solver.cpp
  src/mpc_tracker/solver/ldl.cpp
  src/mpc_tracker/solver/matrix_support.cpp
  src/mpc_tracker/solver/solver.cpp
  src/mpc_tracker/solver/util.cpp
  )

add_library(MpcControllerSolver src/mpc_controller/mpc_controller_solver.cpp
  src/mpc_controller/solver/ldl.cpp
  src/mpc_controller/solver/matrix_support.cpp
  src/mpc_controller/solver/solver.cpp
  src/mpc_controller/solver/util.cpp
  )

install(TARGETS
  MpcTrackerSolver
  MpcControllerSolver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  )
