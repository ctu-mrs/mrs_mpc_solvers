cmake_minimum_required(VERSION 3.12)
project(mrs_mpc_solvers)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  Eigen3
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

set(LIBRARIES
  MrsMpcSolvers_MpcTrackerSolver
  MrsMpcSolvers_MpcControllerSolver
  )
  
## --------------------------------------------------------------
## |                       compile                              |
## --------------------------------------------------------------

# MPC Tracker

add_library(MrsMpcSolvers_MpcTrackerSolver SHARED
  src/mpc_tracker/mpc_tracker_solver.cpp
  src/mpc_tracker/solver/ldl.cpp
  src/mpc_tracker/solver/matrix_support.cpp
  src/mpc_tracker/solver/solver.cpp
  src/mpc_tracker/solver/util.cpp
  )

target_include_directories(MrsMpcSolvers_MpcTrackerSolver
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(MrsMpcSolvers_MpcTrackerSolver
  rclcpp
  Eigen3
  )

# MPC Controller

add_library(MrsMpcSolvers_MpcControllerSolver SHARED
  src/mpc_controller/mpc_controller_solver.cpp
  src/mpc_controller/solver/ldl.cpp
  src/mpc_controller/solver/matrix_support.cpp
  src/mpc_controller/solver/solver.cpp
  src/mpc_controller/solver/util.cpp
  )

ament_target_dependencies(MrsMpcSolvers_MpcControllerSolver
  rclcpp
  Eigen3
  )

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

ament_export_libraries(
  ${LIBRARIES}
)

install(TARGETS
  ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(
  include/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
