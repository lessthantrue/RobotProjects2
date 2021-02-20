cmake_minimum_required(VERSION 3.5)
project(robot_projects_simulator)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)

set(dependencies 
  rclcpp
  Eigen3
  geometry_msgs
  tf2
  visualization_msgs
)

file(GLOB all_srcs src/*.cpp src/*/*.cpp)

file(GLOB system_srcs src/systems/*.cpp)
set(SIMULATION_LIB robot_projects_simulator)
add_library(${SIMULATION_LIB} SHARED
  ${all_srcs})

target_include_directories(${SIMULATION_LIB} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${SIMULATION_LIB} ${dependencies})

file(GLOB base_srcs src/*.cpp)
add_executable(simple_sim src/executables/simple_sim.cpp)
target_link_libraries(simple_sim ${SIMULATION_LIB})

ament_target_dependencies(simple_sim ${dependencies})
ament_export_dependencies(${dependencies})

install(TARGETS ${SIMULATION_LIB} simple_sim
  EXPORT export_${PENDULUM_CONTROLLER_LIB}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
