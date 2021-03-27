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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rcutils REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

set(SIMULATION_LIB robot_projects_simulator)

set(dependencies 
  rclcpp
  Eigen3
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_msgs
  visualization_msgs
  pcl_conversions
  rcl_interfaces
  rcutils
)

file(GLOB all_srcs src/*.cpp src/*/*.cpp)
file(GLOB all_headers include/*.h include/*/*.h)

add_library(${SIMULATION_LIB} SHARED
  ${all_srcs} ${all_headers})

target_include_directories(${SIMULATION_LIB} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/systems)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sensors)

link_directories(${PCL_INCLUDE_DIRS})
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/systems)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/sensors)

# target_link_libraries(${SIMULATION_LIB} ${dependencies})
ament_target_dependencies(${SIMULATION_LIB} ${dependencies})

add_executable(ekf_sim_world executables/ekf_sim_world.cpp)
add_executable(test_sim_world executables/test_sim_world.cpp)

target_link_libraries(ekf_sim_world 
  ${SIMULATION_LIB}
  ${BOOST_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES})
target_link_libraries(test_sim_world 
  ${SIMULATION_LIB}
  ${BOOST_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES})

target_include_directories(ekf_sim_world PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

ament_target_dependencies(ekf_sim_world ${dependencies})
ament_target_dependencies(test_sim_world ${dependencies})
ament_export_dependencies(${dependencies})

install(TARGETS ${SIMULATION_LIB} ekf_sim_world
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${SIMULATION_LIB} test_sim_world
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
