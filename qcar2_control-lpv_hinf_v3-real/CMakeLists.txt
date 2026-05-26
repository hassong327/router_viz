cmake_minimum_required(VERSION 3.8)
project(qcar2_control_2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(qcar2_msgs_2 REQUIRED)
find_package(qcar2_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/ph_path_publisher
  scripts/dashboard_viewer
  DESTINATION lib/${PROJECT_NAME}
)

add_library(ph_runtime OBJECT
  src/ph_runtime.cpp
)
ament_target_dependencies(ph_runtime
  qcar2_msgs_2
)
target_include_directories(ph_runtime PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

add_executable(vfg_guidance_ph_node src/vfg_guidance_ph.cpp)
ament_target_dependencies(vfg_guidance_ph_node
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  qcar2_msgs_2
  qcar2_interfaces
)
target_include_directories(vfg_guidance_ph_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(vfg_guidance_ph_node
  ph_runtime
)

add_executable(pid_lateral_controller_ph_node src/pid_controller_ph.cpp)
ament_target_dependencies(pid_lateral_controller_ph_node
  rclcpp
  std_msgs
  qcar2_msgs_2
  qcar2_interfaces
)
target_include_directories(pid_lateral_controller_ph_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

add_executable(longitudinal_controller_node src/longitudinal_controller.cpp)
ament_target_dependencies(longitudinal_controller_node
  rclcpp
  std_msgs
  qcar2_msgs_2
  qcar2_interfaces
)
target_include_directories(longitudinal_controller_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(longitudinal_controller_node
  ph_runtime
)

add_library(lpv_hinf_runtime STATIC
  src/lpv_hinf_runtime.cpp
)
target_include_directories(lpv_hinf_runtime PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

add_executable(h_inf_lateral_controller_node src/h_inf_controller.cpp)
ament_target_dependencies(h_inf_lateral_controller_node
  rclcpp
  std_msgs
  sensor_msgs
  qcar2_msgs_2
  qcar2_interfaces
  ament_index_cpp
)
target_include_directories(h_inf_lateral_controller_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(h_inf_lateral_controller_node
  lpv_hinf_runtime
)

add_executable(dashboard_logger_node src/dashboard_logger.cpp)
ament_target_dependencies(dashboard_logger_node
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  qcar2_msgs_2
  qcar2_interfaces
)

install(TARGETS
  longitudinal_controller_node
  vfg_guidance_ph_node
  pid_lateral_controller_ph_node
  h_inf_lateral_controller_node
  dashboard_logger_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
