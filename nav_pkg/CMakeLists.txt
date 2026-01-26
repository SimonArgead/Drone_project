cmake_minimum_required(VERSION 3.16)
project(nav_pkg)

# === Warnings ===
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === ROS 2 dependencies ===
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# === Executables ===
add_executable(rtabmap_nav src/rtabmap_nav.cpp)
add_executable(vision_twist_node src/vision_twist_node.cpp)
add_executable(autopilot src/autopilot.cpp)
add_executable(low_level_cnt src/low_level_cnt.cpp)

# === Include directories ===
target_include_directories(rtabmap_nav PRIVATE
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(autopilot PRIVATE
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# === Dependencies (ROS messages MUST use ament_target_dependencies) ===
ament_target_dependencies(rtabmap_nav
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
)

ament_target_dependencies(vision_twist_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
)

ament_target_dependencies(autopilot
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)

ament_target_dependencies(low_level_cnt
  rclcpp
  std_msgs)

install(TARGETS
  low_level_cnt
  DESTINATION lib/${PROJECT_NAME})

# === Install ===
install(TARGETS
  rtabmap_nav
  vision_twist_node
  autopilot
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/config)

ament_package()
