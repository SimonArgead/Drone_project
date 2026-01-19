cmake_minimum_required(VERSION 3.8)
project(drone_sim)

# Kompilerings‐warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Kun ament_cmake – vi har ingen C++‐targets her
find_package(ament_cmake REQUIRED)

# Installér worlds, models, config og launch
install(
  DIRECTORY
    worlds
    models
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

# Kopiér hele urdf-mappen til install/share/drone_sim/urdf
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
  FILES_MATCHING PATTERN "*.xacro" PATTERN "*.urdf"
)

ament_package()