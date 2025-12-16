cmake_minimum_required(VERSION 3.8)
project(lidar_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

pkg_check_modules(GPIOD libgpiod REQUIRED)

include_directories(
  include
)

add_executable(lds_lidar src/lds_lidar.cpp)
ament_target_dependencies(lds_lidar 
  rclcpp 
  sensor_msgs
  tf2_ros
  geometry_msgs
  )

target_include_directories(lds_lidar PUBLIC ${GPIOD_INCLUDE_DIRS})
target_link_libraries(lds_lidar ${GPIOD_LIBRARIES})

install(TARGETS
  lds_lidar
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
