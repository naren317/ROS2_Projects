cmake_minimum_required(VERSION 3.8)
project(pointcloud)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

#add_executable(cloud_filter src/cloud_filter.cpp)
#ament_target_dependencies(cloud_filter rclcpp sensor_msgs pcl_conversions pcl_ros)

add_executable(point_cloud_processing src/point_cloud_processing.cpp)

ament_target_dependencies(point_cloud_processing rclcpp sensor_msgs pcl_conversions pcl_ros tf2_ros geometry_msgs tf2_geometry_msgs)

#install(TARGETS cloud_filter DESTINATION lib/${PROJECT_NAME})

install(TARGETS point_cloud_processing DESTINATION lib/${PROJECT_NAME})

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
