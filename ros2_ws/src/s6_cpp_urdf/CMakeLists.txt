cmake_minimum_required(VERSION 3.8)
project(s6_cpp_urdf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Added manually
add_executable(cylinder_exe src/cylinder.cpp)
ament_target_dependencies(cylinder_exe rclcpp geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs)

add_executable(wheel_exe src/wheel.cpp)
ament_target_dependencies(wheel_exe rclcpp geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs)

add_executable(r2d2_exe src/r2d2.cpp)
ament_target_dependencies(r2d2_exe rclcpp geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs)

add_executable(rdk_x3_robot_exe src/rdk_x3_robot.cpp)
ament_target_dependencies(rdk_x3_robot_exe rclcpp geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs)

# Added manually
install(TARGETS
  cylinder_exe
  wheel_exe
  r2d2_exe
  rdk_x3_robot_exe
  DESTINATION lib/${PROJECT_NAME}
)

# Added manually
install(DIRECTORY 
  launch
  meshes
  rviz
  urdf
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
