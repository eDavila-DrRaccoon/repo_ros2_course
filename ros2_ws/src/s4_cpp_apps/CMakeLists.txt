cmake_minimum_required(VERSION 3.8)
project(s4_cpp_apps)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(s4_custom_interface REQUIRED)

add_executable(publisher_exe src/publisher.cpp)
ament_target_dependencies(publisher_exe rclcpp s4_custom_interface)

add_executable(subscriber_exe src/subscriber.cpp)
ament_target_dependencies(subscriber_exe rclcpp s4_custom_interface)

add_executable(server_exe src/server.cpp)
ament_target_dependencies(server_exe rclcpp s4_custom_interface)

add_executable(client_exe src/client.cpp)
ament_target_dependencies(client_exe rclcpp s4_custom_interface)

install(TARGETS
  publisher_exe
  subscriber_exe
  server_exe
  client_exe
  DESTINATION lib/${PROJECT_NAME}
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
