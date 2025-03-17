cmake_minimum_required(VERSION 3.8)
project(rtos_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)

# Add executable
add_executable(rtos_node src/pub.cpp)
ament_target_dependencies(rtos_node rclcpp rclcpp_action nav2_msgs std_msgs)

# Set C++ standard and real-time priority flags
target_compile_features(rtos_node PRIVATE cxx_std_17)
target_link_libraries(rtos_node pthread)  # Link pthread for multi-threading

add_executable(rtos_delivery_scheduler src/goalScheduler.cpp)
ament_target_dependencies(rtos_delivery_scheduler rclcpp rclcpp_action nav2_msgs std_msgs geometry_msgs)

add_executable(moveRobot src/moveRobot.cpp)
ament_target_dependencies(moveRobot rclcpp rclcpp_action nav2_msgs geometry_msgs)

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

# Install
install(TARGETS
  rtos_node
  rtos_delivery_scheduler
  moveRobot
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
