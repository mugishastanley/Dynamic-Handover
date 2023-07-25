#copyright Mugisha 06/2023
cmake_minimum_required(VERSION 3.8)
project(hello_moveit_ur)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


# Add executable
add_executable(hello_moveit_ur src/hello_moveit_ur.cpp)
add_executable(point_to_point src/point_to_point.cpp)
add_executable(moveit_eval src/moveit_eval.cpp)

# Include directories
target_include_directories(hello_moveit_ur PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


target_include_directories(point_to_point PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

target_include_directories(moveit_eval PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)


target_compile_features(hello_moveit_ur PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  hello_moveit_ur
  "moveit_ros_planning_interface"
  "rclcpp"
)

target_compile_features(point_to_point PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  point_to_point
  "moveit_ros_planning_interface"
  "rclcpp"
  "geometry_msgs"
)

target_compile_features(moveit_eval PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  moveit_eval
  "moveit_ros_planning_interface"
  "rclcpp"
  "geometry_msgs"
)


# Install the executable
install(TARGETS hello_moveit_ur
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS point_to_point
  DESTINATION lib/${PROJECT_NAME})

  install(TARGETS moveit_eval
  DESTINATION lib/${PROJECT_NAME})


# Install launch files.
install(DIRECTORY
  launch
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
