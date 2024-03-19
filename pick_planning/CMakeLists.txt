cmake_minimum_required(VERSION 3.8)
project(pick_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(moveit REQUIRED)
find_package(image_geometry REQUIRED)
find_package(pick_planning_interfaces REQUIRED)
find_package(gazebo_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})


add_executable(move_arm src/move_arm.cpp)
target_include_directories(move_arm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(move_arm PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  move_arm
  "moveit_ros_planning_interface"
  "pick_planning_interfaces"
  "rclcpp"
)


# ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/local_planner.py
  scripts/global_planner.py
  scripts/multi_apple_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS move_arm
  DESTINATION lib/${PROJECT_NAME})




# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
# Install launch files.
install(DIRECTORY
  resources
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
