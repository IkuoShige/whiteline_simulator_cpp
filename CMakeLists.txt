cmake_minimum_required(VERSION 3.8)
project(whiteline_simulator_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# Include directories
include_directories(include)

# Add Preprocessor library
add_library(preprocessor src/preprocessor.cpp)
ament_target_dependencies(preprocessor rclcpp)

# Add executable for whiteline simulator node
add_executable(whiteline_simulator_node src/whiteline_simulator_node.cpp)
target_link_libraries(whiteline_simulator_node preprocessor)
ament_target_dependencies(whiteline_simulator_node rclcpp std_msgs geometry_msgs nav_msgs tf2 tf2_ros visualization_msgs)

# Add executable for whiteline listener node
add_executable(whiteline_listener src/whiteline_listener.cpp)
target_link_libraries(whiteline_listener preprocessor)
ament_target_dependencies(whiteline_listener rclcpp std_msgs geometry_msgs)

# Add executable for soccer field generator
add_executable(soccer_field_generator src/soccer_field_generator.cpp)
target_link_libraries(soccer_field_generator preprocessor)
ament_target_dependencies(soccer_field_generator rclcpp)

# Install executables
install(TARGETS
  whiteline_simulator_node
  whiteline_listener
  soccer_field_generator
  DESTINATION lib/${PROJECT_NAME})

# Install library
install(TARGETS
  preprocessor
  DESTINATION lib/${PROJECT_NAME})

# Install header files
install(DIRECTORY include/
  DESTINATION include/)

# Install data files
install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data/)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/)

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
