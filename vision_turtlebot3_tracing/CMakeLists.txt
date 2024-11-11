cmake_minimum_required(VERSION 3.8)
project(vision_turtlebot3_tracing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Setting C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Targeting include directory 
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

# Add executable
add_executable(main_control
  src/main.cpp
  src/main_control.cpp
)

ament_target_dependencies(main_control
  rclcpp 
  std_msgs
)

# Setting install target
install(TARGETS
  main_control
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
