cmake_minimum_required(VERSION 3.8)
project(torsobot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find_library(PIGPIO_LIBRARY NAMES pigpio PATHS /usr/lib/libpigpio.so)
# find_path(PIGPIO_INCLUDE_DIR NAMES pigpio.h PATHS usr/include/pigpio.h)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(torsobot_interfaces REQUIRED)
# find_package(gpiodcxx REQUIRED) # For libgpiod C++ bindings 

# --- Manual Find for libgpiodcxx (if find_package fails) ---
find_path(GPIODCXX_INCLUDE_DIR NAMES gpiod.hpp
  PATHS /usr/include /usr/local/include)
find_library(GPIODCXX_LIBRARY NAMES gpiodcxx
  PATHS /usr/lib /usr/local/lib)

# It's good practice to add a check here to ensure they were found
if(NOT GPIODCXX_INCLUDE_DIR OR NOT GPIODCXX_LIBRARY)
  message(FATAL_ERROR "Could not find libgpiodcxx. Please ensure libgpiod-dev is installed correctly.")
endif()

add_executable(mcu_node src/mcu_node.cpp)
add_executable(i2c_manager_node src/i2c_manager_node.cpp)

ament_target_dependencies(mcu_node rclcpp torsobot_interfaces std_msgs)
ament_target_dependencies(i2c_manager_node rclcpp torsobot_interfaces std_msgs)

# Link against libgpiodcxx manually
target_include_directories(i2c_manager_node PRIVATE ${GPIODCXX_INCLUDE_DIR})
target_link_libraries(i2c_manager_node ${GPIODCXX_LIBRARY})

install(TARGETS
  mcu_node
  i2c_manager_node
  DESTINATION lib/${PROJECT_NAME})

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
