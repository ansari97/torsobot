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
find_package(yaml-cpp REQUIRED)
# find_package(rosbag2_cpp REQUIRED)
# find_package(rosbag2_storage REQUIRED)
# find_package(rosbag2_transport REQUIRED)

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
add_executable(data_logger_node src/data_logger_node.cpp)
add_executable(serial_test_node src/serial_test_node.cpp)
# add_executable(i2c_manager_node src/i2c_manager_node.cpp)

ament_target_dependencies(serial_test_node
  rclcpp
  torsobot_interfaces
  std_msgs)

ament_target_dependencies(mcu_node
  rclcpp
  torsobot_interfaces
  std_msgs)
# torsobot_interfaces_rosidl_typesupport_fastrtps_cpp
# rosbag2_cpp
# rosbag2_storage
# rosbag2_transport)

target_include_directories(mcu_node PRIVATE ${GPIODCXX_INCLUDE_DIR})
target_link_libraries(mcu_node ${GPIODCXX_LIBRARY})
# target_link_libraries(mcu_node PRIVATE yaml-cpp::yaml-cpp)

target_include_directories(mcu_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(data_logger_node yaml-cpp::yaml-cpp)

target_include_directories(serial_test_node PRIVATE ${GPIODCXX_INCLUDE_DIR})
target_link_libraries(serial_test_node ${GPIODCXX_LIBRARY})

target_include_directories(data_logger_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)



ament_target_dependencies(data_logger_node
  rclcpp
  torsobot_interfaces
  std_msgs)

# ament_target_dependencies(i2c_manager_node rclcpp torsobot_interfaces std_msgs)

# Link against libgpiodcxx manually
# target_include_directories(i2c_manager_node PRIVATE ${GPIODCXX_INCLUDE_DIR})
# target_link_libraries(i2c_manager_node ${GPIODCXX_LIBRARY})

install(TARGETS
  mcu_node
  data_logger_node
  serial_test_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

  # Find all launch files in the launch directory
file(GLOB LAUNCH_FILES launch/*.py)

# Install the launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

# # Install the config files
# install(
#   DIRECTORY config/
#   DESTINATION share/${PROJECT_NAME}
# )

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
