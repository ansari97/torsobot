# Install script for directory: /home/pi/torsobot/Code/Code/ros2_ws/src/torsobot_interfaces

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/pi/torsobot/Code/Code/ros2_ws/install/torsobot_interfaces")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/rosidl_interfaces" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/ament_cmake_index/share/ament_index/resource_index/rosidl_interfaces/torsobot_interfaces")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/msg" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/rosidl_generator_type_description/torsobot_interfaces/msg/TorsobotData.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/msg" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/rosidl_generator_type_description/torsobot_interfaces/msg/TorsobotState.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/srv" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/rosidl_generator_type_description/torsobot_interfaces/srv/RequestControlParams.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/torsobot_interfaces/torsobot_interfaces" TYPE DIRECTORY FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/rosidl_generator_c/torsobot_interfaces/" REGEX "/[^/]*\\.h$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/environment" TYPE FILE FILES "/home/pi/ros2_jazzy/build/ament_package/ament_package/template/environment_hook/library_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/environment" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/ament_cmake_environment_hooks/library_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_generator_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_generator_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_generator_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/libtorsobot_interfaces__rosidl_generator_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_generator_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_generator_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_generator_c.so"
         OLD_RPATH "/home/pi/ros2_jazzy/install/service_msgs/lib:/home/pi/ros2_jazzy/install/builtin_interfaces/lib:/home/pi/ros2_jazzy/install/rosidl_runtime_c/lib:/home/pi/ros2_jazzy/install/rcutils/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_generator_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/torsobot_interfaces/torsobot_interfaces" TYPE DIRECTORY FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/rosidl_typesupport_introspection_c/torsobot_interfaces/" REGEX "/[^/]*\\.h$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_introspection_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_introspection_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_introspection_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/libtorsobot_interfaces__rosidl_typesupport_introspection_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_introspection_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_introspection_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_introspection_c.so"
         OLD_RPATH "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces:/home/pi/ros2_jazzy/install/service_msgs/lib:/home/pi/ros2_jazzy/install/builtin_interfaces/lib:/home/pi/ros2_jazzy/install/rosidl_typesupport_introspection_c/lib:/home/pi/ros2_jazzy/install/rosidl_runtime_c/lib:/home/pi/ros2_jazzy/install/rcutils/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_introspection_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/torsobot_interfaces/torsobot_interfaces" TYPE DIRECTORY FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/rosidl_typesupport_fastrtps_c/torsobot_interfaces/" REGEX "/[^/]*\\.cpp$" EXCLUDE)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_fastrtps_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_fastrtps_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_fastrtps_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/libtorsobot_interfaces__rosidl_typesupport_fastrtps_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_fastrtps_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_fastrtps_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_fastrtps_c.so"
         OLD_RPATH "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces:/home/pi/ros2_jazzy/install/service_msgs/lib:/home/pi/ros2_jazzy/install/builtin_interfaces/lib:/home/pi/ros2_jazzy/install/rosidl_typesupport_fastrtps_cpp/lib:/home/pi/ros2_jazzy/install/rmw/lib:/home/pi/ros2_jazzy/install/rosidl_dynamic_typesupport/lib:/home/pi/ros2_jazzy/install/rosidl_typesupport_fastrtps_c/lib:/home/pi/ros2_jazzy/install/fastcdr/lib:/home/pi/ros2_jazzy/install/rosidl_runtime_c/lib:/home/pi/ros2_jazzy/install/rcutils/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_fastrtps_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/libtorsobot_interfaces__rosidl_typesupport_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_c.so"
         OLD_RPATH "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces:/home/pi/ros2_jazzy/install/service_msgs/lib:/home/pi/ros2_jazzy/install/builtin_interfaces/lib:/home/pi/ros2_jazzy/install/rosidl_typesupport_c/lib:/home/pi/ros2_jazzy/install/rcpputils/lib:/home/pi/ros2_jazzy/install/rosidl_runtime_c/lib:/home/pi/ros2_jazzy/install/rcutils/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/torsobot_interfaces/torsobot_interfaces" TYPE DIRECTORY FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/rosidl_generator_cpp/torsobot_interfaces/" REGEX "/[^/]*\\.hpp$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/torsobot_interfaces/torsobot_interfaces" TYPE DIRECTORY FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/rosidl_typesupport_introspection_cpp/torsobot_interfaces/" REGEX "/[^/]*\\.hpp$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_introspection_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_introspection_cpp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_introspection_cpp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/libtorsobot_interfaces__rosidl_typesupport_introspection_cpp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_introspection_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_introspection_cpp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_introspection_cpp.so"
         OLD_RPATH "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces:/home/pi/ros2_jazzy/install/service_msgs/lib:/home/pi/ros2_jazzy/install/builtin_interfaces/lib:/home/pi/ros2_jazzy/install/rosidl_typesupport_introspection_cpp/lib:/home/pi/ros2_jazzy/install/rosidl_typesupport_introspection_c/lib:/home/pi/ros2_jazzy/install/rosidl_runtime_c/lib:/home/pi/ros2_jazzy/install/rcutils/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_introspection_cpp.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/torsobot_interfaces/torsobot_interfaces" TYPE DIRECTORY FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/rosidl_typesupport_fastrtps_cpp/torsobot_interfaces/" REGEX "/[^/]*\\.cpp$" EXCLUDE)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_fastrtps_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_fastrtps_cpp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_fastrtps_cpp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/libtorsobot_interfaces__rosidl_typesupport_fastrtps_cpp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_fastrtps_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_fastrtps_cpp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_fastrtps_cpp.so"
         OLD_RPATH "/home/pi/ros2_jazzy/install/service_msgs/lib:/home/pi/ros2_jazzy/install/builtin_interfaces/lib:/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces:/home/pi/ros2_jazzy/install/rosidl_typesupport_fastrtps_cpp/lib:/home/pi/ros2_jazzy/install/fastcdr/lib:/home/pi/ros2_jazzy/install/rmw/lib:/home/pi/ros2_jazzy/install/rosidl_dynamic_typesupport/lib:/home/pi/ros2_jazzy/install/rosidl_runtime_c/lib:/home/pi/ros2_jazzy/install/rcutils/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_fastrtps_cpp.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_cpp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_cpp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/libtorsobot_interfaces__rosidl_typesupport_cpp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_cpp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_cpp.so"
         OLD_RPATH "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces:/home/pi/ros2_jazzy/install/service_msgs/lib:/home/pi/ros2_jazzy/install/builtin_interfaces/lib:/home/pi/ros2_jazzy/install/rosidl_typesupport_cpp/lib:/home/pi/ros2_jazzy/install/rosidl_typesupport_c/lib:/home/pi/ros2_jazzy/install/rcpputils/lib:/home/pi/ros2_jazzy/install/rosidl_runtime_c/lib:/home/pi/ros2_jazzy/install/rcutils/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_typesupport_cpp.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/environment" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/ament_cmake_environment_hooks/pythonpath.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/environment" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/ament_cmake_environment_hooks/pythonpath.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.11/site-packages/torsobot_interfaces-0.0.0-py3.11.egg-info" TYPE DIRECTORY FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/ament_cmake_python/torsobot_interfaces/torsobot_interfaces.egg-info/")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.11/site-packages/torsobot_interfaces" TYPE DIRECTORY FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/rosidl_generator_py/torsobot_interfaces/" REGEX "/[^/]*\\.pyc$" EXCLUDE REGEX "/\\_\\_pycache\\_\\_$" EXCLUDE)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(
        COMMAND
        "/usr/bin/python3" "-m" "compileall"
        "/home/pi/torsobot/Code/Code/ros2_ws/install/torsobot_interfaces/lib/python3.11/site-packages/torsobot_interfaces"
      )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.11/site-packages/torsobot_interfaces/torsobot_interfaces_s__rosidl_typesupport_introspection_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.11/site-packages/torsobot_interfaces/torsobot_interfaces_s__rosidl_typesupport_introspection_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.11/site-packages/torsobot_interfaces/torsobot_interfaces_s__rosidl_typesupport_introspection_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.11/site-packages/torsobot_interfaces" TYPE MODULE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/rosidl_generator_py/torsobot_interfaces/torsobot_interfaces_s__rosidl_typesupport_introspection_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.11/site-packages/torsobot_interfaces/torsobot_interfaces_s__rosidl_typesupport_introspection_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.11/site-packages/torsobot_interfaces/torsobot_interfaces_s__rosidl_typesupport_introspection_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.11/site-packages/torsobot_interfaces/torsobot_interfaces_s__rosidl_typesupport_introspection_c.so"
         OLD_RPATH "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces:/home/pi/ros2_jazzy/install/rmw/lib:/home/pi/ros2_jazzy/install/service_msgs/lib:/home/pi/ros2_jazzy/install/builtin_interfaces/lib:/home/pi/ros2_jazzy/install/rosidl_typesupport_fastrtps_c/lib:/home/pi/ros2_jazzy/install/rosidl_typesupport_introspection_cpp/lib:/home/pi/ros2_jazzy/install/rosidl_typesupport_fastrtps_cpp/lib:/home/pi/ros2_jazzy/install/fastcdr/lib:/home/pi/ros2_jazzy/install/rosidl_dynamic_typesupport/lib:/home/pi/ros2_jazzy/install/rosidl_typesupport_cpp/lib:/home/pi/ros2_jazzy/install/rosidl_typesupport_c/lib:/home/pi/ros2_jazzy/install/rcpputils/lib:/home/pi/ros2_jazzy/install/rosidl_typesupport_introspection_c/lib:/home/pi/ros2_jazzy/install/rosidl_runtime_c/lib:/home/pi/ros2_jazzy/install/rcutils/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.11/site-packages/torsobot_interfaces/torsobot_interfaces_s__rosidl_typesupport_introspection_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.11/site-packages/torsobot_interfaces/torsobot_interfaces_s__rosidl_typesupport_fastrtps_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.11/site-packages/torsobot_interfaces/torsobot_interfaces_s__rosidl_typesupport_fastrtps_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.11/site-packages/torsobot_interfaces/torsobot_interfaces_s__rosidl_typesupport_fastrtps_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.11/site-packages/torsobot_interfaces" TYPE MODULE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/rosidl_generator_py/torsobot_interfaces/torsobot_interfaces_s__rosidl_typesupport_fastrtps_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.11/site-packages/torsobot_interfaces/torsobot_interfaces_s__rosidl_typesupport_fastrtps_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.11/site-packages/torsobot_interfaces/torsobot_interfaces_s__rosidl_typesupport_fastrtps_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.11/site-packages/torsobot_interfaces/torsobot_interfaces_s__rosidl_typesupport_fastrtps_c.so"
         OLD_RPATH "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces:/home/pi/ros2_jazzy/install/rmw/lib:/home/pi/ros2_jazzy/install/service_msgs/lib:/home/pi/ros2_jazzy/install/builtin_interfaces/lib:/home/pi/ros2_jazzy/install/rosidl_typesupport_introspection_cpp/lib:/home/pi/ros2_jazzy/install/rosidl_typesupport_introspection_c/lib:/home/pi/ros2_jazzy/install/rosidl_typesupport_cpp/lib:/home/pi/ros2_jazzy/install/rosidl_typesupport_c/lib:/home/pi/ros2_jazzy/install/rcpputils/lib:/home/pi/ros2_jazzy/install/rosidl_typesupport_fastrtps_cpp/lib:/home/pi/ros2_jazzy/install/rosidl_dynamic_typesupport/lib:/home/pi/ros2_jazzy/install/rosidl_typesupport_fastrtps_c/lib:/home/pi/ros2_jazzy/install/fastcdr/lib:/home/pi/ros2_jazzy/install/rosidl_runtime_c/lib:/home/pi/ros2_jazzy/install/rcutils/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.11/site-packages/torsobot_interfaces/torsobot_interfaces_s__rosidl_typesupport_fastrtps_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.11/site-packages/torsobot_interfaces/torsobot_interfaces_s__rosidl_typesupport_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.11/site-packages/torsobot_interfaces/torsobot_interfaces_s__rosidl_typesupport_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.11/site-packages/torsobot_interfaces/torsobot_interfaces_s__rosidl_typesupport_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.11/site-packages/torsobot_interfaces" TYPE MODULE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/rosidl_generator_py/torsobot_interfaces/torsobot_interfaces_s__rosidl_typesupport_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.11/site-packages/torsobot_interfaces/torsobot_interfaces_s__rosidl_typesupport_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.11/site-packages/torsobot_interfaces/torsobot_interfaces_s__rosidl_typesupport_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.11/site-packages/torsobot_interfaces/torsobot_interfaces_s__rosidl_typesupport_c.so"
         OLD_RPATH "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces:/home/pi/ros2_jazzy/install/rmw/lib:/home/pi/ros2_jazzy/install/service_msgs/lib:/home/pi/ros2_jazzy/install/builtin_interfaces/lib:/home/pi/ros2_jazzy/install/rosidl_typesupport_fastrtps_c/lib:/home/pi/ros2_jazzy/install/rosidl_typesupport_introspection_cpp/lib:/home/pi/ros2_jazzy/install/rosidl_typesupport_introspection_c/lib:/home/pi/ros2_jazzy/install/rosidl_typesupport_fastrtps_cpp/lib:/home/pi/ros2_jazzy/install/fastcdr/lib:/home/pi/ros2_jazzy/install/rosidl_dynamic_typesupport/lib:/home/pi/ros2_jazzy/install/rosidl_typesupport_cpp/lib:/home/pi/ros2_jazzy/install/rosidl_typesupport_c/lib:/home/pi/ros2_jazzy/install/rcpputils/lib:/home/pi/ros2_jazzy/install/rosidl_runtime_c/lib:/home/pi/ros2_jazzy/install/rcutils/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.11/site-packages/torsobot_interfaces/torsobot_interfaces_s__rosidl_typesupport_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_generator_py.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_generator_py.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_generator_py.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/libtorsobot_interfaces__rosidl_generator_py.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_generator_py.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_generator_py.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_generator_py.so"
         OLD_RPATH "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces:/home/pi/ros2_jazzy/install/service_msgs/lib:/home/pi/ros2_jazzy/install/builtin_interfaces/lib:/home/pi/ros2_jazzy/install/rosidl_typesupport_fastrtps_c/lib:/home/pi/ros2_jazzy/install/rosidl_typesupport_introspection_cpp/lib:/home/pi/ros2_jazzy/install/rosidl_typesupport_introspection_c/lib:/home/pi/ros2_jazzy/install/rosidl_typesupport_fastrtps_cpp/lib:/home/pi/ros2_jazzy/install/fastcdr/lib:/home/pi/ros2_jazzy/install/rmw/lib:/home/pi/ros2_jazzy/install/rosidl_dynamic_typesupport/lib:/home/pi/ros2_jazzy/install/rosidl_typesupport_cpp/lib:/home/pi/ros2_jazzy/install/rosidl_typesupport_c/lib:/home/pi/ros2_jazzy/install/rosidl_runtime_c/lib:/home/pi/ros2_jazzy/install/rcpputils/lib:/home/pi/ros2_jazzy/install/rcutils/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtorsobot_interfaces__rosidl_generator_py.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/msg" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/rosidl_adapter/torsobot_interfaces/msg/TorsobotData.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/msg" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/rosidl_adapter/torsobot_interfaces/msg/TorsobotState.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/srv" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/rosidl_adapter/torsobot_interfaces/srv/RequestControlParams.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/msg" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/src/torsobot_interfaces/msg/TorsobotData.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/msg" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/src/torsobot_interfaces/msg/TorsobotState.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/srv" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/src/torsobot_interfaces/srv/RequestControlParams.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/torsobot_interfaces")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/torsobot_interfaces")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/environment" TYPE FILE FILES "/home/pi/ros2_jazzy/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/environment" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/environment" TYPE FILE FILES "/home/pi/ros2_jazzy/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/environment" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/ament_cmake_index/share/ament_index/resource_index/packages/torsobot_interfaces")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/export_torsobot_interfaces__rosidl_generator_cExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/export_torsobot_interfaces__rosidl_generator_cExport.cmake"
         "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/CMakeFiles/Export/9060e834e906ee75d1e6e8f2ad0f8fd8/export_torsobot_interfaces__rosidl_generator_cExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/export_torsobot_interfaces__rosidl_generator_cExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/export_torsobot_interfaces__rosidl_generator_cExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/CMakeFiles/Export/9060e834e906ee75d1e6e8f2ad0f8fd8/export_torsobot_interfaces__rosidl_generator_cExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/CMakeFiles/Export/9060e834e906ee75d1e6e8f2ad0f8fd8/export_torsobot_interfaces__rosidl_generator_cExport-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/torsobot_interfaces__rosidl_typesupport_introspection_cExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/torsobot_interfaces__rosidl_typesupport_introspection_cExport.cmake"
         "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/CMakeFiles/Export/9060e834e906ee75d1e6e8f2ad0f8fd8/torsobot_interfaces__rosidl_typesupport_introspection_cExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/torsobot_interfaces__rosidl_typesupport_introspection_cExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/torsobot_interfaces__rosidl_typesupport_introspection_cExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/CMakeFiles/Export/9060e834e906ee75d1e6e8f2ad0f8fd8/torsobot_interfaces__rosidl_typesupport_introspection_cExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/CMakeFiles/Export/9060e834e906ee75d1e6e8f2ad0f8fd8/torsobot_interfaces__rosidl_typesupport_introspection_cExport-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/export_torsobot_interfaces__rosidl_typesupport_fastrtps_cExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/export_torsobot_interfaces__rosidl_typesupport_fastrtps_cExport.cmake"
         "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/CMakeFiles/Export/9060e834e906ee75d1e6e8f2ad0f8fd8/export_torsobot_interfaces__rosidl_typesupport_fastrtps_cExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/export_torsobot_interfaces__rosidl_typesupport_fastrtps_cExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/export_torsobot_interfaces__rosidl_typesupport_fastrtps_cExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/CMakeFiles/Export/9060e834e906ee75d1e6e8f2ad0f8fd8/export_torsobot_interfaces__rosidl_typesupport_fastrtps_cExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/CMakeFiles/Export/9060e834e906ee75d1e6e8f2ad0f8fd8/export_torsobot_interfaces__rosidl_typesupport_fastrtps_cExport-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/torsobot_interfaces__rosidl_typesupport_cExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/torsobot_interfaces__rosidl_typesupport_cExport.cmake"
         "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/CMakeFiles/Export/9060e834e906ee75d1e6e8f2ad0f8fd8/torsobot_interfaces__rosidl_typesupport_cExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/torsobot_interfaces__rosidl_typesupport_cExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/torsobot_interfaces__rosidl_typesupport_cExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/CMakeFiles/Export/9060e834e906ee75d1e6e8f2ad0f8fd8/torsobot_interfaces__rosidl_typesupport_cExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/CMakeFiles/Export/9060e834e906ee75d1e6e8f2ad0f8fd8/torsobot_interfaces__rosidl_typesupport_cExport-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/export_torsobot_interfaces__rosidl_generator_cppExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/export_torsobot_interfaces__rosidl_generator_cppExport.cmake"
         "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/CMakeFiles/Export/9060e834e906ee75d1e6e8f2ad0f8fd8/export_torsobot_interfaces__rosidl_generator_cppExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/export_torsobot_interfaces__rosidl_generator_cppExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/export_torsobot_interfaces__rosidl_generator_cppExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/CMakeFiles/Export/9060e834e906ee75d1e6e8f2ad0f8fd8/export_torsobot_interfaces__rosidl_generator_cppExport.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/torsobot_interfaces__rosidl_typesupport_introspection_cppExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/torsobot_interfaces__rosidl_typesupport_introspection_cppExport.cmake"
         "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/CMakeFiles/Export/9060e834e906ee75d1e6e8f2ad0f8fd8/torsobot_interfaces__rosidl_typesupport_introspection_cppExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/torsobot_interfaces__rosidl_typesupport_introspection_cppExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/torsobot_interfaces__rosidl_typesupport_introspection_cppExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/CMakeFiles/Export/9060e834e906ee75d1e6e8f2ad0f8fd8/torsobot_interfaces__rosidl_typesupport_introspection_cppExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/CMakeFiles/Export/9060e834e906ee75d1e6e8f2ad0f8fd8/torsobot_interfaces__rosidl_typesupport_introspection_cppExport-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/export_torsobot_interfaces__rosidl_typesupport_fastrtps_cppExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/export_torsobot_interfaces__rosidl_typesupport_fastrtps_cppExport.cmake"
         "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/CMakeFiles/Export/9060e834e906ee75d1e6e8f2ad0f8fd8/export_torsobot_interfaces__rosidl_typesupport_fastrtps_cppExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/export_torsobot_interfaces__rosidl_typesupport_fastrtps_cppExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/export_torsobot_interfaces__rosidl_typesupport_fastrtps_cppExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/CMakeFiles/Export/9060e834e906ee75d1e6e8f2ad0f8fd8/export_torsobot_interfaces__rosidl_typesupport_fastrtps_cppExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/CMakeFiles/Export/9060e834e906ee75d1e6e8f2ad0f8fd8/export_torsobot_interfaces__rosidl_typesupport_fastrtps_cppExport-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/torsobot_interfaces__rosidl_typesupport_cppExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/torsobot_interfaces__rosidl_typesupport_cppExport.cmake"
         "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/CMakeFiles/Export/9060e834e906ee75d1e6e8f2ad0f8fd8/torsobot_interfaces__rosidl_typesupport_cppExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/torsobot_interfaces__rosidl_typesupport_cppExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/torsobot_interfaces__rosidl_typesupport_cppExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/CMakeFiles/Export/9060e834e906ee75d1e6e8f2ad0f8fd8/torsobot_interfaces__rosidl_typesupport_cppExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/CMakeFiles/Export/9060e834e906ee75d1e6e8f2ad0f8fd8/torsobot_interfaces__rosidl_typesupport_cppExport-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/export_torsobot_interfaces__rosidl_generator_pyExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/export_torsobot_interfaces__rosidl_generator_pyExport.cmake"
         "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/CMakeFiles/Export/9060e834e906ee75d1e6e8f2ad0f8fd8/export_torsobot_interfaces__rosidl_generator_pyExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/export_torsobot_interfaces__rosidl_generator_pyExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake/export_torsobot_interfaces__rosidl_generator_pyExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/CMakeFiles/Export/9060e834e906ee75d1e6e8f2ad0f8fd8/export_torsobot_interfaces__rosidl_generator_pyExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/CMakeFiles/Export/9060e834e906ee75d1e6e8f2ad0f8fd8/export_torsobot_interfaces__rosidl_generator_pyExport-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/rosidl_cmake/rosidl_cmake-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/ament_cmake_export_include_directories/ament_cmake_export_include_directories-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/ament_cmake_export_libraries/ament_cmake_export_libraries-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/ament_cmake_export_targets/ament_cmake_export_targets-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/rosidl_cmake/rosidl_cmake_export_typesupport_targets-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/rosidl_cmake/rosidl_cmake_export_typesupport_libraries-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces/cmake" TYPE FILE FILES
    "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/ament_cmake_core/torsobot_interfacesConfig.cmake"
    "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/ament_cmake_core/torsobot_interfacesConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/torsobot_interfaces" TYPE FILE FILES "/home/pi/torsobot/Code/Code/ros2_ws/src/torsobot_interfaces/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/torsobot_interfaces__py/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/pi/torsobot/Code/Code/ros2_ws/build/torsobot_interfaces/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
