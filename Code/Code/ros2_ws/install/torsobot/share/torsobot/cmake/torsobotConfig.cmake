# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_torsobot_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED torsobot_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(torsobot_FOUND FALSE)
  elseif(NOT torsobot_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(torsobot_FOUND FALSE)
  endif()
  return()
endif()
set(_torsobot_CONFIG_INCLUDED TRUE)

# output package information
if(NOT torsobot_FIND_QUIETLY)
  message(STATUS "Found torsobot: 0.0.0 (${torsobot_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'torsobot' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT torsobot_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(torsobot_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${torsobot_DIR}/${_extra}")
endforeach()
