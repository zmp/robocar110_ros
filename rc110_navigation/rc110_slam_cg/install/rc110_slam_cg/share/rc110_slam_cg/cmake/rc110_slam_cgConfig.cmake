# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_rc110_slam_cg_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED rc110_slam_cg_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(rc110_slam_cg_FOUND FALSE)
  elseif(NOT rc110_slam_cg_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(rc110_slam_cg_FOUND FALSE)
  endif()
  return()
endif()
set(_rc110_slam_cg_CONFIG_INCLUDED TRUE)

# output package information
if(NOT rc110_slam_cg_FIND_QUIETLY)
  message(STATUS "Found rc110_slam_cg: 2.0.0 (${rc110_slam_cg_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'rc110_slam_cg' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${rc110_slam_cg_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(rc110_slam_cg_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${rc110_slam_cg_DIR}/${_extra}")
endforeach()
