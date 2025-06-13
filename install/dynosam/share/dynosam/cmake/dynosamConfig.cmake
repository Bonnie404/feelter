# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_dynosam_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED dynosam_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(dynosam_FOUND FALSE)
  elseif(NOT dynosam_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(dynosam_FOUND FALSE)
  endif()
  return()
endif()
set(_dynosam_CONFIG_INCLUDED TRUE)

# output package information
if(NOT dynosam_FIND_QUIETLY)
  message(STATUS "Found dynosam: 0.0.1 (${dynosam_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'dynosam' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${dynosam_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(dynosam_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_libraries-extras.cmake;ament_cmake_export_dependencies-extras.cmake;ament_cmake_export_include_directories-extras.cmake")
foreach(_extra ${_extras})
  include("${dynosam_DIR}/${_extra}")
endforeach()
