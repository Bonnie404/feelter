#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebinfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "dynamic_slam_interfaces::dynamic_slam_interfaces__rosidl_generator_py" for configuration "RelWithDebinfo"
set_property(TARGET dynamic_slam_interfaces::dynamic_slam_interfaces__rosidl_generator_py APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(dynamic_slam_interfaces::dynamic_slam_interfaces__rosidl_generator_py PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libdynamic_slam_interfaces__rosidl_generator_py.so"
  IMPORTED_SONAME_RELWITHDEBINFO "libdynamic_slam_interfaces__rosidl_generator_py.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS dynamic_slam_interfaces::dynamic_slam_interfaces__rosidl_generator_py )
list(APPEND _IMPORT_CHECK_FILES_FOR_dynamic_slam_interfaces::dynamic_slam_interfaces__rosidl_generator_py "${_IMPORT_PREFIX}/lib/libdynamic_slam_interfaces__rosidl_generator_py.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
