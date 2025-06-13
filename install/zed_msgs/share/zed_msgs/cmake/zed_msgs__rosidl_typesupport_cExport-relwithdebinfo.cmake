#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebinfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "zed_msgs::zed_msgs__rosidl_typesupport_c" for configuration "RelWithDebinfo"
set_property(TARGET zed_msgs::zed_msgs__rosidl_typesupport_c APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(zed_msgs::zed_msgs__rosidl_typesupport_c PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELWITHDEBINFO "rosidl_runtime_c::rosidl_runtime_c;rosidl_typesupport_c::rosidl_typesupport_c"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libzed_msgs__rosidl_typesupport_c.so"
  IMPORTED_SONAME_RELWITHDEBINFO "libzed_msgs__rosidl_typesupport_c.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS zed_msgs::zed_msgs__rosidl_typesupport_c )
list(APPEND _IMPORT_CHECK_FILES_FOR_zed_msgs::zed_msgs__rosidl_typesupport_c "${_IMPORT_PREFIX}/lib/libzed_msgs__rosidl_typesupport_c.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
