#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebinfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rviz_dynamic_slam_plugins::rviz_dynamic_slam_plugins" for configuration "RelWithDebinfo"
set_property(TARGET rviz_dynamic_slam_plugins::rviz_dynamic_slam_plugins APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(rviz_dynamic_slam_plugins::rviz_dynamic_slam_plugins PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELWITHDEBINFO "resource_retriever::resource_retriever"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/librviz_dynamic_slam_plugins.so"
  IMPORTED_SONAME_RELWITHDEBINFO "librviz_dynamic_slam_plugins.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS rviz_dynamic_slam_plugins::rviz_dynamic_slam_plugins )
list(APPEND _IMPORT_CHECK_FILES_FOR_rviz_dynamic_slam_plugins::rviz_dynamic_slam_plugins "${_IMPORT_PREFIX}/lib/librviz_dynamic_slam_plugins.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
