#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "cartographer_rviz::cartographer_rviz" for configuration "Release"
set_property(TARGET cartographer_rviz::cartographer_rviz APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(cartographer_rviz::cartographer_rviz PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libcartographer_rviz.so"
  IMPORTED_SONAME_RELEASE "libcartographer_rviz.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS cartographer_rviz::cartographer_rviz )
list(APPEND _IMPORT_CHECK_FILES_FOR_cartographer_rviz::cartographer_rviz "${_IMPORT_PREFIX}/lib/libcartographer_rviz.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
