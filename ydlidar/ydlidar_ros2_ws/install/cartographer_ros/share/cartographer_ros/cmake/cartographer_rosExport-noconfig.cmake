#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "cartographer_ros::cartographer_ros" for configuration ""
set_property(TARGET cartographer_ros::cartographer_ros APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(cartographer_ros::cartographer_ros PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_NOCONFIG "pcl_common;std_msgs::std_msgs__rosidl_generator_c;std_msgs::std_msgs__rosidl_typesupport_fastrtps_c;std_msgs::std_msgs__rosidl_typesupport_fastrtps_cpp;std_msgs::std_msgs__rosidl_typesupport_introspection_c;std_msgs::std_msgs__rosidl_typesupport_c;std_msgs::std_msgs__rosidl_typesupport_introspection_cpp;std_msgs::std_msgs__rosidl_typesupport_cpp;std_msgs::std_msgs__rosidl_generator_py;tf2::tf2;tf2_msgs::tf2_msgs__rosidl_generator_c;tf2_msgs::tf2_msgs__rosidl_typesupport_fastrtps_c;tf2_msgs::tf2_msgs__rosidl_typesupport_introspection_c;tf2_msgs::tf2_msgs__rosidl_typesupport_c;tf2_msgs::tf2_msgs__rosidl_typesupport_fastrtps_cpp;tf2_msgs::tf2_msgs__rosidl_typesupport_introspection_cpp;tf2_msgs::tf2_msgs__rosidl_typesupport_cpp;tf2_msgs::tf2_msgs__rosidl_generator_py;urdf::urdf"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libcartographer_ros.so"
  IMPORTED_SONAME_NOCONFIG "libcartographer_ros.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS cartographer_ros::cartographer_ros )
list(APPEND _IMPORT_CHECK_FILES_FOR_cartographer_ros::cartographer_ros "${_IMPORT_PREFIX}/lib/libcartographer_ros.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
