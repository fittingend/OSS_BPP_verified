#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "lanelet2_traffic_rules" for configuration ""
set_property(TARGET lanelet2_traffic_rules APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(lanelet2_traffic_rules PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/liblanelet2_traffic_rules.so"
  IMPORTED_SONAME_NOCONFIG "liblanelet2_traffic_rules.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS lanelet2_traffic_rules )
list(APPEND _IMPORT_CHECK_FILES_FOR_lanelet2_traffic_rules "${_IMPORT_PREFIX}/lib/liblanelet2_traffic_rules.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
