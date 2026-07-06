#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "hiwin_libmodbus::hiwin_libmodbus" for configuration "Release"
set_property(TARGET hiwin_libmodbus::hiwin_libmodbus APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(hiwin_libmodbus::hiwin_libmodbus PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libhiwin_libmodbus.so"
  IMPORTED_SONAME_RELEASE "libhiwin_libmodbus.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS hiwin_libmodbus::hiwin_libmodbus )
list(APPEND _IMPORT_CHECK_FILES_FOR_hiwin_libmodbus::hiwin_libmodbus "${_IMPORT_PREFIX}/lib/libhiwin_libmodbus.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
