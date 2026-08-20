if(NOT DEFINED SOURCE_DIR OR SOURCE_DIR STREQUAL "")
  message(FATAL_ERROR "SOURCE_DIR must be provided")
endif()

include("${CMAKE_CURRENT_LIST_DIR}/ReadSourceLines.cmake")

set(PUBLIC_HEADER_DIRS
    "${SOURCE_DIR}/include/1q/airborne_radar"
    "${SOURCE_DIR}/include/1q/electro_optical_sensor"
    "${SOURCE_DIR}/include/1q/electronic_surveillance_radar")

set(VIOLATIONS)

foreach(PUBLIC_HEADER_DIR IN LISTS PUBLIC_HEADER_DIRS)
  file(GLOB_RECURSE PUBLIC_HEADERS
       "${PUBLIC_HEADER_DIR}/*.h"
       "${PUBLIC_HEADER_DIR}/*.hpp")

  foreach(PUBLIC_HEADER IN LISTS PUBLIC_HEADERS)
    oneq_read_source_lines(INCLUDE_LINES "${PUBLIC_HEADER}")
    list(FILTER INCLUDE_LINES INCLUDE REGEX
         "^[ \t]*#include[ \t]+[<\"][^>\"]+[>\"]")
    foreach(INCLUDE_LINE IN LISTS INCLUDE_LINES)
      string(REGEX REPLACE "^[ \t]*#include[ \t]+([<\"])([^>\"]+)[>\"]"
                           "\\1;\\2" INCLUDE_META "${INCLUDE_LINE}")
      list(GET INCLUDE_META 0 INCLUDE_DELIM)
      list(GET INCLUDE_META 1 INCLUDE_PATH)

      if(INCLUDE_DELIM STREQUAL "\"")
        if(NOT INCLUDE_PATH MATCHES "^1q/")
          list(APPEND VIOLATIONS
               "${PUBLIC_HEADER}: non-public quoted include '${INCLUDE_PATH}'")
        endif()
      else()
        # Public headers only allow standard-library-style angle includes such as
        # <cstdint> / <vector>. Any nested path (<foo/bar>) or extension-like
        # include (<foo.hpp>) is treated as external dependency exposure.
        if(NOT INCLUDE_PATH MATCHES "^[A-Za-z0-9_]+$")
          list(APPEND VIOLATIONS
               "${PUBLIC_HEADER}: external angle include '${INCLUDE_PATH}'")
        endif()
      endif()
    endforeach()
  endforeach()
endforeach()

if(VIOLATIONS)
  list(JOIN VIOLATIONS "\n" VIOLATION_TEXT)
  message(FATAL_ERROR
          "Public header external dependency isolation check failed.\n"
          "Rules:\n"
          "1) Public headers must include project headers only via \"1q/...\".\n"
          "2) Public headers must not expose third-party or system path includes; angle includes\n"
          "   are limited to single-token standard-library style headers (e.g. <vector>, <cstdint>).\n"
          "Violations:\n${VIOLATION_TEXT}")
endif()
