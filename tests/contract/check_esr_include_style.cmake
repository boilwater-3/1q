if(NOT DEFINED SOURCE_DIR OR SOURCE_DIR STREQUAL "")
  message(FATAL_ERROR "SOURCE_DIR must be provided")
endif()

set(ESR_SRC_DIR "${SOURCE_DIR}/src/electronic_surveillance_radar")
set(PUBLIC_INCLUDE_ROOT "${SOURCE_DIR}/include/1q")
set(PRIVATE_INCLUDE_ROOT "${SOURCE_DIR}/src")

file(GLOB_RECURSE ESR_IMPL_FILES
     "${ESR_SRC_DIR}/*.h"
     "${ESR_SRC_DIR}/*.hpp"
     "${ESR_SRC_DIR}/*.cpp"
     "${ESR_SRC_DIR}/*.cc")

set(VIOLATIONS)

foreach(IMPL_FILE IN LISTS ESR_IMPL_FILES)
  file(STRINGS "${IMPL_FILE}" INCLUDE_LINES
       REGEX "^[ \t]*#include[ \t]+\"(1q/electronic_surveillance_radar/|electronic_surveillance_radar/)[^\"]+\"")
  foreach(INCLUDE_LINE IN LISTS INCLUDE_LINES)
    string(REGEX REPLACE "^[ \t]*#include[ \t]+\"([^\"]+)\".*$" "\\1"
           INCLUDE_PATH "${INCLUDE_LINE}")

    if(INCLUDE_PATH MATCHES "^1q/electronic_surveillance_radar/")
      string(REGEX REPLACE "^1q/" "" MAYBE_PRIVATE_RELATIVE "${INCLUDE_PATH}")
      if(EXISTS "${PRIVATE_INCLUDE_ROOT}/${MAYBE_PRIVATE_RELATIVE}")
        list(APPEND VIOLATIONS
             "${IMPL_FILE}: private header uses public prefix '${INCLUDE_PATH}'")
      endif()
    elseif(INCLUDE_PATH MATCHES "^electronic_surveillance_radar/")
      if(EXISTS "${PUBLIC_INCLUDE_ROOT}/${INCLUDE_PATH}"
         AND NOT EXISTS "${PRIVATE_INCLUDE_ROOT}/${INCLUDE_PATH}")
        list(APPEND VIOLATIONS
             "${IMPL_FILE}: public header should use '1q/' prefix for '${INCLUDE_PATH}'")
      endif()
    endif()
  endforeach()
endforeach()

if(VIOLATIONS)
  list(JOIN VIOLATIONS "\n" VIOLATION_TEXT)
  message(FATAL_ERROR
          "ESR include style check failed.\n"
          "Rules:\n"
          "1) Private headers under src/electronic_surveillance_radar should be included as electronic_surveillance_radar/...\n"
          "2) Public headers under include/1q/electronic_surveillance_radar should be included as 1q/electronic_surveillance_radar/...\n"
          "Violations:\n${VIOLATION_TEXT}")
endif()
