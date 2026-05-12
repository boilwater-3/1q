if(NOT DEFINED SOURCE_DIR OR SOURCE_DIR STREQUAL "")
  message(FATAL_ERROR "SOURCE_DIR must be provided")
endif()

set(EOS_SRC_DIR "${SOURCE_DIR}/src/electro_optical_sensor")
set(PUBLIC_INCLUDE_ROOT "${SOURCE_DIR}/include/1q")
set(PRIVATE_INCLUDE_ROOT "${SOURCE_DIR}/src")

file(GLOB_RECURSE EOS_IMPL_FILES
     "${EOS_SRC_DIR}/*.h"
     "${EOS_SRC_DIR}/*.hpp"
     "${EOS_SRC_DIR}/*.cpp"
     "${EOS_SRC_DIR}/*.cc")

set(VIOLATIONS)

foreach(IMPL_FILE IN LISTS EOS_IMPL_FILES)
  file(STRINGS "${IMPL_FILE}" INCLUDE_LINES
       REGEX "^[ \t]*#include[ \t]+\"(1q/electro_optical_sensor/|electro_optical_sensor/)[^\"]+\"")
  foreach(INCLUDE_LINE IN LISTS INCLUDE_LINES)
    string(REGEX REPLACE "^[ \t]*#include[ \t]+\"([^\"]+)\".*$" "\\1"
           INCLUDE_PATH "${INCLUDE_LINE}")

    if(INCLUDE_PATH MATCHES "^1q/electro_optical_sensor/")
      string(REGEX REPLACE "^1q/" "" RELATIVE_PATH "${INCLUDE_PATH}")
      if(EXISTS "${PRIVATE_INCLUDE_ROOT}/${RELATIVE_PATH}")
        list(APPEND VIOLATIONS
             "${IMPL_FILE}: private header should use 'electro_optical_sensor/' for '${INCLUDE_PATH}'")
      endif()
    elseif(INCLUDE_PATH MATCHES "^electro_optical_sensor/")
      if(EXISTS "${PUBLIC_INCLUDE_ROOT}/${INCLUDE_PATH}")
        list(APPEND VIOLATIONS
             "${IMPL_FILE}: public header should use '1q/electro_optical_sensor/' for '${INCLUDE_PATH}'")
      endif()
    endif()
  endforeach()
endforeach()

if(VIOLATIONS)
  list(JOIN VIOLATIONS "\n" VIOLATION_TEXT)
  message(FATAL_ERROR
          "EOS include style check failed.\n"
          "Rules:\n"
          "1) Private headers under src/electro_optical_sensor should be included as electro_optical_sensor/...\n"
          "2) Public headers under include/1q/electro_optical_sensor should be included as 1q/electro_optical_sensor/...\n"
          "Violations:\n${VIOLATION_TEXT}")
endif()
