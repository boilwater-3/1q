if(NOT DEFINED SOURCE_DIR OR SOURCE_DIR STREQUAL "")
  message(FATAL_ERROR "SOURCE_DIR must be provided")
endif()

set(EOS_SRC_DIR "${SOURCE_DIR}/src/electro_optical_sensor")
set(CORE_COMPOSITION_ROOT_FILE
    "${SOURCE_DIR}/src/electro_optical_sensor/session/EosSessionCompositionRoot.cpp")

file(GLOB_RECURSE EOS_IMPL_FILES
     "${EOS_SRC_DIR}/*.h"
     "${EOS_SRC_DIR}/*.hpp"
     "${EOS_SRC_DIR}/*.cpp"
     "${EOS_SRC_DIR}/*.cc")

set(HARD_VIOLATIONS)
set(CORE_CONCRETE_DEP_WARNINGS)
set(CORE_COMPOSITION_ROOT_EXEMPTIONS)

foreach(IMPL_FILE IN LISTS EOS_IMPL_FILES)
  file(STRINGS "${IMPL_FILE}" INCLUDE_LINES
       REGEX "^[ \t]*#include[ \t]+\"electro_optical_sensor/[^\"]+\"")

  foreach(INCLUDE_LINE IN LISTS INCLUDE_LINES)
    string(REGEX REPLACE "^[ \t]*#include[ \t]+\"([^\"]+)\".*$" "\\1"
           INCLUDE_PATH "${INCLUDE_LINE}")

    if(IMPL_FILE MATCHES "/src/electro_optical_sensor/(foundation|signal)/")
      if(INCLUDE_PATH MATCHES "^electro_optical_sensor/(runtime|session)/")
        list(APPEND HARD_VIOLATIONS
             "${IMPL_FILE}: engine must not include '${INCLUDE_PATH}'")
      endif()
    endif()

    if(IMPL_FILE MATCHES "/src/electro_optical_sensor/environment/")
      if(INCLUDE_PATH MATCHES "^electro_optical_sensor/(foundation|signal)/")
        list(APPEND HARD_VIOLATIONS
             "${IMPL_FILE}: environment must not include '${INCLUDE_PATH}'")
      endif()
    endif()

    if(IMPL_FILE MATCHES "/src/electro_optical_sensor/(runtime|session)/")
      if(INCLUDE_PATH MATCHES "^electro_optical_sensor/(foundation|signal|environment)/")
        if(IMPL_FILE STREQUAL "${CORE_COMPOSITION_ROOT_FILE}")
          list(APPEND CORE_COMPOSITION_ROOT_EXEMPTIONS
               "${IMPL_FILE}: composition-root exemption '${INCLUDE_PATH}'")
        else()
          list(APPEND CORE_CONCRETE_DEP_WARNINGS
               "${IMPL_FILE}: core concrete dependency '${INCLUDE_PATH}'")
        endif()
      endif()
    endif()
  endforeach()
endforeach()

if(CORE_COMPOSITION_ROOT_EXEMPTIONS)
  list(REMOVE_DUPLICATES CORE_COMPOSITION_ROOT_EXEMPTIONS)
  list(SORT CORE_COMPOSITION_ROOT_EXEMPTIONS)
  list(JOIN CORE_COMPOSITION_ROOT_EXEMPTIONS "\n" CORE_EXEMPTION_TEXT)
  message(STATUS
          "EOS include direction composition-root exemptions.\n"
          "Allowed in current stage:\n${CORE_EXEMPTION_TEXT}")
endif()

if(CORE_CONCRETE_DEP_WARNINGS)
  list(REMOVE_DUPLICATES CORE_CONCRETE_DEP_WARNINGS)
  list(SORT CORE_CONCRETE_DEP_WARNINGS)
  list(JOIN CORE_CONCRETE_DEP_WARNINGS "\n" CORE_WARNING_TEXT)
  message(WARNING
          "EOS include direction core-abstraction baseline (warning only).\n"
          "Current mapping: core := runtime + session.\n"
          "Concrete dependencies detected (excluding composition-root exemptions):\n${CORE_WARNING_TEXT}")
endif()

if(HARD_VIOLATIONS)
  list(REMOVE_DUPLICATES HARD_VIOLATIONS)
  list(SORT HARD_VIOLATIONS)
  list(JOIN HARD_VIOLATIONS "\n" VIOLATION_TEXT)
  message(FATAL_ERROR
          "EOS include direction check failed.\n"
          "Hard rules:\n"
          "1) src/electro_optical_sensor/foundation/** and src/electro_optical_sensor/signal/** must not include electro_optical_sensor/runtime/** or electro_optical_sensor/session/**.\n"
          "2) src/electro_optical_sensor/environment/** must not include electro_optical_sensor/foundation/** or electro_optical_sensor/signal/**.\n"
          "Soft rule (warning-only in current stage):\n"
          "- src/electro_optical_sensor/runtime/** and src/electro_optical_sensor/session/** should prefer extension interfaces over concrete impl headers.\n"
          "Violations:\n${VIOLATION_TEXT}")
endif()
