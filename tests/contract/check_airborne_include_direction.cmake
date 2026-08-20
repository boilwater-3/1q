if(NOT DEFINED SOURCE_DIR OR SOURCE_DIR STREQUAL "")
  message(FATAL_ERROR "SOURCE_DIR must be provided")
endif()

include("${CMAKE_CURRENT_LIST_DIR}/ReadSourceLines.cmake")

set(AIRBORNE_SRC_DIR "${SOURCE_DIR}/src/airborne_radar")
set(CORE_COMPOSITION_ROOT_FILE
    "${SOURCE_DIR}/src/airborne_radar/session/ArSessionCompositionRoot.cpp")

file(GLOB_RECURSE AIRBORNE_IMPL_FILES
     "${AIRBORNE_SRC_DIR}/*.h"
     "${AIRBORNE_SRC_DIR}/*.hpp"
     "${AIRBORNE_SRC_DIR}/*.cpp"
     "${AIRBORNE_SRC_DIR}/*.cc")

set(HARD_VIOLATIONS)
set(CORE_CONCRETE_DEP_WARNINGS)
set(CORE_COMPOSITION_ROOT_EXEMPTIONS)
set(CORE_GRAY_ZONE_WARNINGS)

foreach(IMPL_FILE IN LISTS AIRBORNE_IMPL_FILES)
  oneq_read_source_lines(INCLUDE_LINES "${IMPL_FILE}")
  list(FILTER INCLUDE_LINES INCLUDE REGEX
       "^[ \t]*#include[ \t]+\"airborne_radar/[^\"]+\"")

  foreach(INCLUDE_LINE IN LISTS INCLUDE_LINES)
    string(REGEX REPLACE "^[ \t]*#include[ \t]+\"([^\"]+)\".*$" "\\1"
           INCLUDE_PATH "${INCLUDE_LINE}")

    if(IMPL_FILE MATCHES "/src/airborne_radar/signal/")
      if(INCLUDE_PATH MATCHES "^airborne_radar/(runtime|session)/")
        list(APPEND HARD_VIOLATIONS
             "${IMPL_FILE}: signal must not include '${INCLUDE_PATH}'")
      endif()
    endif()

    if(IMPL_FILE MATCHES "/src/airborne_radar/environment/")
      if(INCLUDE_PATH MATCHES "^airborne_radar/(decision|signal)/")
        list(APPEND HARD_VIOLATIONS
             "${IMPL_FILE}: environment must not include '${INCLUDE_PATH}'")
      endif()
    endif()

    if(IMPL_FILE MATCHES "/src/airborne_radar/(runtime|session)/")
      if(INCLUDE_PATH MATCHES "^airborne_radar/(decision|signal|environment)/")
        if(IMPL_FILE STREQUAL "${CORE_COMPOSITION_ROOT_FILE}")
          list(APPEND CORE_COMPOSITION_ROOT_EXEMPTIONS
               "${IMPL_FILE}: composition root exemption '${INCLUDE_PATH}'")
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
          "Airborne include direction composition-root exemptions.\n"
          "Allowed in current stage:\n${CORE_EXEMPTION_TEXT}")
endif()

if(CORE_GRAY_ZONE_WARNINGS)
  list(REMOVE_DUPLICATES CORE_GRAY_ZONE_WARNINGS)
  list(SORT CORE_GRAY_ZONE_WARNINGS)
  list(JOIN CORE_GRAY_ZONE_WARNINGS "\n" CORE_GRAY_ZONE_TEXT)
  message(WARNING
          "Airborne include direction gray-zone baseline (warning only).\n"
          "Track and retire these dependencies in follow-up refactor:\n${CORE_GRAY_ZONE_TEXT}")
endif()

if(CORE_CONCRETE_DEP_WARNINGS)
  list(REMOVE_DUPLICATES CORE_CONCRETE_DEP_WARNINGS)
  list(SORT CORE_CONCRETE_DEP_WARNINGS)
  list(JOIN CORE_CONCRETE_DEP_WARNINGS "\n" CORE_WARNING_TEXT)
  message(WARNING
          "Airborne include direction core-abstraction baseline (warning only).\n"
          "Current mapping: core := runtime + session.\n"
          "Concrete dependencies detected (excluding composition-root exemptions and gray-zone list):\n${CORE_WARNING_TEXT}")
endif()

if(HARD_VIOLATIONS)
  list(REMOVE_DUPLICATES HARD_VIOLATIONS)
  list(SORT HARD_VIOLATIONS)
  list(JOIN HARD_VIOLATIONS "\n" VIOLATION_TEXT)
  message(FATAL_ERROR
          "Airborne include direction check failed.\n"
          "Hard rules:\n"
          "1) src/airborne_radar/signal/** must not include airborne_radar/runtime/** or airborne_radar/session/**.\n"
          "2) src/airborne_radar/environment/** must not include airborne_radar/decision/** or airborne_radar/signal/**.\n"
          "Soft rule (warning-only in current stage):\n"
          "- src/airborne_radar/runtime/** and src/airborne_radar/session/** should prefer extension interfaces over concrete impl headers.\n"
          "Violations:\n${VIOLATION_TEXT}")
endif()
