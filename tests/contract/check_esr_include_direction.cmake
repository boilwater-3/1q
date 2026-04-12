if(NOT DEFINED SOURCE_DIR OR SOURCE_DIR STREQUAL "")
  message(FATAL_ERROR "SOURCE_DIR must be provided")
endif()

set(ESR_SRC_DIR "${SOURCE_DIR}/src/electronic_surveillance_radar")
set(CORE_COMPOSITION_ROOT_FILE
    "${SOURCE_DIR}/src/electronic_surveillance_radar/session/EsrSessionCompositionRoot.cpp")

file(GLOB_RECURSE ESR_IMPL_FILES
     "${ESR_SRC_DIR}/*.h"
     "${ESR_SRC_DIR}/*.hpp"
     "${ESR_SRC_DIR}/*.cpp"
     "${ESR_SRC_DIR}/*.cc")

set(HARD_VIOLATIONS)
set(CORE_CONCRETE_DEP_WARNINGS)
set(CORE_COMPOSITION_ROOT_EXEMPTIONS)

foreach(IMPL_FILE IN LISTS ESR_IMPL_FILES)
  file(STRINGS "${IMPL_FILE}" INCLUDE_LINES
       REGEX "^[ \t]*#include[ \t]+\"electronic_surveillance_radar/[^\"]+\"")

  foreach(INCLUDE_LINE IN LISTS INCLUDE_LINES)
    string(REGEX REPLACE "^[ \t]*#include[ \t]+\"([^\"]+)\".*$" "\\1"
           INCLUDE_PATH "${INCLUDE_LINE}")

    if(IMPL_FILE MATCHES "/src/electronic_surveillance_radar/pipeline/")
      if(INCLUDE_PATH MATCHES "^electronic_surveillance_radar/(runtime|session)/")
        list(APPEND HARD_VIOLATIONS
             "${IMPL_FILE}: pipeline must not include '${INCLUDE_PATH}'")
      endif()
    endif()

    if(IMPL_FILE MATCHES "/src/electronic_surveillance_radar/intercept/")
      if(INCLUDE_PATH MATCHES "^electronic_surveillance_radar/(runtime|session|pipeline)/")
        list(APPEND HARD_VIOLATIONS
             "${IMPL_FILE}: intercept must not include '${INCLUDE_PATH}'")
      endif()
    endif()

    if(IMPL_FILE MATCHES "/src/electronic_surveillance_radar/environment/")
      if(INCLUDE_PATH MATCHES "^electronic_surveillance_radar/(intercept|pipeline|runtime|session|output)/")
        list(APPEND HARD_VIOLATIONS
             "${IMPL_FILE}: environment must not include '${INCLUDE_PATH}'")
      endif()
    endif()

    if(IMPL_FILE MATCHES "/src/electronic_surveillance_radar/(runtime|session)/")
      if(INCLUDE_PATH MATCHES "^electronic_surveillance_radar/(pipeline|environment|intercept|output)/")
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
          "ESR include direction composition-root exemptions.\n"
          "Allowed in current stage:\n${CORE_EXEMPTION_TEXT}")
endif()

if(CORE_CONCRETE_DEP_WARNINGS)
  list(REMOVE_DUPLICATES CORE_CONCRETE_DEP_WARNINGS)
  list(SORT CORE_CONCRETE_DEP_WARNINGS)
  list(JOIN CORE_CONCRETE_DEP_WARNINGS "\n" CORE_WARNING_TEXT)
  message(WARNING
          "ESR include direction core-abstraction baseline (warning only).\n"
          "Current mapping: core := runtime + session.\n"
          "Concrete dependencies detected (excluding composition-root exemptions):\n${CORE_WARNING_TEXT}")
endif()

if(HARD_VIOLATIONS)
  list(REMOVE_DUPLICATES HARD_VIOLATIONS)
  list(SORT HARD_VIOLATIONS)
  list(JOIN HARD_VIOLATIONS "\n" VIOLATION_TEXT)
  message(FATAL_ERROR
          "ESR include direction check failed.\n"
          "Hard rules:\n"
          "1) src/electronic_surveillance_radar/pipeline/** must not include electronic_surveillance_radar/runtime/** or electronic_surveillance_radar/session/**.\n"
          "2) src/electronic_surveillance_radar/intercept/** must not include electronic_surveillance_radar/runtime/**, electronic_surveillance_radar/session/**, or electronic_surveillance_radar/pipeline/**.\n"
          "3) src/electronic_surveillance_radar/environment/** must not include electronic_surveillance_radar/intercept/**, electronic_surveillance_radar/pipeline/**, electronic_surveillance_radar/runtime/**, electronic_surveillance_radar/session/**, or electronic_surveillance_radar/output/**.\n"
          "Soft rule (warning-only in current stage):\n"
          "- src/electronic_surveillance_radar/runtime/** and src/electronic_surveillance_radar/session/** should prefer extension interfaces over concrete impl headers.\n"
          "Violations:\n${VIOLATION_TEXT}")
endif()
