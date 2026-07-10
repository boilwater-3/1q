# Keep the physical project-layer layout aligned with the approved CMake plan.

cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED SOURCE_DIR)
    message(FATAL_ERROR "SOURCE_DIR is required")
endif()

set(PROJECT_CMAKE_DIR "${SOURCE_DIR}/cmake/project")
file(GLOB_RECURSE ACTUAL_PROJECT_FILES RELATIVE "${PROJECT_CMAKE_DIR}"
    "${PROJECT_CMAKE_DIR}/*.cmake"
    "${PROJECT_CMAKE_DIR}/*.cmake.in")
list(SORT ACTUAL_PROJECT_FILES)

set(EXPECTED_PROJECT_FILES
    "ProjectDependencies.cmake"
    "ProjectInstall.cmake"
    "ProjectOptions.cmake"
    "ProjectSetup.cmake"
    "ProjectSummary.cmake"
    "ProjectTargets.cmake"
    "codegen/FlatBuffers.cmake"
    "dependencies/ConanPackages.cmake"
    "dependencies/JsbsimProvider.cmake"
    "legacy/Vs2015SourceNormalization.cmake")
list(SORT EXPECTED_PROJECT_FILES)

set(LAYOUT_VIOLATIONS "")
foreach(expected_file IN LISTS EXPECTED_PROJECT_FILES)
    if(NOT expected_file IN_LIST ACTUAL_PROJECT_FILES)
        list(APPEND LAYOUT_VIOLATIONS "missing: cmake/project/${expected_file}")
    endif()
endforeach()
foreach(actual_file IN LISTS ACTUAL_PROJECT_FILES)
    if(NOT actual_file IN_LIST EXPECTED_PROJECT_FILES)
        list(APPEND LAYOUT_VIOLATIONS "unexpected: cmake/project/${actual_file}")
    endif()
endforeach()

if(LAYOUT_VIOLATIONS)
    string(REPLACE ";" "\n  " LAYOUT_VIOLATIONS_TEXT "${LAYOUT_VIOLATIONS}")
    message(FATAL_ERROR
        "CMake project layout differs from the approved structure:\n  ${LAYOUT_VIOLATIONS_TEXT}")
endif()

message(STATUS "[cmake-project-layout] approved project layout is intact")
