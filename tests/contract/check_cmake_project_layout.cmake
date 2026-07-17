# Keep the physical project-layer layout aligned with the approved CMake plan.

cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED SOURCE_DIR)
    message(FATAL_ERROR "SOURCE_DIR is required")
endif()
# 规范化为绝对路径：file(GLOB_RECURSE ... RELATIVE <base>) 在 -P 脚本模式下，
# 当 base 为相对路径时返回空列表（误报全部 missing）。ctest 经 ContractGuards.cmake
# 传入的 ${CMAKE_SOURCE_DIR} 已是绝对路径，此调用对绝对输入幂等，仅兜底手动 -DSOURCE_DIR=. 调用。
get_filename_component(SOURCE_DIR "${SOURCE_DIR}" ABSOLUTE)

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
    "dependencies/VendorPackages.cmake")
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
