# Keep the public header boundary, installation rule and public API whitelist
# aligned without parsing arbitrary module CMakeLists.txt text.

cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED SOURCE_DIR)
    message(FATAL_ERROR "SOURCE_DIR is required")
endif()

set(PUBLIC_INCLUDE_DIR "${SOURCE_DIR}/include/1q")
file(GLOB_RECURSE DISK_HEADERS_REL
    LIST_DIRECTORIES FALSE
    RELATIVE "${SOURCE_DIR}/include"
    "${PUBLIC_INCLUDE_DIR}/*.h"
    "${PUBLIC_INCLUDE_DIR}/*.hpp")
list(TRANSFORM DISK_HEADERS_REL PREPEND "include/")
list(SORT DISK_HEADERS_REL)

# ProjectInstall must mirror the whole guarded public tree. This makes every
# disk header installable by construction and removes duplicated per-module
# install manifests.
set(PROJECT_INSTALL_FILE "${SOURCE_DIR}/cmake/project/ProjectInstall.cmake")
file(READ "${PROJECT_INSTALL_FILE}" PROJECT_INSTALL_CONTENT)
foreach(required_fragment IN ITEMS
        "install(DIRECTORY"
        "include/1q/"
        "FILES_MATCHING"
        "PATTERN \"*.h\""
        "PATTERN \"*.hpp\"")
    string(FIND "${PROJECT_INSTALL_CONTENT}" "${required_fragment}" fragment_index)
    if(fragment_index EQUAL -1)
        message(FATAL_ERROR
            "[install-manifest] ProjectInstall.cmake no longer mirrors the guarded public tree; missing '${required_fragment}'.")
    endif()
endforeach()

# Extract the independently maintained public API whitelist and compare it to
# the disk boundary. The installed header set is DISK_HEADERS_REL by the mirror
# rule above.
set(BOUNDARY_SCRIPT "${SOURCE_DIR}/tests/contract/check_public_api_boundary.cmake")
file(STRINGS "${BOUNDARY_SCRIPT}" boundary_lines)
set(BOUNDARY_HEADERS "")
foreach(line IN LISTS boundary_lines)
    string(REGEX MATCHALL "\"[A-Za-z][A-Za-z0-9_/]+\\.(hpp|h)\"" path_matches "${line}")
    foreach(path_match IN LISTS path_matches)
        string(REGEX REPLACE "^\"" "" path_match "${path_match}")
        string(REGEX REPLACE "\"$" "" path_match "${path_match}")
        list(APPEND BOUNDARY_HEADERS "include/1q/${path_match}")
    endforeach()
endforeach()
list(REMOVE_DUPLICATES BOUNDARY_HEADERS)
list(SORT BOUNDARY_HEADERS)

set(MISSING_FROM_BOUNDARY "")
foreach(header IN LISTS DISK_HEADERS_REL)
    list(FIND BOUNDARY_HEADERS "${header}" header_index)
    if(header_index EQUAL -1)
        list(APPEND MISSING_FROM_BOUNDARY "${header}")
    endif()
endforeach()

set(MISSING_FROM_DISK "")
foreach(header IN LISTS BOUNDARY_HEADERS)
    list(FIND DISK_HEADERS_REL "${header}" header_index)
    if(header_index EQUAL -1)
        list(APPEND MISSING_FROM_DISK "${header}")
    endif()
endforeach()

if(MISSING_FROM_BOUNDARY OR MISSING_FROM_DISK)
    message(FATAL_ERROR
        "[install-manifest] public disk boundary and whitelist diverged.\n"
        "Missing from boundary: ${MISSING_FROM_BOUNDARY}\n"
        "Missing from disk: ${MISSING_FROM_DISK}")
endif()

list(LENGTH DISK_HEADERS_REL PUBLIC_HEADER_COUNT)
message(STATUS
    "[install-manifest] passed: ${PUBLIC_HEADER_COUNT} guarded public headers are mirrored by ProjectInstall")
