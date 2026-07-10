# JSBSim provider resolution.

option(ONEQ_JSBSIM_FROM_SOURCE "Build JSBSim from third_party source (enables source-level debugging)" OFF)
set(ONEQ_JSBSIM_PREBUILT_ROOT_DIR "" CACHE PATH
    "Use a prebuilt JSBSim tree instead of building/finding JSBSim")

if(ONEQ_JSBSIM_PREBUILT_ROOT_DIR)
    set(ONEQ_JSBSIM_BINARY_SOURCE "prebuilt:${ONEQ_JSBSIM_PREBUILT_ROOT_DIR}")
    find_library(ONEQ_JSBSIM_PREBUILT_LIBRARY
        NAMES JSBSim libJSBSim
        PATHS "${ONEQ_JSBSIM_PREBUILT_ROOT_DIR}/lib"
        NO_DEFAULT_PATH)
    if(NOT ONEQ_JSBSIM_PREBUILT_LIBRARY)
        message(FATAL_ERROR
            "ONEQ_JSBSIM_PREBUILT_ROOT_DIR does not contain lib/libJSBSim: "
            "${ONEQ_JSBSIM_PREBUILT_ROOT_DIR}")
    endif()
    add_library(JSBSim_prebuilt SHARED IMPORTED)
    set_target_properties(JSBSim_prebuilt PROPERTIES
        IMPORTED_LOCATION "${ONEQ_JSBSIM_PREBUILT_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/third_party/jsbsim/src")
    add_library(JSBSim::JSBSim ALIAS JSBSim_prebuilt)
elseif(ONEQ_JSBSIM_FROM_SOURCE)
    set(ONEQ_JSBSIM_BINARY_SOURCE "vendor:third_party/jsbsim")
    set(_oneq_prev_build_shared_libs ${BUILD_SHARED_LIBS})
    set(BUILD_SHARED_LIBS ON CACHE BOOL "Build JSBSim as shared library" FORCE)
    set(BUILD_DOCS OFF CACHE BOOL "" FORCE)

    add_subdirectory(${CMAKE_SOURCE_DIR}/third_party/jsbsim ${CMAKE_BINARY_DIR}/third_party/jsbsim)

    set(BUILD_SHARED_LIBS ${_oneq_prev_build_shared_libs} CACHE BOOL "" FORCE)
    unset(_oneq_prev_build_shared_libs)

    add_library(JSBSim_interface INTERFACE)
    target_link_libraries(JSBSim_interface INTERFACE libJSBSim)
    target_include_directories(JSBSim_interface INTERFACE
        ${CMAKE_SOURCE_DIR}/third_party/jsbsim/src)
    add_library(JSBSim::JSBSim ALIAS JSBSim_interface)
else()
    set(ONEQ_JSBSIM_BINARY_SOURCE "conan:jsbsim/1.3.1")
    find_package(jsbsim CONFIG REQUIRED)
    add_library(JSBSim::JSBSim ALIAS jsbsim::jsbsim)
endif()

set(ONEQ_JSBSIM_BINARY_SOURCE "${ONEQ_JSBSIM_BINARY_SOURCE}"
    CACHE INTERNAL "Resolved JSBSim binary source")
set(ONEQ_JSBSIM_DATA_ROOT_DIR "${CMAKE_SOURCE_DIR}/third_party/jsbsim"
    CACHE INTERNAL "Resolved JSBSim aircraft data root")
set(ONEQ_JSBSIM_DATA_SOURCE "vendor:third_party/jsbsim"
    CACHE INTERNAL "Resolved JSBSim aircraft data source")
message(STATUS "JSBSim binary source: ${ONEQ_JSBSIM_BINARY_SOURCE}")
message(STATUS "JSBSim data source: ${ONEQ_JSBSIM_DATA_SOURCE} (${ONEQ_JSBSIM_DATA_ROOT_DIR})")
