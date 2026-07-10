# Conan package discovery and exported package-level dependency list.

# PACKAGE_MANAGER is validated by ProjectOptions.cmake before this entry point.
if(WIN32)
    set(PROJECT_ENABLE_SPDLOG OFF)
else()
    set(PROJECT_ENABLE_SPDLOG ON)
endif()
find_package(Eigen3 CONFIG REQUIRED)
find_package(Boost CONFIG REQUIRED)
find_package(nanoflann CONFIG REQUIRED)
find_package(flatbuffers CONFIG REQUIRED)
if(PROJECT_ENABLE_SPDLOG)
    find_package(spdlog CONFIG REQUIRED)
    if(TARGET spdlog::spdlog)
        set(PROJECT_SPDLOG_TARGET spdlog::spdlog)
    elseif(TARGET spdlog::spdlog_header_only)
        message(FATAL_ERROR "spdlog header-only target is not allowed in this configuration")
    else()
        message(FATAL_ERROR "spdlog target is not available")
    endif()
endif()
find_package(ZLIB REQUIRED)

if(PROJECT_CXX_STANDARD GREATER_EQUAL 17)
    find_package(HighFive CONFIG REQUIRED)
    set(ONEQ_ENABLE_HDF5_OUTPUT ON)
    message(STATUS "SAR HDF5 output: ENABLED (HighFive found)")
else()
    set(ONEQ_ENABLE_HDF5_OUTPUT OFF)
    message(STATUS "SAR HDF5 output: disabled (requires C++17, current: C++${PROJECT_CXX_STANDARD})")
endif()

set(ONEQ_HAVE_ZLIB ON)
set(ONEQ_LINK_DEPENDENCIES
    Eigen3::Eigen
    Boost::boost
    nanoflann::nanoflann
    flatbuffers::flatbuffers)
