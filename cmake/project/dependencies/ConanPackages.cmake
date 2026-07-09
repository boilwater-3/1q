# Conan package discovery and exported package-level dependency list.

if(PACKAGE_MANAGER STREQUAL "conan")
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
    endif()
    find_package(ZLIB REQUIRED)
else()
    message(FATAL_ERROR
        "PACKAGE_MANAGER=none is unsupported because third_party is not tracked. "
        "Use PACKAGE_MANAGER=conan.")
endif()

set(ONEQ_HAVE_ZLIB ON)
set(ONEQ_LINK_DEPENDENCIES
    Eigen3::Eigen
    Boost::boost
    nanoflann::nanoflann
    flatbuffers::flatbuffers)
