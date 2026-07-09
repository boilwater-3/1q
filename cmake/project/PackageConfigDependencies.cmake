# PackageConfig dependency block used by ProjectTemplateConfig.cmake.in.

set(ONEQ_CONFIG_FIND_DEPENDENCIES "")
if(PACKAGE_MANAGER STREQUAL "conan")
    string(APPEND ONEQ_CONFIG_FIND_DEPENDENCIES
        "find_dependency(Eigen3 REQUIRED CONFIG)\n"
        "find_dependency(Boost REQUIRED CONFIG)\n"
        "find_dependency(nanoflann REQUIRED CONFIG)\n"
        "find_dependency(flatbuffers REQUIRED CONFIG)\n")
    if(PROJECT_ENABLE_SPDLOG)
        string(APPEND ONEQ_CONFIG_FIND_DEPENDENCIES
            "find_dependency(spdlog REQUIRED CONFIG)\n")
    endif()
    string(APPEND ONEQ_CONFIG_FIND_DEPENDENCIES
        "find_dependency(ZLIB REQUIRED)\n")
endif()
