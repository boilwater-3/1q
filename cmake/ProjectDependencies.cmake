# 第三方依赖加载与链接
# Conan / vcpkg 通过 find_package 获取；vendor 模式通过 add_subdirectory 构建内置依赖。

if(PACKAGE_MANAGER STREQUAL "conan" OR PACKAGE_MANAGER STREQUAL "vcpkg")
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
    if(WIN32)
        set(PROJECT_ENABLE_SPDLOG OFF)
    else()
        set(PROJECT_ENABLE_SPDLOG ON)
    endif()
    find_package(ZLIB QUIET)
    if(NOT ZLIB_FOUND)
        set(ONEQ_VENDOR_ZLIB ON)
    endif()
    add_subdirectory(${CMAKE_SOURCE_DIR}/third_party ${CMAKE_BINARY_DIR}/third_party)
endif()
set(ONEQ_HAVE_ZLIB ON)

set(ONEQ_LINK_DEPENDENCIES
    Eigen3::Eigen
    Boost::boost
    nanoflann::nanoflann
)
list(APPEND ONEQ_LINK_DEPENDENCIES flatbuffers::flatbuffers)

# Vendor dependencies are non-IMPORTED targets built in-tree via add_subdirectory.
# install(EXPORT) requires all referenced targets to be resolvable; wrapping with
# BUILD_INTERFACE confines them to build-time only.  This is safe for static builds
# because compiled code is already archived into the static library.
if(PACKAGE_MANAGER STREQUAL "none" AND ENABLE_INSTALL)
    set(_oneq_export_safe_deps)
    foreach(_dep IN LISTS ONEQ_LINK_DEPENDENCIES)
        list(APPEND _oneq_export_safe_deps "$<BUILD_INTERFACE:${_dep}>")
    endforeach()
    set(ONEQ_LINK_DEPENDENCIES ${_oneq_export_safe_deps})
    unset(_oneq_export_safe_deps)
    unset(_dep)
endif()

foreach(ONEQ_BUILD_TARGET IN ITEMS
    ${PROJECT_CORE_TARGET}
    ${ONEQ_OBJECT_TARGETS}
)
    target_link_libraries(${ONEQ_BUILD_TARGET} PRIVATE ${ONEQ_LINK_DEPENDENCIES})
endforeach()
unset(ONEQ_BUILD_TARGET)

# Conan's VS multi-config generation may only attach header-only include dirs
# to Release. Mirror those include dirs onto this target when available so
# Debug/RelWithDebInfo/MinSizeRel can compile against the same headers.
set(_oneq_conan_include_vars
    eigen_INCLUDE_DIRS_RELEASE
    boost_INCLUDE_DIRS_RELEASE
    nanoflann_INCLUDE_DIRS_RELEASE
    flatbuffers_INCLUDE_DIRS_RELEASE
    zlib_INCLUDE_DIRS_RELEASE
)
foreach(_oneq_conan_include_var IN LISTS _oneq_conan_include_vars)
    if(DEFINED ${_oneq_conan_include_var})
        foreach(ONEQ_BUILD_TARGET IN ITEMS
            ${PROJECT_CORE_TARGET}
            ${ONEQ_OBJECT_TARGETS}
        )
            target_include_directories(${ONEQ_BUILD_TARGET} PRIVATE ${${_oneq_conan_include_var}})
        endforeach()
    endif()
endforeach()
unset(_oneq_conan_include_var)
unset(_oneq_conan_include_vars)
unset(ONEQ_BUILD_TARGET)

# -- spdlog --
if(PROJECT_ENABLE_SPDLOG)
    if(TARGET spdlog::spdlog)
        set(PROJECT_SPDLOG_TARGET spdlog::spdlog)
    elseif(TARGET spdlog::spdlog_header_only)
        message(FATAL_ERROR "spdlog header-only target is not allowed in this configuration")
    else()
        message(FATAL_ERROR "spdlog target is not available")
    endif()
    if(PACKAGE_MANAGER STREQUAL "none" AND ENABLE_INSTALL)
        set(PROJECT_SPDLOG_TARGET "$<BUILD_INTERFACE:${PROJECT_SPDLOG_TARGET}>")
    endif()
    foreach(ONEQ_BUILD_TARGET IN ITEMS
        ${PROJECT_CORE_TARGET}
        ${ONEQ_OBJECT_TARGETS}
    )
        target_link_libraries(${ONEQ_BUILD_TARGET} PRIVATE ${PROJECT_SPDLOG_TARGET})
    endforeach()
    unset(ONEQ_BUILD_TARGET)
endif()

# -- compile definitions for dependency features --
foreach(ONEQ_BUILD_TARGET IN ITEMS
    ${PROJECT_CORE_TARGET}
    ${ONEQ_OBJECT_TARGETS}
)
    target_compile_definitions(${ONEQ_BUILD_TARGET}
        PRIVATE PROJECT_LOG_BACKEND_SPDLOG=$<BOOL:${PROJECT_ENABLE_SPDLOG}>
                ONEQ_HAVE_ZLIB=$<BOOL:${ONEQ_HAVE_ZLIB}>
    )
endforeach()
unset(ONEQ_BUILD_TARGET)

# -- zlib --
if(ONEQ_HAVE_ZLIB)
    if(PACKAGE_MANAGER STREQUAL "none" AND ENABLE_INSTALL)
        set(_oneq_zlib_dep "$<BUILD_INTERFACE:ZLIB::ZLIB>")
    else()
        set(_oneq_zlib_dep ZLIB::ZLIB)
    endif()
    foreach(ONEQ_BUILD_TARGET IN ITEMS
        ${PROJECT_CORE_TARGET}
        ${ONEQ_OBJECT_TARGETS}
    )
        target_link_libraries(${ONEQ_BUILD_TARGET} PRIVATE ${_oneq_zlib_dep})
    endforeach()
    unset(_oneq_zlib_dep)
    unset(ONEQ_BUILD_TARGET)
endif()
