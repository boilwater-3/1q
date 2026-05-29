# 第三方依赖加载与链接
# Conan / vcpkg 通过 find_package 获取；vendor 模式通过 add_subdirectory 构建内置依赖。

# -- JSBSim (macOS 通过 Conan 获取；Windows 从 third_party 源码构建) --
# ONEQ_JSBSIM_FROM_SOURCE: 即使 Conan 可用，也从 third_party 源码构建 JSBSim。
# 用于 LLDB 源码级调试：Debug 构建会在 JSBSim 代码中生成完整 DWARF 符号，
# 可在 FGFDMExec、FGFCS、FGTrim 等内部设断点直接观察仿真状态。
option(ONEQ_JSBSIM_FROM_SOURCE "Build JSBSim from third_party source (enables source-level debugging)" OFF)

if(PACKAGE_MANAGER STREQUAL "conan" AND NOT ONEQ_JSBSIM_FROM_SOURCE)
  # macOS 开发：使用 conancenter 预编译的 jsbsim/1.3.1
  set(ONEQ_JSBSIM_BINARY_SOURCE "conan:jsbsim/1.3.1")
  find_package(jsbsim CONFIG REQUIRED)
  add_library(JSBSim::JSBSim ALIAS jsbsim::jsbsim)
elseif(PACKAGE_MANAGER STREQUAL "none" OR ONEQ_JSBSIM_FROM_SOURCE)
  # Windows/VS2015 生产编译：从 third_party 源码构建为共享库（C++11 兼容，LGPL 合规）。
  set(ONEQ_JSBSIM_BINARY_SOURCE "vendor:third_party/jsbsim")
  set(_oneq_prev_build_shared_libs ${BUILD_SHARED_LIBS})
  set(BUILD_SHARED_LIBS ON CACHE BOOL "Build JSBSim as shared library" FORCE)
  set(BUILD_DOCS OFF CACHE BOOL "" FORCE)

  # 1Q 项目通过 add_compile_options(-fvisibility=hidden) 隐藏符号，
  # 但 C++11 版 JSBSim 无导出宏。必须在 JSBSim 编译前移除该选项，
  # 编译后恢复。使用 COMPILE_OPTIONS 列表操作确保其他选项不受影响。
  get_directory_property(_oneq_compile_opts COMPILE_OPTIONS)
  set(_oneq_filtered_opts)
  foreach(_opt IN LISTS _oneq_compile_opts)
    if(NOT _opt STREQUAL "-fvisibility=hidden")
      list(APPEND _oneq_filtered_opts "${_opt}")
    endif()
  endforeach()
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "${_oneq_filtered_opts}")
  unset(_oneq_filtered_opts)

  add_subdirectory(${CMAKE_SOURCE_DIR}/third_party/jsbsim ${CMAKE_BINARY_DIR}/third_party/jsbsim)

  # 恢复可见性选项
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "${_oneq_compile_opts}")
  unset(_oneq_compile_opts)
  set(BUILD_SHARED_LIBS ${_oneq_prev_build_shared_libs} CACHE BOOL "" FORCE)
  unset(_oneq_prev_build_shared_libs)

  # JSBSim 使用 include_directories() 而非 target_include_directories()，
  # 需显式将 include 路径附加到 INTERFACE 目标以供消费者使用。
  add_library(JSBSim_interface INTERFACE)
  target_link_libraries(JSBSim_interface INTERFACE libJSBSim)
  target_include_directories(JSBSim_interface INTERFACE
      ${CMAKE_SOURCE_DIR}/third_party/jsbsim/src)
  add_library(JSBSim::JSBSim ALIAS JSBSim_interface)
endif()
set(ONEQ_JSBSIM_BINARY_SOURCE "${ONEQ_JSBSIM_BINARY_SOURCE}"
    CACHE INTERNAL "Resolved JSBSim binary source")
set(ONEQ_JSBSIM_DATA_ROOT_DIR "${CMAKE_SOURCE_DIR}/third_party/jsbsim"
    CACHE INTERNAL "Resolved JSBSim aircraft data root")
set(ONEQ_JSBSIM_DATA_SOURCE "vendor:third_party/jsbsim"
    CACHE INTERNAL "Resolved JSBSim aircraft data source")
message(STATUS "JSBSim binary source: ${ONEQ_JSBSIM_BINARY_SOURCE}")
message(STATUS "JSBSim data source: ${ONEQ_JSBSIM_DATA_SOURCE} (${ONEQ_JSBSIM_DATA_ROOT_DIR})")

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

# -- 主库目标需要全部第三方依赖用于最终链接 --
target_link_libraries(${PROJECT_CORE_TARGET} PRIVATE ${ONEQ_LINK_DEPENDENCIES})

# -- OBJECT 目标依赖（header-only 传递） --
# AR/ESR 的 core 层因 composition root 需引用 engine 头文件，必须持有全部算法库路径。
# EOS 确认无数学库传递依赖，因此 engine/core 均不链接 Eigen/Boost/nanoflann。
# flatbuffers/zlib 因 replay codec 文件散布于各域，统一提供。
target_link_libraries(airborne_engine PRIVATE ${ONEQ_LINK_DEPENDENCIES})
target_link_libraries(airborne_core PRIVATE ${ONEQ_LINK_DEPENDENCIES})
target_link_libraries(esr_engine PRIVATE ${ONEQ_LINK_DEPENDENCIES})
target_link_libraries(esr_core PRIVATE ${ONEQ_LINK_DEPENDENCIES})
target_link_libraries(eos_engine PRIVATE flatbuffers::flatbuffers)
target_link_libraries(eos_core PRIVATE flatbuffers::flatbuffers)

# flight_dynamic 模块依赖 JSBSim 飞行动力学引擎。
target_link_libraries(fd_engine PRIVATE JSBSim::JSBSim)
if(TARGET fd_core)
  target_link_libraries(fd_core PRIVATE JSBSim::JSBSim)
endif()
target_link_libraries(${PROJECT_CORE_TARGET} PRIVATE JSBSim::JSBSim)

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

# -- find_dependency 块（供 PackageConfig 模板注入）--
# 与 conanfile.py requirements() 保持一致：Eigen3, Boost, nanoflann, flatbuffers, ZLIB,
# 非 Windows 平台额外需要 spdlog。Vendor 模式下留空，消费者无需安装任何第三方依赖。
set(ONEQ_CONFIG_FIND_DEPENDENCIES "")
if(PACKAGE_MANAGER STREQUAL "conan" OR PACKAGE_MANAGER STREQUAL "vcpkg")
    string(APPEND ONEQ_CONFIG_FIND_DEPENDENCIES
        "find_dependency(Eigen3 REQUIRED CONFIG)\n"
        "find_dependency(Boost REQUIRED CONFIG)\n"
        "find_dependency(nanoflann REQUIRED CONFIG)\n"
        "find_dependency(flatbuffers REQUIRED CONFIG)\n"
    )
    if(PROJECT_ENABLE_SPDLOG)
        string(APPEND ONEQ_CONFIG_FIND_DEPENDENCIES
            "find_dependency(spdlog REQUIRED CONFIG)\n"
        )
    endif()
    string(APPEND ONEQ_CONFIG_FIND_DEPENDENCIES
        "find_dependency(ZLIB REQUIRED)\n"
    )
endif()
