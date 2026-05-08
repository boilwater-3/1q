# =============================================================================
# Vendored dependencies — 为每个第三方库创建与 find_package 同名的 CMake target。
# 无网络、无 Conan、无 vcpkg 时可独立构建，无需外部包管理器。
# =============================================================================

# 第三方库的编译产物统一输出到 vendor/ 子目录，与项目自身产物隔离。
# 多配置生成器下，目录级 CMAKE_ARCHIVE_OUTPUT_DIRECTORY_<CONFIG> 优先级高于
# target 级 ARCHIVE_OUTPUT_DIRECTORY，因此必须显式覆盖每个配置的输出路径。
function(_vendor_output_dir target)
  if(NOT TARGET ${target})
    return()
  endif()
  set(_base "${CMAKE_BINARY_DIR}/vendor")
  get_property(_isMultiConfig GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
  if(_isMultiConfig)
    foreach(_cfg ${CMAKE_CONFIGURATION_TYPES})
      string(TOUPPER ${_cfg} _cfg_upper)
      set_target_properties(${target} PROPERTIES
        ARCHIVE_OUTPUT_DIRECTORY_${_cfg_upper} "${_base}/${_cfg}/lib"
        LIBRARY_OUTPUT_DIRECTORY_${_cfg_upper} "${_base}/${_cfg}/lib"
        RUNTIME_OUTPUT_DIRECTORY_${_cfg_upper} "${_base}/${_cfg}/bin"
      )
    endforeach()
  else()
    set_target_properties(${target} PROPERTIES
      ARCHIVE_OUTPUT_DIRECTORY "${_base}/lib"
      LIBRARY_OUTPUT_DIRECTORY "${_base}/lib"
      RUNTIME_OUTPUT_DIRECTORY "${_base}/bin"
    )
  endif()
endfunction()

# -- Eigen3 (header-only) ---------------------------------------------------
add_library(eigen3_interface INTERFACE)
target_include_directories(eigen3_interface INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/eigen)
add_library(Eigen3::Eigen ALIAS eigen3_interface)

# -- Boost (header-only，仅用 pool + math/special_functions) ------------------
add_library(boost_interface INTERFACE)
target_include_directories(boost_interface INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/boost)
add_library(Boost::boost ALIAS boost_interface)

# -- nanoflann (header-only, 单文件) ------------------------------------------
add_library(nanoflann_interface INTERFACE)
target_include_directories(nanoflann_interface INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/nanoflann)
add_library(nanoflann::nanoflann ALIAS nanoflann_interface)

# -- fmt (保留源码；vendor 模式下 spdlog 使用自带 bundled fmt) -----------------
if(PROJECT_ENABLE_SPDLOG)
    set(FMT_INSTALL OFF CACHE BOOL "" FORCE)
    set(FMT_TEST OFF CACHE BOOL "" FORCE)
    set(FMT_DOC OFF CACHE BOOL "" FORCE)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/fmt ${CMAKE_BINARY_DIR}/third_party/fmt)
    _vendor_output_dir(fmt)

    # -- spdlog ------------------------------------------------------------------
    set(SPDLOG_FMT_EXTERNAL OFF CACHE BOOL "" FORCE)
    set(SPDLOG_INSTALL OFF CACHE BOOL "" FORCE)
    set(SPDLOG_BUILD_EXAMPLE OFF CACHE BOOL "" FORCE)
    set(SPDLOG_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/spdlog ${CMAKE_BINARY_DIR}/third_party/spdlog)
    _vendor_output_dir(spdlog)
endif()

# -- flatbuffers (仅构建运行时库，跳过 flatc / flathash / 测试) ----------------
# flatbuffers 1.12 的 cmake_minimum_required 较老，CMake 4.0+ 需此策略
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
set(FLATBUFFERS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(FLATBUFFERS_BUILD_FLATLIB ON CACHE BOOL "" FORCE)
set(FLATBUFFERS_BUILD_FLATC OFF CACHE BOOL "" FORCE)
set(FLATBUFFERS_BUILD_FLATHASH OFF CACHE BOOL "" FORCE)
set(FLATBUFFERS_INSTALL OFF CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/flatbuffers ${CMAKE_BINARY_DIR}/third_party/flatbuffers)
if(TARGET flatbuffers AND NOT TARGET flatbuffers::flatbuffers)
    add_library(flatbuffers::flatbuffers ALIAS flatbuffers)
endif()
_vendor_output_dir(flatbuffers)

# -- zlib (仅静态库，跳过共享库和示例) ----------------------------------------
# zlib 1.2.11 无条件构建 SHARED + STATIC + 示例，没有禁用选项。
# 通过 EXCLUDE_FROM_ALL 排除不需要的 target。
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/zlib ${CMAKE_BINARY_DIR}/third_party/zlib)
# 排除共享库和示例程序
if(TARGET zlib)
    set_target_properties(zlib PROPERTIES EXCLUDE_FROM_ALL ON EXCLUDE_FROM_DEFAULT_BUILD ON)
endif()
foreach(_t example minigzip example64 minigzip64)
    if(TARGET ${_t})
        set_target_properties(${_t} PROPERTIES EXCLUDE_FROM_ALL ON EXCLUDE_FROM_DEFAULT_BUILD ON)
    endif()
endforeach()
# ZLIB::ZLIB 指向静态库 zlibstatic，确保主库静态链接 zlib
if(NOT TARGET ZLIB::ZLIB)
    add_library(ZLIB::ZLIB ALIAS zlibstatic)
endif()
# zlib 1.2.11 uses directory-scope include_directories(), not target PUBLIC.
# Add the include path to the target so consumers can find zlib.h.
target_include_directories(zlibstatic
    SYSTEM PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/zlib
                  ${CMAKE_BINARY_DIR}/third_party/zlib)
_vendor_output_dir(zlibstatic)

# -- GTest (仅测试用，不构建可跳过) ------------------------------------------
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/googletest ${CMAKE_BINARY_DIR}/third_party/googletest)
# googletest 的 add_subdirectory 创建的是 gtest/gtest_main/gmock，上层期望 GTest:: 前缀
if(NOT TARGET GTest::gtest)
    add_library(GTest::gtest ALIAS gtest)
endif()
if(NOT TARGET GTest::gtest_main)
    add_library(GTest::gtest_main ALIAS gtest_main)
endif()
if(TARGET gmock AND NOT TARGET GTest::gmock)
    add_library(GTest::gmock ALIAS gmock)
endif()
foreach(_t gtest gtest_main gmock gmock_main)
    _vendor_output_dir(${_t})
endforeach()

# 需要合并进主库的编译型第三方 target（header-only 的不需要）
set(ONEQ_VENDOR_MERGE_TARGETS
    flatbuffers
    zlibstatic
    PARENT_SCOPE
)
