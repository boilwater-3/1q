# =============================================================================
# Vendored dependencies — 为每个第三方库创建与 find_package 同名的 CMake target。
# 无网络、无 Conan、无 vcpkg 时可独立构建，无需外部包管理器。
# =============================================================================

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

    # -- spdlog ------------------------------------------------------------------
    set(SPDLOG_FMT_EXTERNAL OFF CACHE BOOL "" FORCE)
    set(SPDLOG_INSTALL OFF CACHE BOOL "" FORCE)
    set(SPDLOG_BUILD_EXAMPLE OFF CACHE BOOL "" FORCE)
    set(SPDLOG_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/spdlog ${CMAKE_BINARY_DIR}/third_party/spdlog)
endif()

# -- flatbuffers (仅构建运行时库，跳过 flatc 和测试) -------------------------
# flatbuffers 1.12 的 cmake_minimum_required 较老，CMake 4.0+ 需此策略
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
set(FLATBUFFERS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(FLATBUFFERS_BUILD_FLATLIB ON CACHE BOOL "" FORCE)
set(FLATBUFFERS_BUILD_FLATC OFF CACHE BOOL "" FORCE)
set(FLATBUFFERS_INSTALL OFF CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/flatbuffers ${CMAKE_BINARY_DIR}/third_party/flatbuffers)
if(TARGET flatbuffers AND NOT TARGET flatbuffers::flatbuffers)
    add_library(flatbuffers::flatbuffers ALIAS flatbuffers)
endif()

# -- zlib ---------------------------------------------------------------------
set(ZLIB_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/zlib ${CMAKE_BINARY_DIR}/third_party/zlib)
# vendored zlib 创建的是 "zlib" target，但上层代码期望 ZLIB::ZLIB
if(NOT TARGET ZLIB::ZLIB)
    add_library(ZLIB::ZLIB ALIAS zlib)
endif()
# zlib 1.2.11 uses directory-scope include_directories(), not target PUBLIC.
# Add the include path to the target so consumers can find zlib.h.
target_include_directories(zlib
    SYSTEM PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/zlib
                  ${CMAKE_CURRENT_BINARY_DIR}/third_party/zlib)

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
