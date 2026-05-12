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

# -- flatbuffers (OBJECT library，直接编进主库) ----------------------------
add_library(flatbuffers_objects OBJECT
    ${CMAKE_CURRENT_SOURCE_DIR}/flatbuffers/src/idl_parser.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/flatbuffers/src/idl_gen_text.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/flatbuffers/src/reflection.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/flatbuffers/src/util.cpp
)
target_include_directories(flatbuffers_objects SYSTEM PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/flatbuffers/include
)
set_target_properties(flatbuffers_objects PROPERTIES
    CXX_STANDARD 11
    POSITION_INDEPENDENT_CODE ON
)
if(MSVC)
    target_compile_options(flatbuffers_objects PRIVATE /W0)
else()
    target_compile_options(flatbuffers_objects PRIVATE -w)
endif()
# INTERFACE 目标供 ProjectDependencies.cmake 的 target_link_libraries 使用
add_library(flatbuffers_interface INTERFACE)
target_link_libraries(flatbuffers_interface INTERFACE flatbuffers_objects)
add_library(flatbuffers::flatbuffers ALIAS flatbuffers_interface)

# -- zlib (OBJECT library，直接编进主库) ------------------------------------
# zconf.h 由 zlib 的 CMakeLists.txt 通过 configure_file 生成，这里需要手动生成。
set(ZLIB_ZCONF_DIR "${CMAKE_BINARY_DIR}/third_party/zlib_objects")
file(MAKE_DIRECTORY "${ZLIB_ZCONF_DIR}")
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/zlib/zconf.h.in
    ${ZLIB_ZCONF_DIR}/zconf.h
    @ONLY
)
add_library(zlib_objects OBJECT
    ${CMAKE_CURRENT_SOURCE_DIR}/zlib/adler32.c
    ${CMAKE_CURRENT_SOURCE_DIR}/zlib/compress.c
    ${CMAKE_CURRENT_SOURCE_DIR}/zlib/crc32.c
    ${CMAKE_CURRENT_SOURCE_DIR}/zlib/deflate.c
    ${CMAKE_CURRENT_SOURCE_DIR}/zlib/gzclose.c
    ${CMAKE_CURRENT_SOURCE_DIR}/zlib/gzlib.c
    ${CMAKE_CURRENT_SOURCE_DIR}/zlib/gzread.c
    ${CMAKE_CURRENT_SOURCE_DIR}/zlib/gzwrite.c
    ${CMAKE_CURRENT_SOURCE_DIR}/zlib/inflate.c
    ${CMAKE_CURRENT_SOURCE_DIR}/zlib/infback.c
    ${CMAKE_CURRENT_SOURCE_DIR}/zlib/inftrees.c
    ${CMAKE_CURRENT_SOURCE_DIR}/zlib/inffast.c
    ${CMAKE_CURRENT_SOURCE_DIR}/zlib/trees.c
    ${CMAKE_CURRENT_SOURCE_DIR}/zlib/uncompr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/zlib/zutil.c
)
target_include_directories(zlib_objects SYSTEM PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/zlib
    ${ZLIB_ZCONF_DIR}
)
if(MSVC)
    target_compile_definitions(zlib_objects PRIVATE
        _CRT_SECURE_NO_DEPRECATE _CRT_NONSTDC_NO_DEPRECATE)
    target_compile_options(zlib_objects PRIVATE /W0)
else()
    target_compile_options(zlib_objects PRIVATE -w)
endif()
set_target_properties(zlib_objects PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)
# INTERFACE 目标供 ProjectDependencies.cmake 的 target_link_libraries 使用
add_library(zlib_interface INTERFACE)
target_link_libraries(zlib_interface INTERFACE zlib_objects)
add_library(ZLIB::ZLIB ALIAS zlib_interface)

# -- GTest (仅测试用，不构建可跳过) ------------------------------------------
if(ENABLE_TESTING)
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
endif()

# 需要直接编进主库的编译型第三方 OBJECT target
set(ONEQ_VENDOR_OBJECT_TARGETS
    flatbuffers_objects
    zlib_objects
    PARENT_SCOPE
)
