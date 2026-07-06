# 项目顶层 CMakeLists.txt
# 本文件是项目的主入口，定义项目基础配置并协调各个模块

cmake_minimum_required(VERSION 3.16...3.30)

if(DEFINED CMAKE_TOOLCHAIN_FILE)
    get_filename_component(_requested_toolchain "${CMAKE_TOOLCHAIN_FILE}" ABSOLUTE)
    get_filename_component(_bootstrap_toolchain
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/ConanBootstrapToolchain.cmake" ABSOLUTE)

    if(NOT EXISTS "${_requested_toolchain}"
       AND NOT _requested_toolchain STREQUAL _bootstrap_toolchain)
        message(FATAL_ERROR
            "Requested toolchain does not exist: ${CMAKE_TOOLCHAIN_FILE}\n"
            "Use ${_bootstrap_toolchain} as CMAKE_TOOLCHAIN_FILE so Conan can "
            "bootstrap the generated toolchain, or run conan install manually "
            "before configuring.")
    endif()
endif()

include(cmake/ProjectLanguageDefaults.cmake)

# 项目期望的 C++ 标准
if(DEFINED CMAKE_CXX_STANDARD AND NOT CMAKE_CXX_STANDARD STREQUAL "")
    set(PROJECT_CXX_STANDARD "${CMAKE_CXX_STANDARD}")
else()
    set(PROJECT_CXX_STANDARD "${PROJECT_DEFAULT_CXX_STANDARD}")
endif()
if(PROJECT_CXX_STANDARD LESS 11)
    message(FATAL_ERROR "CMAKE_CXX_STANDARD must be at least 11")
endif()

# 项目元数据
set(PROJECT_NAME "1q" CACHE STRING "Root project name for this template")
set(PROJECT_ALIAS "${PROJECT_NAME}::core" CACHE STRING "Alias namespace for the primary library target")
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
set(PROJECT_CORE_TARGET "${PROJECT_NAME_LOWER}_lib" CACHE STRING "Primary library target name")
set(PROJECT_EXPORT_SET "${PROJECT_NAME}Targets" CACHE STRING "Export set name for installation")

# CMake 策略设置
if(POLICY CMP0091)
    cmake_policy(SET CMP0091 NEW)  # MSVC runtime library flags (CMake 3.15+)
endif()
if(POLICY CMP0092)
    cmake_policy(SET CMP0092 NEW)  # MSVC warning flags (CMake 3.15+)
endif()

# 项目声明
project(${PROJECT_NAME}
    VERSION 1.0.0
    DESCRIPTION "Project description"
    LANGUAGES CXX C
)

# C++ 标准配置
set(CMAKE_CXX_STANDARD "${PROJECT_CXX_STANDARD}")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)  # 为 IDE 工具提供编译命令数据库

# 构建类型配置
get_property(isMultiConfig GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)

if(isMultiConfig)
    # 多配置生成器（Visual Studio, Xcode）
    message(STATUS "Multi-config generator detected (${CMAKE_GENERATOR})")
    message(STATUS "Available configurations: ${CMAKE_CONFIGURATION_TYPES}")
    
    # 确保 RelWithDebInfo 配置可用
    if(NOT "RelWithDebInfo" IN_LIST CMAKE_CONFIGURATION_TYPES)
        list(APPEND CMAKE_CONFIGURATION_TYPES RelWithDebInfo)
    endif()
else()
    # 单配置生成器（Makefile, Ninja）
    if(NOT CMAKE_BUILD_TYPE)
        message(STATUS "Setting build type to 'Release' as none was specified")
        set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build" FORCE)
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS 
            "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
    endif()
    message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
endif()

# 平台兼容性检查
if(CMAKE_SIZEOF_VOID_P EQUAL 4)
    message(WARNING "32-bit compilation detected. This may not be fully supported.")
    message(WARNING "Consider using 64-bit architecture for better compatibility and performance.")
endif()

# 包含配置模块
# 用户可配置选项
include(cmake/ProjectOptions.cmake)

# 构建特性配置
include(cmake/FeatureUnityBuild.cmake)
include(cmake/FeatureCCache.cmake)
include(cmake/FeaturePrecompiledHeaders.cmake)
include(cmake/FeatureClangTidy.cmake)
include(cmake/FeatureClangFormat.cmake)
include(cmake/FeatureCoverage.cmake)
include(cmake/FeatureFlatBuffers.cmake)

# 编译器特定配置
if(MSVC)
    include(cmake/CompilerMSVC.cmake)
else()
    include(cmake/CompilerGNU.cmake)
endif()

# 包管理器配置（延迟到 target 创建之后）
# 见下方 add_subdirectory(src) 之后的包含

# 输出目录配置
if(isMultiConfig)
    # 多配置生成器：为每个配置创建独立目录
    foreach(OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES})
        string(TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG_UPPER)
        set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUTCONFIG_UPPER} ${CMAKE_BINARY_DIR}/${OUTPUTCONFIG}/lib)
        set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG_UPPER} ${CMAKE_BINARY_DIR}/${OUTPUTCONFIG}/lib)
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG_UPPER} ${CMAKE_BINARY_DIR}/${OUTPUTCONFIG}/bin)
    endforeach()
    message(STATUS "Output directories: ${CMAKE_BINARY_DIR}/<CONFIG>/{lib,bin}")
else()
    # 单配置生成器：所有输出到统一目录
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    message(STATUS "Output directories: ${CMAKE_BINARY_DIR}/{lib,bin}")
endif()

# 测试支持
if(ENABLE_TESTING)
    include(CTest)
    enable_testing()
    message(STATUS "Testing: Enabled (use 'ctest' to run tests)")
else()
    message(STATUS "Testing: Disabled")
endif()

# 安装规则配置
if(ENABLE_INSTALL)
    include(GNUInstallDirs)
    # 设置默认安装路径(可通过-DCMAKE_INSTALL_PREFIX修改)
    if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
        set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Installation prefix" FORCE)
    endif()
    message(STATUS "Installation: Enabled")
    message(STATUS "  └─ Prefix: ${CMAKE_INSTALL_PREFIX}")
    message(STATUS "  └─ Use 'cmake --install' to install")
else()
    message(STATUS "Installation: Disabled")
endif()

# 构建配置总结
message(STATUS "Configuration Summary")
message(STATUS "  └─ Project: ${PROJECT_NAME} ${PROJECT_VERSION}")
message(STATUS "  └─ C++ Standard: C++${CMAKE_CXX_STANDARD}")
message(STATUS "  └─ Build shared libs: ${BUILD_SHARED_LIBS}")
message(STATUS "  └─ Build examples: ${ENABLE_EXAMPLES}")
message(STATUS "  └─ Build tests: ${ENABLE_TESTING}")
message(STATUS "  └─ Coverage: ${ENABLE_COVERAGE}")
message(STATUS "  └─ Install support: ${ENABLE_INSTALL}")

# VS2015 行尾兼容处理
include(cmake/EnsureCRLF.cmake)

# 子目录包含
if(EXISTS "${CMAKE_SOURCE_DIR}/src/CMakeLists.txt")
    add_subdirectory(src)
else()
    message(FATAL_ERROR "Required module missing: ${CMAKE_SOURCE_DIR}/src/CMakeLists.txt")
endif()

# 包管理器配置（必须在 target 创建之后）
if(PACKAGE_MANAGER STREQUAL "conan")
    include(cmake/PackageManagerConan.cmake)
endif()

if(ENABLE_TESTING AND EXISTS "${CMAKE_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()

if(ENABLE_EXAMPLES AND EXISTS "${CMAKE_SOURCE_DIR}/examples/CMakeLists.txt")
    add_subdirectory(examples)
endif()

if(EXISTS "${CMAKE_SOURCE_DIR}/tools/CMakeLists.txt")
    add_subdirectory(tools)
endif()
