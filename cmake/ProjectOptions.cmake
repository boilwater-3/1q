# 用户可配置选项
# 本文件定义所有用户可配置的构建选项
# 修改这些选项可以控制项目的构建行为

# 基础构建选项

# BUILD_SHARED_LIBS: 构建动态库(.dll/.so)而非静态库(.lib/.a)
# 优点：减小可执行文件体积，便于库更新，减少内存占用
# 缺点：需要运行时依赖，部署稍复杂
option(BUILD_SHARED_LIBS "Build shared libraries instead of static" ON)

# ENABLE_TESTING: 启用CTest测试框架支持
# 开启后可使用add_test添加测试用例，通过 ctest 命令运行测试
option(ENABLE_TESTING "Enable testing support" OFF)

# ENABLE_EXAMPLES: 构建示例程序
# 包含项目相关的示例代码，便于学习和演示功能
option(ENABLE_EXAMPLES "Build example programs" OFF)

# ENABLE_INSTALL: 启用安装规则
# 开启后可使用 cmake --install 安装到系统或指定目录
option(ENABLE_INSTALL "Enable installation rules" OFF)

# 性能优化选项

# ENABLE_UNITY_BUILD: 启用Unity Build（需要 CMake 3.16+）
# 将多个源文件合并编译以加速构建，适合大型项目
# 优点：可减少30%-50%编译时间
# 缺点：可能隐藏某些编译错误（如重复符号定义）
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.16")
    option(ENABLE_UNITY_BUILD "Enable Unity Build for faster compilation" OFF)
else()
    set(ENABLE_UNITY_BUILD OFF CACHE INTERNAL "Unity Build requires CMake 3.16+")
endif()

# USE_CCACHE: 使用ccache缓存编译结果加速重新编译（仅Linux/macOS）
# 首次编译无加速，后续修改少量文件时显著提升编译速度
# 适合频繁增量编译的开发场景
if(UNIX AND NOT APPLE)
    option(USE_CCACHE "Use ccache to accelerate rebuilds" OFF)
elseif(APPLE)
    option(USE_CCACHE "Use ccache to accelerate rebuilds" OFF)
else()
    set(USE_CCACHE OFF CACHE INTERNAL "ccache is primarily for Linux/macOS")
endif()

# ENABLE_PCH: 启用预编译头（需要 CMake 3.16+）
# 可显著加速包含大量第三方库头文件的项目编译
# 适合包含 Boost、Qt、Windows SDK 等大型库的项目
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.16")
    option(ENABLE_PCH "Enable precompiled headers" OFF)
    mark_as_advanced(ENABLE_PCH)
else()
    set(ENABLE_PCH OFF CACHE INTERNAL "Precompiled headers require CMake 3.16+")
endif()

# 代码质量和调试选项

# ENABLE_WARNINGS: 启用额外的编译器警告
# MSVC: /W4 (高警告级别)
# GCC/Clang: -Wall -Wextra (所有常见警告)
# 推荐开启，帮助发现潜在问题，提高代码质量
option(ENABLE_WARNINGS "Enable additional compiler warnings" ON)

# ENABLE_CLANG_TIDY: 启用clang-tidy静态分析
# 在编译时执行代码检查，发现潜在bug、代码风格问题、现代化建议
# 会显著增加编译时间，建议仅在开发时开启
option(ENABLE_CLANG_TIDY "Enable clang-tidy static analysis" OFF)
mark_as_advanced(ENABLE_CLANG_TIDY)

# 高级选项

# STACK_SIZE_OPTION: 程序栈大小配置
# DEFAULT: 使用系统默认栈大小（Windows 1MB，Linux 8MB）
# RECOMMENDED: 2MB - 适合大多数项目的推荐配置
# LARGE_PROJECT: 4MB - 适合有复杂调用栈或大型局部变量的项目
# EXTREME_RECURSION: 8MB - 适合极端递归深度的算法（如深度优先搜索）
set(STACK_SIZE_OPTION "RECOMMENDED" CACHE STRING "Stack size configuration")
set_property(CACHE STACK_SIZE_OPTION PROPERTY STRINGS 
    "DEFAULT" "RECOMMENDED" "LARGE_PROJECT" "EXTREME_RECURSION")
mark_as_advanced(STACK_SIZE_OPTION)

# 包管理器选项

# PACKAGE_MANAGER: 选择包管理器
# none: 不使用包管理器（手动管理依赖）
# vcpkg: 使用 vcpkg（微软开发，跨平台，与 CMake 集成良好）
# conan: 使用 Conan（去中心化，支持多种构建系统）
set(PACKAGE_MANAGER "none" CACHE STRING "Package manager to use for dependencies")
set_property(CACHE PACKAGE_MANAGER PROPERTY STRINGS 
    "none" "vcpkg" "conan")

# 包管理器配置验证（调试信息）
message(STATUS "DEBUG: PACKAGE_MANAGER = '${PACKAGE_MANAGER}'")
message(STATUS "DEBUG: PACKAGE_MANAGER type = '${CMAKE_MATCH_0}'")

# 使用最可靠的 STREQUAL 方法
if(NOT ((PACKAGE_MANAGER STREQUAL "none") OR 
        (PACKAGE_MANAGER STREQUAL "vcpkg") OR 
        (PACKAGE_MANAGER STREQUAL "conan")))
    message(FATAL_ERROR "Invalid PACKAGE_MANAGER: '${PACKAGE_MANAGER}'. Must be one of: none, vcpkg, conan")
endif()

if(PACKAGE_MANAGER STREQUAL "vcpkg")
    message(STATUS "Package Manager: vcpkg")
elseif(PACKAGE_MANAGER STREQUAL "conan")
    message(STATUS "Package Manager: Conan")
else()
    message(STATUS "Package Manager: None (manual dependency management)")
endif()
