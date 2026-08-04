# dependencies/ConanPackages.cmake
# Conan 包发现与项目级依赖列表导出

# PACKAGE_MANAGER 已在 ProjectOptions.cmake 校验为 conan，此处直接解析包。
# spdlog 在 Windows 上存在已知的编译/链接问题，仅 UNIX 下启用。
if(WIN32)
    set(PROJECT_ENABLE_SPDLOG OFF)
else()
    set(PROJECT_ENABLE_SPDLOG ON)
endif()
# 核心第三方库：特征值、Boost 头文件、nanoflann 近邻搜索、FlatBuffers 运行时。
find_package(Eigen3 CONFIG REQUIRED)
find_package(Boost CONFIG REQUIRED)
find_package(nanoflann CONFIG REQUIRED)
find_package(flatbuffers CONFIG REQUIRED)
if(PROJECT_ENABLE_SPDLOG)
    find_package(spdlog CONFIG REQUIRED)
    # 仅接受编译型 spdlog::spdlog；header-only 形态在本项目中不被允许，
    # 因日志宏需绑定编译期后端实现而非纯头文件展开。
    if(TARGET spdlog::spdlog)
        set(PROJECT_SPDLOG_TARGET spdlog::spdlog)
    elseif(TARGET spdlog::spdlog_header_only)
        message(FATAL_ERROR "spdlog header-only target is not allowed in this configuration")
    else()
        message(FATAL_ERROR "spdlog target is not available")
    endif()
endif()
find_package(ZLIB REQUIRED)

# sqlite3（airborne_radar 识别特征库加载；库内部 PRIVATE 依赖，不进 ONEQ_LINK_DEPENDENCIES）。
# conan-center 的 sqlite3 recipe 经 CMakeDeps 生成包名 SQLite3、target SQLite::SQLite3。
find_package(SQLite3 CONFIG REQUIRED)

# HDF5 输出能力依赖 HighFive，其要求 C++17；低标准构建时静默关闭。
if(PROJECT_CXX_STANDARD GREATER_EQUAL 17)
    find_package(HighFive CONFIG REQUIRED)
    set(ONEQ_ENABLE_HDF5_OUTPUT ON)
    message(STATUS "SAR HDF5 output: ENABLED (HighFive found)")
else()
    set(ONEQ_ENABLE_HDF5_OUTPUT OFF)
    message(STATUS "SAR HDF5 output: disabled (requires C++17, current: C++${PROJECT_CXX_STANDARD})")
endif()

# 标记 zlib 可用；导出公共库 target 需链接的依赖列表。
set(ONEQ_HAVE_ZLIB ON)
set(ONEQ_LINK_DEPENDENCIES
    Eigen3::Eigen
    Boost::boost
    nanoflann::nanoflann
    flatbuffers::flatbuffers)
