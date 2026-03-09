# vcpkg 包管理器配置
# 本文件管理所有通过 vcpkg 安装的第三方依赖

message(STATUS "Configuring vcpkg dependencies...")

# vcpkg 集成检查
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    message(WARNING "CMAKE_TOOLCHAIN_FILE is not set.")
    message(WARNING "  To use vcpkg, configure with:")
    message(WARNING "  cmake -DCMAKE_TOOLCHAIN_FILE=<vcpkg_root>/scripts/buildsystems/vcpkg.cmake")
endif()

# 第三方依赖声明
# 在此处使用 find_package 查找 vcpkg 安装的库

# 示例：常用库
# find_package(spdlog CONFIG REQUIRED)
# find_package(nlohmann_json CONFIG REQUIRED)
# find_package(fmt CONFIG REQUIRED)
# find_package(Catch2 CONFIG)

# 链接依赖到主库目标
# 注意：此变量由 src/CMakeLists.txt 提供
if(TARGET ${PROJECT_CORE_TARGET})
    # target_link_libraries(${PROJECT_CORE_TARGET}
    #     PUBLIC
    #         spdlog::spdlog
    #         nlohmann_json::nlohmann_json
    #     PRIVATE
    #         fmt::fmt
    # )
    
    message(STATUS "  └─ No dependencies configured (edit PackageManagerVcpkg.cmake to add)")
else()
    message(FATAL_ERROR "PROJECT_CORE_TARGET not defined. PackageManagerVcpkg.cmake must be included after target creation.")
endif()

# Windows DLL 运行时安装（可选）
if(ENABLE_INSTALL AND WIN32 AND BUILD_SHARED_LIBS)
    if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.21)
        # CMake 3.21+ 支持自动安装导入的运行时文件
        # install(IMPORTED_RUNTIME_ARTIFACTS
        #     spdlog::spdlog
        #     nlohmann_json::nlohmann_json
        #     RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        # )
        message(STATUS "  └─ DLL installation: Available (uncomment to enable)")
    else()
        # 使用自定义辅助函数（需要 cmake/ProjectRuntimeHelpers.cmake）
        # include(${CMAKE_SOURCE_DIR}/cmake/ProjectRuntimeHelpers.cmake)
        # project_helper_install_vcpkg_runtime_dlls()
        message(STATUS "  └─ DLL installation: Requires custom helper (CMake < 3.21)")
    endif()
endif()
