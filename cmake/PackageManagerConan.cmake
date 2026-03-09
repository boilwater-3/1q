# Conan 包管理器配置
# 本文件管理所有通过 Conan 安装的第三方依赖

message(STATUS "Configuring Conan dependencies...")

# Conan 集成检查
# Conan 2.x 推荐使用 CMakeToolchain + CMakeDeps
set(CONAN_BUILD_INFO "${CMAKE_BINARY_DIR}/conan_toolchain.cmake")
if(DEFINED CMAKE_TOOLCHAIN_FILE AND EXISTS "${CMAKE_TOOLCHAIN_FILE}")
    message(STATUS "  └─ Using toolchain: ${CMAKE_TOOLCHAIN_FILE}")
elseif(EXISTS "${CONAN_BUILD_INFO}")
    include("${CONAN_BUILD_INFO}")
    message(STATUS "  └─ Found Conan toolchain: ${CONAN_BUILD_INFO}")
else()
    message(WARNING "Conan toolchain not found at: ${CONAN_BUILD_INFO}")
    message(WARNING "  To use Conan, run in build directory:")
    message(WARNING "  conan install <source_dir> --output-folder=. --build=missing")
endif()

# 第三方依赖声明
# Conan 生成的 find_package 配置会自动提供这些库

# 示例：常用库（对应 conanfile.txt 或 conanfile.py 中的 requires）
# find_package(spdlog REQUIRED)
# find_package(nlohmann_json REQUIRED)
# find_package(fmt REQUIRED)
# find_package(Catch2)

# 链接依赖到主库目标
# 注意：此变量由 src/CMakeLists.txt 提供
if(TARGET ${PROJECT_CORE_TARGET})
    # Conan 2.x 目标命名约定：<package>::<package>
    # target_link_libraries(${PROJECT_CORE_TARGET}
    #     PUBLIC
    #         spdlog::spdlog
    #         nlohmann_json::nlohmann_json
    #     PRIVATE
    #         fmt::fmt
    # )
    
    message(STATUS "  └─ No dependencies configured (edit PackageManagerConan.cmake to add)")
else()
    message(FATAL_ERROR "PROJECT_CORE_TARGET not defined. PackageManagerConan.cmake must be included after target creation.")
endif()

# Conan generated files description
message(STATUS "  └─ Conan generates:")
message(STATUS "      • conan_toolchain.cmake - Toolchain configuration")
message(STATUS "      • Find*.cmake - Package finder scripts")
message(STATUS "      • *-config.cmake - CMake config files")

# Windows DLL 运行时安装（可选）
if(ENABLE_INSTALL AND WIN32 AND BUILD_SHARED_LIBS)
    # Conan 2.x 会将 DLLs 放在 bin/ 目录
    # 可以使用 install(DIRECTORY) 或 install(FILES) 手动安装
    # file(GLOB CONAN_DLLS "${CMAKE_BINARY_DIR}/bin/*.dll")
    # install(FILES ${CONAN_DLLS} DESTINATION ${CMAKE_INSTALL_BINDIR})
    message(STATUS "  └─ DLL installation: Manual configuration required")
endif()
