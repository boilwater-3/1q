# 安装规则：库目标、PDB、头文件、CMake 包配置

if(NOT ENABLE_INSTALL)
    return()
endif()

include(CMakePackageConfigHelpers)

set(PUBLIC_HEADERS_ROOT
    ${CMAKE_SOURCE_DIR}/include/1q/api.hpp
    ${CMAKE_SOURCE_DIR}/include/1q/README.md
)

# 安装库目标
install(TARGETS ${PROJECT_CORE_TARGET}
    EXPORT ${PROJECT_EXPORT_SET}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# 安装公共头文件（显式白名单，避免内部头被自动导出）
install(FILES ${PUBLIC_HEADERS_ROOT}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q
)
install(FILES "${PROJECT_GENERATED_EXPORT_HEADER}"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q
)

# 安装导出的 CMake targets 文件
install(EXPORT ${PROJECT_EXPORT_SET}
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
    FILE ${PROJECT_NAME}Targets.cmake
)

# 生成 PackageConfig 文件（供 find_package 使用）
configure_package_config_file(
    "${CMAKE_CURRENT_LIST_DIR}/ProjectTemplateConfig.cmake.in"
    "${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
    PATH_VARS CMAKE_INSTALL_INCLUDEDIR
)

# 生成版本文件
write_basic_package_version_file(
    "${CMAKE_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

# 安装 Config 文件
install(FILES
    "${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    "${CMAKE_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
