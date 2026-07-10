# ProjectDependencies.cmake
# 第三方依赖发现入口
# 仅创建/导入 provider target，业务模块在各自 CMakeLists 中消费这些 target

set(ONEQ_PROJECT_DEPENDENCIES_DIR "${CMAKE_CURRENT_LIST_DIR}")

function(oneq_find_project_dependencies)
    # 先解析 JSBSim（三条策略之一），再解析 Conan 包并组装 ONEQ_LINK_DEPENDENCIES。
    include("${ONEQ_PROJECT_DEPENDENCIES_DIR}/dependencies/JsbsimProvider.cmake")
    include("${ONEQ_PROJECT_DEPENDENCIES_DIR}/dependencies/ConanPackages.cmake")

    # 将 provider 解析出的结果变量提升到调用方作用域，供 ProjectTargets / 业务模块使用。
    foreach(_oneq_dependency_var IN ITEMS
            ONEQ_HAVE_ZLIB
            ONEQ_ENABLE_HDF5_OUTPUT
            ONEQ_LINK_DEPENDENCIES
            ONEQ_JSBSIM_BINARY_SOURCE
            ONEQ_JSBSIM_DATA_ROOT_DIR
            ONEQ_JSBSIM_DATA_SOURCE
            PROJECT_ENABLE_SPDLOG
            PROJECT_SPDLOG_TARGET)
        if(DEFINED ${_oneq_dependency_var})
            set(${_oneq_dependency_var} "${${_oneq_dependency_var}}" PARENT_SCOPE)
        endif()
    endforeach()
endfunction()
