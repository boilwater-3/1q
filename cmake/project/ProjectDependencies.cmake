# ProjectDependencies.cmake
# 第三方依赖发现入口
# 仅创建/导入 provider target，业务模块在各自 CMakeLists 中消费这些 target

set(ONEQ_PROJECT_DEPENDENCIES_DIR "${CMAKE_CURRENT_LIST_DIR}")

function(oneq_find_project_dependencies)
    # 按 PACKAGE_MANAGER 分派依赖 provider。两个 provider 对外契约相同：
    # 均产出 ONEQ_LINK_DEPENDENCIES、ONEQ_HAVE_ZLIB、PROJECT_ENABLE_SPDLOG、
    # ONEQ_ENABLE_HDF5_OUTPUT 等变量与 Eigen3::Eigen 等标准 imported target，
    # 故下游 src/、tests/、install 消费点不感知 provider 切换。
    if(PACKAGE_MANAGER STREQUAL "none")
        include("${ONEQ_PROJECT_DEPENDENCIES_DIR}/dependencies/VendorPackages.cmake")
    else()
        include("${ONEQ_PROJECT_DEPENDENCIES_DIR}/dependencies/ConanPackages.cmake")
    endif()
    # JSBSim 解析与 provider 无关（自有三条策略，含 conan 包/源码/预编译）。
    include("${ONEQ_PROJECT_DEPENDENCIES_DIR}/dependencies/JsbsimProvider.cmake")

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
