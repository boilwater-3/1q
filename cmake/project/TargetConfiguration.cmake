# TargetConfiguration.cmake
# 目标级属性与编译选项的统一入口。
# 在 src/CMakeLists.txt 完成目标创建、依赖解析、FlatBuffers 代码生成后调用，
# 对所有 OBJECT 目标与核心库目标统一应用：
#   - 编译器选项（警告、栈大小，委托给对应编译器模块）
#   - 构建特性（Unity Build、Coverage、PCH，委托给对应 feature 模块）
#   - 目标属性（MSVC 运行时库、输出名称、符号可见性、静态库宏）

set(_oneq_config_targets ${PROJECT_CORE_TARGET} ${ONEQ_OBJECT_TARGETS})

# -- 构建特性 ----------------------------------------------------------------
apply_unity_build(TARGETS ${_oneq_config_targets})

if(MSVC)
    apply_msvc_options(
        TARGETS ${_oneq_config_targets}
        LINK_TARGETS ${PROJECT_CORE_TARGET}
        ENABLE_WARNINGS ${ENABLE_WARNINGS}
        STACK_SIZE_OPTION ${STACK_SIZE_OPTION})
else()
    apply_clang_gcc_options(
        TARGETS ${_oneq_config_targets}
        LINK_TARGETS ${PROJECT_CORE_TARGET}
        ENABLE_WARNINGS ${ENABLE_WARNINGS}
        STACK_SIZE_OPTION ${STACK_SIZE_OPTION})
endif()

apply_coverage_options(
    TARGETS ${_oneq_config_targets}
    LINK_TARGETS ${PROJECT_CORE_TARGET})

apply_precompiled_headers(TARGETS ${_oneq_config_targets})

# -- 目标属性 ----------------------------------------------------------------
# MSVC 运行时库：显式设置，编译与消费端 CRT 一致
set(ONEQ_MSVC_RUNTIME_LIBRARY "" CACHE STRING
    "MSVC runtime override. Empty = default dynamic CRT (/MD /MDd).")
if(MSVC)
    if(ONEQ_MSVC_RUNTIME_LIBRARY STREQUAL "")
        set(_oneq_msvc_rt "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
    else()
        set(_oneq_msvc_rt "${ONEQ_MSVC_RUNTIME_LIBRARY}")
    endif()
    set_property(TARGET ${PROJECT_CORE_TARGET} PROPERTY
        MSVC_RUNTIME_LIBRARY "${_oneq_msvc_rt}")
    foreach(ONEQ_BUILD_TARGET IN LISTS ONEQ_OBJECT_TARGETS)
        set_property(TARGET ${ONEQ_BUILD_TARGET} PROPERTY
            MSVC_RUNTIME_LIBRARY "${_oneq_msvc_rt}")
    endforeach()
    unset(ONEQ_BUILD_TARGET)
    unset(_oneq_msvc_rt)
endif()

# 静态库宏定义：消费端需用 ONEQ_STATIC_DEFINE 告知导入符号为静态链接
if(NOT BUILD_SHARED_LIBS)
    target_compile_definitions(${PROJECT_CORE_TARGET}
        PUBLIC ONEQ_STATIC_DEFINE
    )
    foreach(ONEQ_OBJECT_TARGET IN LISTS ONEQ_OBJECT_TARGETS)
        target_compile_definitions(${ONEQ_OBJECT_TARGET}
            PRIVATE ONEQ_STATIC_DEFINE
        )
    endforeach()
    unset(ONEQ_OBJECT_TARGET)
endif()

# 输出名称与版本
set_target_properties(${PROJECT_CORE_TARGET} PROPERTIES
    EXPORT_NAME ${PROJECT_NAME_LOWER}
    OUTPUT_NAME ${PROJECT_NAME_LOWER}
    DEBUG_POSTFIX d
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

# 非 Windows 平台默认隐藏符号，仅显式导出公共 API
if(NOT WIN32)
    set_target_properties(${PROJECT_CORE_TARGET} PROPERTIES
        CXX_VISIBILITY_PRESET hidden
        VISIBILITY_INLINES_HIDDEN YES
    )
endif()

unset(_oneq_config_targets)
