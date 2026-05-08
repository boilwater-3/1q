# 目标属性：输出名称、MSVC 运行时、静态库定义、PDB 路径、vendor 合并、符号可见性

# -- MSVC 运行时库覆盖 --
set(ONEQ_MSVC_RUNTIME_LIBRARY "" CACHE STRING
    "Optional MSVC runtime override (example: MultiThreadedDLL). Empty keeps toolchain/profile default.")
if(MSVC AND NOT ONEQ_MSVC_RUNTIME_LIBRARY STREQUAL "")
    foreach(ONEQ_BUILD_TARGET IN ITEMS
        ${PROJECT_CORE_TARGET}
        ${ONEQ_OBJECT_TARGETS}
    )
        set_property(TARGET ${ONEQ_BUILD_TARGET} PROPERTY
            MSVC_RUNTIME_LIBRARY "${ONEQ_MSVC_RUNTIME_LIBRARY}")
    endforeach()
    unset(ONEQ_BUILD_TARGET)
endif()

# -- 静态库宏定义 --
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

# -- 输出名称与版本 --
set_target_properties(${PROJECT_CORE_TARGET} PROPERTIES
    EXPORT_NAME ${PROJECT_NAME_LOWER}
    OUTPUT_NAME ${PROJECT_NAME_LOWER}
    DEBUG_POSTFIX d
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

# MSVC: PDB 输出到与 .lib 同目录（VS 生成器下 COMPILE_PDB_OUTPUT_DIRECTORY 对静态库不生效）
if(MSVC)
    get_property(_isMultiConfig GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
    if(_isMultiConfig)
        foreach(_cfg ${CMAKE_CONFIGURATION_TYPES})
            string(TOUPPER ${_cfg} _cfg_upper)
            target_compile_options(${PROJECT_CORE_TARGET} PRIVATE
                $<$<AND:$<CONFIG:${_cfg}>,$<CXX_COMPILER_ID:MSVC>>:/Fd${CMAKE_BINARY_DIR}/${_cfg}/lib/>
            )
        endforeach()
    else()
        target_compile_options(${PROJECT_CORE_TARGET} PRIVATE /Fd${CMAKE_BINARY_DIR}/lib/)
    endif()
endif()

# -- Vendor 模式下将编译型第三方静态库合并进主库 --
if(PACKAGE_MANAGER STREQUAL "none" AND ONEQ_VENDOR_MERGE_TARGETS)
    set(_merge_libs "$<TARGET_FILE:${PROJECT_CORE_TARGET}>")
    foreach(_dep IN LISTS ONEQ_VENDOR_MERGE_TARGETS)
        if(TARGET ${_dep})
            list(APPEND _merge_libs "$<TARGET_FILE:${_dep}>")
        endif()
    endforeach()
    if(MSVC)
        add_custom_command(TARGET ${PROJECT_CORE_TARGET} POST_BUILD
            COMMAND ${CMAKE_AR} /OUT:$<TARGET_FILE:${PROJECT_CORE_TARGET}> ${_merge_libs}
            COMMENT "Merging vendor dependencies into ${PROJECT_CORE_TARGET}"
        )
    else()
        # GCC/Clang: 解压所有 .a 的 .o 文件，重新打包
        set(_merge_dir "${CMAKE_BINARY_DIR}/vendor_merge_$<CONFIG>")
        add_custom_command(TARGET ${PROJECT_CORE_TARGET} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E rm -rf ${_merge_dir}
            COMMAND ${CMAKE_COMMAND} -E make_directory ${_merge_dir}
            COMMAND ${CMAKE_COMMAND} -E chdir ${_merge_dir} ${CMAKE_AR} x $<TARGET_FILE:${PROJECT_CORE_TARGET}>
            COMMAND ${CMAKE_COMMAND} -E chdir ${_merge_dir} ${CMAKE_AR} x $<TARGET_FILE:zlibstatic>
            COMMAND ${CMAKE_COMMAND} -E chdir ${_merge_dir} ${CMAKE_AR} x $<TARGET_FILE:flatbuffers>
            COMMAND ${CMAKE_COMMAND} -E chdir ${_merge_dir} ${CMAKE_AR} rcs $<TARGET_FILE:${PROJECT_CORE_TARGET}> *.o
            COMMENT "Merging vendor dependencies into ${PROJECT_CORE_TARGET}"
        )
    endif()
    unset(_merge_libs)
endif()

# -- 非 Windows 平台默认隐藏符号，仅显式导出公共 API --
if(NOT WIN32)
    set_target_properties(${PROJECT_CORE_TARGET} PROPERTIES
        CXX_VISIBILITY_PRESET hidden
        VISIBILITY_INLINES_HIDDEN YES
    )
endif()
