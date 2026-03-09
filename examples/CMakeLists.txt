# 示例程序构建配置
# 本目录包含项目的使用示例，每个源文件会被编译为独立的可执行文件

# 确保主库目标可用
if(NOT TARGET ${PROJECT_ALIAS})
    message(FATAL_ERROR "${PROJECT_ALIAS} target not found. Ensure src/CMakeLists.txt is processed first")
endif()

# 定义示例源文件
set(EXAMPLE_SOURCES
    # 添加你的示例文件，例如：
    # example_basic_usage.cpp
    # example_advanced_features.cpp
    # example_custom_config.cpp
)

if(NOT EXAMPLE_SOURCES)
    message(STATUS "No example sources defined in ${CMAKE_CURRENT_SOURCE_DIR}")
    message(STATUS "  └─ Add .cpp files to EXAMPLE_SOURCES to create examples")
    return()
endif()

# 为每个示例创建可执行文件
foreach(example_src IN LISTS EXAMPLE_SOURCES)
    get_filename_component(example_name ${example_src} NAME_WE)
    
    add_executable(${example_name} ${example_src})
    
    # 链接主库
    target_link_libraries(${example_name} PRIVATE ${PROJECT_ALIAS})
    
    # 包含目录
    target_include_directories(${example_name} PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
    
    # Windows DLL 运行时处理
    if(WIN32 AND BUILD_SHARED_LIBS)
        add_custom_command(TARGET ${example_name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:${PROJECT_CORE_TARGET}>
                $<TARGET_FILE_DIR:${example_name}>
            COMMENT "Copying ${PROJECT_CORE_TARGET} DLL for ${example_name}"
        )
    endif()
    
    # 安装规则
    if(ENABLE_INSTALL)
        install(TARGETS ${example_name} 
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/examples
        )
    endif()
    
    message(STATUS "  └─ Example: ${example_name}")
endforeach()

# 配置总结
list(LENGTH EXAMPLE_SOURCES example_count)
message(STATUS "Configured ${example_count} example(s)")
