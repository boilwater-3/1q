# 工具程序构建配置
# 本目录包含项目的辅助工具程序，每个 .cpp 文件会被编译为独立的可执行文件

# 自动发现所有工具源文件（每个 .cpp = 一个工具）
# 注意：如果你偏好显式控制，可以改用手动列表，例如：
#   set(TOOL_SOURCES tool_converter.cpp tool_validator.cpp)
file(GLOB TOOL_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

if(NOT TOOL_SOURCES)
    message(STATUS "No tool sources found in ${CMAKE_CURRENT_SOURCE_DIR}")
    message(STATUS "  └─ Add .cpp files to create utility programs")
    return()
endif()

# 为每个源文件创建独立的可执行文件
foreach(tool_src IN LISTS TOOL_SOURCES)
    get_filename_component(tool_name ${tool_src} NAME_WE)
    set(target_name ${PROJECT_NAME}_tool_${tool_name})
    
    add_executable(${target_name} ${tool_src})
    
    # 链接主库
    target_link_libraries(${target_name} PRIVATE ${PROJECT_ALIAS})
    
    # 包含目录
    target_include_directories(${target_name} PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
    
    # Windows DLL 运行时处理
    if(WIN32 AND BUILD_SHARED_LIBS)
        add_custom_command(TARGET ${target_name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:${PROJECT_CORE_TARGET}>
                $<TARGET_FILE_DIR:${target_name}>
            COMMENT "Copying ${PROJECT_CORE_TARGET} DLL for ${tool_name}"
        )
    endif()
    
    message(STATUS "  └─ Tool: ${tool_name}")
endforeach()
