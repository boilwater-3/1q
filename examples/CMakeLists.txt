# 示例程序构建配置
# 本目录包含项目的使用示例，每个源文件会被编译为独立的可执行文件

# 确保主库目标可用
if(NOT TARGET ${PROJECT_ALIAS})
    message(FATAL_ERROR "${PROJECT_ALIAS} target not found. Ensure src/CMakeLists.txt is processed first")
endif()

# 定义示例源文件
set(EXAMPLE_SOURCES
    example_radar_session.cpp
    example_quick_start.cpp
    example_advanced_injection.cpp
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

# ──────────────────────────────────────────────────────────────────────────────
# 可视化 Demo（ImGui + ImPlot 轨迹可视化）
# ──────────────────────────────────────────────────────────────────────────────
option(ENABLE_VISUALIZATION_EXAMPLES "Build ImPlot track visualization demo" OFF)
if(ENABLE_VISUALIZATION_EXAMPLES)
    find_package(imgui  CONFIG QUIET)
    find_package(implot CONFIG QUIET)
    find_package(glfw3  CONFIG QUIET)
    find_package(spdlog CONFIG REQUIRED)
    if(imgui_FOUND AND implot_FOUND AND glfw3_FOUND)
        # imgui conan 包的 bindings 目录（源文件 + 头文件）
        set(IMGUI_BINDINGS_DIR "${imgui_PACKAGE_FOLDER_RELEASE}/res/bindings")
        add_executable(example_track_visualizer
            example_track_visualizer.cpp
            "${IMGUI_BINDINGS_DIR}/imgui_impl_glfw.cpp"
            "${IMGUI_BINDINGS_DIR}/imgui_impl_opengl3.cpp"
        )
        target_link_libraries(example_track_visualizer PRIVATE
            ${PROJECT_ALIAS} imgui::imgui implot::implot glfw spdlog::spdlog)
        # imgui backends 头文件位于 res/bindings/，不在默认 include 路径
        target_include_directories(example_track_visualizer PRIVATE
            ${CMAKE_SOURCE_DIR}/include
            "${IMGUI_BINDINGS_DIR}")
        if(APPLE)
            target_link_libraries(example_track_visualizer PRIVATE
                "-framework OpenGL" "-framework Cocoa" "-framework IOKit")
        endif()
        message(STATUS "  └─ Visualization: example_track_visualizer")

        # 电子对抗场景可视化
        add_executable(example_ecm_scenario_visualizer
            example_ecm_scenario_visualizer.cpp
            "${IMGUI_BINDINGS_DIR}/imgui_impl_glfw.cpp"
            "${IMGUI_BINDINGS_DIR}/imgui_impl_opengl3.cpp"
        )
        target_link_libraries(example_ecm_scenario_visualizer PRIVATE
            ${PROJECT_ALIAS} imgui::imgui implot::implot glfw spdlog::spdlog)
        target_include_directories(example_ecm_scenario_visualizer PRIVATE
            ${CMAKE_SOURCE_DIR}/include
            "${IMGUI_BINDINGS_DIR}")
        if(APPLE)
            target_link_libraries(example_ecm_scenario_visualizer PRIVATE
                "-framework OpenGL" "-framework Cocoa" "-framework IOKit")
        endif()
        message(STATUS "  └─ Visualization: example_ecm_scenario_visualizer")

        # 多目标密集跟踪场景可视化
        add_executable(example_multi_target_visualizer
            example_multi_target_visualizer.cpp
            "${IMGUI_BINDINGS_DIR}/imgui_impl_glfw.cpp"
            "${IMGUI_BINDINGS_DIR}/imgui_impl_opengl3.cpp"
        )
        target_link_libraries(example_multi_target_visualizer PRIVATE
            ${PROJECT_ALIAS} imgui::imgui implot::implot glfw spdlog::spdlog)
        target_include_directories(example_multi_target_visualizer PRIVATE
            ${CMAKE_SOURCE_DIR}/include
            "${IMGUI_BINDINGS_DIR}")
        if(APPLE)
            target_link_libraries(example_multi_target_visualizer PRIVATE
                "-framework OpenGL" "-framework Cocoa" "-framework IOKit")
        endif()
        message(STATUS "  └─ Visualization: example_multi_target_visualizer")

        # 探测距离与 SNR 场景可视化
        add_executable(example_detection_range_visualizer
            example_detection_range_visualizer.cpp
            "${IMGUI_BINDINGS_DIR}/imgui_impl_glfw.cpp"
            "${IMGUI_BINDINGS_DIR}/imgui_impl_opengl3.cpp"
        )
        target_link_libraries(example_detection_range_visualizer PRIVATE
            ${PROJECT_ALIAS} imgui::imgui implot::implot glfw spdlog::spdlog)
        target_include_directories(example_detection_range_visualizer PRIVATE
            ${CMAKE_SOURCE_DIR}/include
            "${IMGUI_BINDINGS_DIR}")
        if(APPLE)
            target_link_libraries(example_detection_range_visualizer PRIVATE
                "-framework OpenGL" "-framework Cocoa" "-framework IOKit")
        endif()
        message(STATUS "  └─ Visualization: example_detection_range_visualizer")

        # 战术模式切换场景可视化
        add_executable(example_tactical_mode_visualizer
            example_tactical_mode_visualizer.cpp
            "${IMGUI_BINDINGS_DIR}/imgui_impl_glfw.cpp"
            "${IMGUI_BINDINGS_DIR}/imgui_impl_opengl3.cpp"
        )
        target_link_libraries(example_tactical_mode_visualizer PRIVATE
            ${PROJECT_ALIAS} imgui::imgui implot::implot glfw spdlog::spdlog)
        target_include_directories(example_tactical_mode_visualizer PRIVATE
            ${CMAKE_SOURCE_DIR}/include
            "${IMGUI_BINDINGS_DIR}")
        if(APPLE)
            target_link_libraries(example_tactical_mode_visualizer PRIVATE
                "-framework OpenGL" "-framework Cocoa" "-framework IOKit")
        endif()
        message(STATUS "  └─ Visualization: example_tactical_mode_visualizer")
    else()
        message(WARNING "[Visualization] 缺少 imgui/implot/glfw，跳过 visualization examples")
    endif()
endif()
