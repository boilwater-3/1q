# AR 示例构建配置

set(AR_EXAMPLE_SOURCES
    ar_radar_session.cpp
    ar_quick_start.cpp
    ar_advanced_injection.cpp
)

foreach(example_src IN LISTS AR_EXAMPLE_SOURCES)
    get_filename_component(example_name ${example_src} NAME_WE)

    add_executable(${example_name} ${example_src})
    target_link_libraries(${example_name} PRIVATE ${PROJECT_ALIAS})
    target_include_directories(${example_name} PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/examples
    )

    if(WIN32 AND BUILD_SHARED_LIBS)
        add_custom_command(TARGET ${example_name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:${PROJECT_CORE_TARGET}>
                $<TARGET_FILE_DIR:${example_name}>
            COMMENT "Copying ${PROJECT_CORE_TARGET} DLL for ${example_name}"
        )
    endif()

    if(ENABLE_INSTALL)
        install(TARGETS ${example_name}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/examples
        )
    endif()

    message(STATUS "  └─ Example(ar): ${example_name}")
endforeach()

list(LENGTH AR_EXAMPLE_SOURCES ar_example_count)
message(STATUS "Configured ${ar_example_count} AR example(s)")

# 可视化 Demo（ImGui + ImPlot）
option(ENABLE_VISUALIZATION_EXAMPLES "Build AR ImPlot visualization examples" OFF)
if(ENABLE_VISUALIZATION_EXAMPLES)
    find_package(imgui  CONFIG QUIET)
    find_package(implot CONFIG QUIET)
    find_package(glfw3  CONFIG QUIET)

    if(imgui_FOUND AND implot_FOUND AND glfw3_FOUND)
        set(IMGUI_BINDINGS_DIR "${imgui_PACKAGE_FOLDER_RELEASE}/res/bindings")

        add_executable(ar_track_visualizer
            ar_track_visualizer.cpp
            "${IMGUI_BINDINGS_DIR}/imgui_impl_glfw.cpp"
            "${IMGUI_BINDINGS_DIR}/imgui_impl_opengl3.cpp"
        )
        target_link_libraries(ar_track_visualizer PRIVATE ${PROJECT_ALIAS} imgui::imgui implot::implot glfw)
        target_include_directories(ar_track_visualizer PRIVATE
            ${CMAKE_SOURCE_DIR}/include
            "${IMGUI_BINDINGS_DIR}"
        )
        if(APPLE)
            target_link_libraries(ar_track_visualizer PRIVATE
                "-framework OpenGL" "-framework Cocoa" "-framework IOKit"
            )
        endif()
        message(STATUS "  └─ Visualization(ar): ar_track_visualizer")

        add_executable(ar_ecm_scenario_visualizer
            ar_ecm_scenario_visualizer.cpp
            "${IMGUI_BINDINGS_DIR}/imgui_impl_glfw.cpp"
            "${IMGUI_BINDINGS_DIR}/imgui_impl_opengl3.cpp"
        )
        target_link_libraries(ar_ecm_scenario_visualizer PRIVATE ${PROJECT_ALIAS} imgui::imgui implot::implot glfw)
        target_include_directories(ar_ecm_scenario_visualizer PRIVATE
            ${CMAKE_SOURCE_DIR}/include
            "${IMGUI_BINDINGS_DIR}"
        )
        if(APPLE)
            target_link_libraries(ar_ecm_scenario_visualizer PRIVATE
                "-framework OpenGL" "-framework Cocoa" "-framework IOKit"
            )
        endif()
        message(STATUS "  └─ Visualization(ar): ar_ecm_scenario_visualizer")

        add_executable(ar_multi_target_visualizer
            ar_multi_target_visualizer.cpp
            "${IMGUI_BINDINGS_DIR}/imgui_impl_glfw.cpp"
            "${IMGUI_BINDINGS_DIR}/imgui_impl_opengl3.cpp"
        )
        target_link_libraries(ar_multi_target_visualizer PRIVATE ${PROJECT_ALIAS} imgui::imgui implot::implot glfw)
        target_include_directories(ar_multi_target_visualizer PRIVATE
            ${CMAKE_SOURCE_DIR}/include
            "${IMGUI_BINDINGS_DIR}"
        )
        if(APPLE)
            target_link_libraries(ar_multi_target_visualizer PRIVATE
                "-framework OpenGL" "-framework Cocoa" "-framework IOKit"
            )
        endif()
        message(STATUS "  └─ Visualization(ar): ar_multi_target_visualizer")

        add_executable(ar_detection_range_visualizer
            ar_detection_range_visualizer.cpp
            "${IMGUI_BINDINGS_DIR}/imgui_impl_glfw.cpp"
            "${IMGUI_BINDINGS_DIR}/imgui_impl_opengl3.cpp"
        )
        target_link_libraries(ar_detection_range_visualizer PRIVATE ${PROJECT_ALIAS} imgui::imgui implot::implot glfw)
        target_include_directories(ar_detection_range_visualizer PRIVATE
            ${CMAKE_SOURCE_DIR}/include
            "${IMGUI_BINDINGS_DIR}"
        )
        if(APPLE)
            target_link_libraries(ar_detection_range_visualizer PRIVATE
                "-framework OpenGL" "-framework Cocoa" "-framework IOKit"
            )
        endif()
        message(STATUS "  └─ Visualization(ar): ar_detection_range_visualizer")

        add_executable(ar_tactical_mode_visualizer
            ar_tactical_mode_visualizer.cpp
            "${IMGUI_BINDINGS_DIR}/imgui_impl_glfw.cpp"
            "${IMGUI_BINDINGS_DIR}/imgui_impl_opengl3.cpp"
        )
        target_link_libraries(ar_tactical_mode_visualizer PRIVATE ${PROJECT_ALIAS} imgui::imgui implot::implot glfw)
        target_include_directories(ar_tactical_mode_visualizer PRIVATE
            ${CMAKE_SOURCE_DIR}/include
            "${IMGUI_BINDINGS_DIR}"
        )
        if(APPLE)
            target_link_libraries(ar_tactical_mode_visualizer PRIVATE
                "-framework OpenGL" "-framework Cocoa" "-framework IOKit"
            )
        endif()
        message(STATUS "  └─ Visualization(ar): ar_tactical_mode_visualizer")
    else()
        message(WARNING "[Visualization][ar] Missing imgui/implot/glfw, skip AR visualization examples")
    endif()
endif()
