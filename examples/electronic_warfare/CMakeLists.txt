# electronic_warfare 示例：session 用法 / 集成演示 / 端到端场景

# session_usage：展示 EsrSession 的标准会话使用方式。
add_executable(electronic_surveillance_radar_session_usage session_usage.cpp)
target_link_libraries(electronic_surveillance_radar_session_usage PRIVATE ${PROJECT_ALIAS})
target_include_directories(electronic_surveillance_radar_session_usage PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${ONEQ_EXAMPLE_COMMON_DIR}")
target_sources(electronic_surveillance_radar_session_usage PRIVATE ${ONEQ_EXAMPLE_COMMON_SOURCES})

# 集成模式示例：展示 EsrModule 在外部引擎中的使用方式。
add_executable(esr_integration_demo integration_demo.cpp)
target_link_libraries(esr_integration_demo PRIVATE ${PROJECT_ALIAS})
target_include_directories(esr_integration_demo PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${ONEQ_EXAMPLE_COMMON_DIR}")
target_sources(esr_integration_demo PRIVATE ${ONEQ_EXAMPLE_COMMON_SOURCES})

# 端到端场景：多目标对抗剧本 + 真实 session 跑多周期，作为模块级冒烟。
add_executable(electronic_warfare_scene electronic_warfare_scene.cpp)
target_link_libraries(electronic_warfare_scene PRIVATE ${PROJECT_ALIAS})
target_include_directories(electronic_warfare_scene PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${ONEQ_EXAMPLE_COMMON_DIR}")
target_compile_definitions(electronic_warfare_scene PRIVATE
    SCENE_CONFIG_DIR="${CMAKE_SOURCE_DIR}/examples/configs")
target_sources(electronic_warfare_scene PRIVATE ${ONEQ_EXAMPLE_COMMON_SOURCES})
