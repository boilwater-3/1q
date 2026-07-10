# electro_optical 示例：session 用法 / 集成演示 / 端到端场景

# session_usage：展示 EosSession 的标准会话使用方式。
add_executable(electro_optical_sensor_session_usage session_usage.cpp)
target_link_libraries(electro_optical_sensor_session_usage PRIVATE ${PROJECT_ALIAS})
target_include_directories(electro_optical_sensor_session_usage PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${ONEQ_EXAMPLE_COMMON_DIR}")
target_sources(electro_optical_sensor_session_usage PRIVATE ${ONEQ_EXAMPLE_COMMON_SOURCES})

# 集成模式示例：展示 EosModule 在外部引擎中的使用方式。
add_executable(eos_integration_demo integration_demo.cpp)
target_link_libraries(eos_integration_demo PRIVATE ${PROJECT_ALIAS})
target_include_directories(eos_integration_demo PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${ONEQ_EXAMPLE_COMMON_DIR}")
target_sources(eos_integration_demo PRIVATE ${ONEQ_EXAMPLE_COMMON_SOURCES})

# 端到端场景：多目标对抗剧本 + 真实 session 跑多周期，作为模块级冒烟。
add_executable(electro_optical_scene electro_optical_scene.cpp)
target_link_libraries(electro_optical_scene PRIVATE ${PROJECT_ALIAS})
target_include_directories(electro_optical_scene PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${ONEQ_EXAMPLE_COMMON_DIR}")
target_compile_definitions(electro_optical_scene PRIVATE
    SCENE_CONFIG_DIR="${CMAKE_SOURCE_DIR}/examples/configs")
target_sources(electro_optical_scene PRIVATE ${ONEQ_EXAMPLE_COMMON_SOURCES})
