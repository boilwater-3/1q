# airborne_radar 示例：session 用法 / 配置对比 / 集成演示 / 端到端场景

# session_usage：展示 ArSession 的标准会话使用方式。
add_executable(airborne_radar_session_usage session_usage.cpp)
target_link_libraries(airborne_radar_session_usage PRIVATE ${PROJECT_ALIAS})
target_include_directories(airborne_radar_session_usage PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${ONEQ_EXAMPLE_COMMON_DIR}")
target_sources(airborne_radar_session_usage PRIVATE ${ONEQ_EXAMPLE_COMMON_SOURCES})

# 配置对比测试：校验多份 JSON 配置的加载与解析一致性。
add_executable(ar_config_compare_test config_compare_test.cpp)
target_link_libraries(ar_config_compare_test PRIVATE ${PROJECT_ALIAS})
target_include_directories(ar_config_compare_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${ONEQ_EXAMPLE_COMMON_DIR}")
target_sources(ar_config_compare_test PRIVATE ${ONEQ_EXAMPLE_COMMON_SOURCES})

# 集成模式示例：展示 RadarModule 在外部引擎中的使用方式。
add_executable(radar_integration_demo integration_demo.cpp)
target_link_libraries(radar_integration_demo PRIVATE ${PROJECT_ALIAS})
target_include_directories(radar_integration_demo PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${ONEQ_EXAMPLE_COMMON_DIR}")
target_sources(radar_integration_demo PRIVATE ${ONEQ_EXAMPLE_COMMON_SOURCES})

# 端到端场景：多目标对抗剧本 + 真实 session 跑多周期，作为模块级冒烟。
# 配置复用 examples/configs/（与 batch_validation 同源），通过 SCENE_CONFIG_DIR 注入。
add_executable(airborne_radar_scene airborne_radar_scene.cpp)
target_link_libraries(airborne_radar_scene PRIVATE ${PROJECT_ALIAS})
target_include_directories(airborne_radar_scene PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${ONEQ_EXAMPLE_COMMON_DIR}")
target_compile_definitions(airborne_radar_scene PRIVATE
    SCENE_CONFIG_DIR="${CMAKE_SOURCE_DIR}/examples/configs")
target_sources(airborne_radar_scene PRIVATE ${ONEQ_EXAMPLE_COMMON_SOURCES})
