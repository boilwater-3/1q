# sar 示例：session 用法 / 集成演示

# session_usage：展示 SarSession 的标准会话使用方式。
add_executable(sar_session_usage session_usage.cpp)
target_link_libraries(sar_session_usage PRIVATE ${PROJECT_ALIAS})
target_include_directories(sar_session_usage PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${ONEQ_EXAMPLE_COMMON_DIR}")
target_sources(sar_session_usage PRIVATE ${ONEQ_EXAMPLE_COMMON_SOURCES})

# 集成模式示例：展示 SarModule 在外部引擎中的使用方式。
add_executable(sar_integration_demo integration_demo.cpp)
target_link_libraries(sar_integration_demo PRIVATE ${PROJECT_ALIAS})
target_include_directories(sar_integration_demo PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${ONEQ_EXAMPLE_COMMON_DIR}")
target_sources(sar_integration_demo PRIVATE ${ONEQ_EXAMPLE_COMMON_SOURCES})
