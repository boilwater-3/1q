# 示例程序构建入口
# 顶层仅负责公共前置检查和子模块分发。

if(NOT TARGET ${PROJECT_ALIAS})
    message(FATAL_ERROR "${PROJECT_ALIAS} target not found. Ensure src/CMakeLists.txt is processed first")
endif()

add_executable(airborne_radar_session_usage airborne_radar_session_usage.cpp)
target_link_libraries(airborne_radar_session_usage PRIVATE ${PROJECT_ALIAS})
