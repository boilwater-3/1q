if(NOT ENABLE_TESTING)
    return()
endif()

# 查找测试框架（测试专用依赖，不污染主库）
find_package(GTest CONFIG REQUIRED)
find_package(eventpp CONFIG REQUIRED)
find_package(Boost CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

# 定义测试源文件
set(TEST_SOURCES
    decision_layer_test.cpp
    core_controller_test.cpp
    event_bus_test.cpp
    signal_environment_test.cpp
    signal_association_test.cpp
    track_lifecycle_test.cpp
)

if(NOT TEST_SOURCES)
    message(STATUS "No test sources defined in ${CMAKE_CURRENT_SOURCE_DIR}")
    message(STATUS "  └─ Add .cpp files to TEST_SOURCES to enable testing")
    return()
endif()

# 创建测试可执行文件
add_executable(${PROJECT_NAME}_tests ${TEST_SOURCES})

# 链接测试依赖
target_link_libraries(${PROJECT_NAME}_tests PRIVATE
    GTest::gtest_main  # 包含 main() 函数
    GTest::gmock       # Mock 功能（可选）
    eventpp::eventpp
    Boost::boost
    Eigen3::Eigen
    ${PROJECT_ALIAS}   # 主库
)

# 包含目录
target_include_directories(${PROJECT_NAME}_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Windows DLL 运行时处理
if(WIN32 AND BUILD_SHARED_LIBS)
    # 复制主库 DLL 到测试目录
    add_custom_command(TARGET ${PROJECT_NAME}_tests POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:${PROJECT_CORE_TARGET}>
            $<TARGET_FILE_DIR:${PROJECT_NAME}_tests>
        COMMENT "Copying ${PROJECT_CORE_TARGET} DLL to tests directory"
    )
    
    # CMake 3.21+ 自动复制第三方 DLLs
    if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.21)
        # 会自动复制 GTest 和其他链接的导入目标的 DLLs
        # 无需手动配置
    else()
        message(STATUS "  └─ CMake < 3.21: May need manual DLL copying for third-party libraries")
    endif()
endif()

# 注册测试用例（使用 GoogleTest 自动发现）
include(GoogleTest)
gtest_discover_tests(${PROJECT_NAME}_tests
    DISCOVERY_MODE PRE_TEST
    DISCOVERY_TIMEOUT 60
)
