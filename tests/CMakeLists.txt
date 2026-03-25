if(NOT ENABLE_TESTING)
    return()
endif()

# 查找测试框架（测试专用依赖，不污染主库）
find_package(GTest CONFIG REQUIRED)
find_package(eventpp CONFIG REQUIRED)
find_package(Boost CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
option(ENABLE_STRESS_TESTING "Enable long-running radar stress tests" OFF)

# 定义测试源文件
set(TEST_SOURCES
    tactical_coordinator_test.cpp
    decision_layer_test.cpp
    data_output_manager_test.cpp
    output_frame_utils_test.cpp
    core_controller_test.cpp
    public_api_convenience_test.cpp
    public_headers_smoke_test.cpp
    radar_joint_integration_test.cpp
    event_bus_test.cpp
    antenna_pattern_utils_test.cpp
    beamwidth_resolution_test.cpp
    radar_orientation_utils_test.cpp
    signal_environment_test.cpp
    signal_association_test.cpp
    signal_detection_test.cpp
    geometry_transform_test.cpp
    timing_regime_model_test.cpp
    validation_utils_test.cpp
    radar_input_validation_test.cpp
    esr_input_validation_test.cpp
    esr_algorithms_test.cpp
    esr_environment_service_test.cpp
    esr_kdtree_clusterer_test.cpp
    esr_hypothesis_associator_test.cpp
    esr_output_manager_test.cpp
    esr_session_config_resolver_test.cpp
    esr_session_integration_test.cpp
    # NOTE:
    # signal_bulk_data_test.cpp contains long-running IMM debug workload tests.
    # Keep it disabled until IMM multi-thread optimization is implemented,
    # then restore it to validate the optimized path.
    # signal_bulk_data_test.cpp
    track_lifecycle_test.cpp
    kalman_filter_test.cpp
    advanced_filter_test.cpp
    radar_session_config_builder_test.cpp
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

add_test(
    NAME public_api_boundary_guard
    COMMAND ${CMAKE_COMMAND}
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/check_public_api_boundary.cmake
)

if(ENABLE_STRESS_TESTING)
    set(STRESS_TEST_SOURCES
        signal_bulk_data_test.cpp
        radar_detection_stress_test.cpp
    )

    add_executable(${PROJECT_NAME}_stress_tests ${STRESS_TEST_SOURCES})
    target_link_libraries(${PROJECT_NAME}_stress_tests PRIVATE
        GTest::gtest_main
        GTest::gmock
        eventpp::eventpp
        Boost::boost
        Eigen3::Eigen
        ${PROJECT_ALIAS}
    )
    target_include_directories(${PROJECT_NAME}_stress_tests PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}
    )

    if(WIN32 AND BUILD_SHARED_LIBS)
        add_custom_command(TARGET ${PROJECT_NAME}_stress_tests POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:${PROJECT_CORE_TARGET}>
                $<TARGET_FILE_DIR:${PROJECT_NAME}_stress_tests>
            COMMENT "Copying ${PROJECT_CORE_TARGET} DLL to stress test directory"
        )
    endif()

    gtest_discover_tests(${PROJECT_NAME}_stress_tests
        DISCOVERY_MODE PRE_TEST
        DISCOVERY_TIMEOUT 120
        TEST_PREFIX "stress::"
        PROPERTIES LABELS "stress"
    )
else()
    message(STATUS "Stress tests disabled. Configure with -DENABLE_STRESS_TESTING=ON to enable long-running suites.")
endif()
