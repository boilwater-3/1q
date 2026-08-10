# FD source-level partitions. Avoids suite filters while preserving the
# non-blocking boundary for known-limit and performance scenarios.

set(_oneq_fd_stable_sources
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/flight_dynamic/fd_adapter_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/flight_dynamic/fd_bare_aircraft_baseline_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/flight_dynamic/fd_robustness_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/flight_dynamic/fd_aircraft_performance_derivation_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/flight_dynamic/fd_angle_normalization_test.cpp")
set(_oneq_fd_known_limit_sources
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/flight_dynamic/fd_aircraft_maneuver_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/flight_dynamic/fd_orbit_quality_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/flight_dynamic/fd_takeoff_substep_test.cpp")

if(ONEQ_ENABLE_FLIGHT_DYNAMIC)
    oneq_add_test_partition(
        TYPE unit DOMAIN flight_dynamic
        SOURCES ${_oneq_fd_stable_sources}
        TIMEOUT 180
        LABELS ci_required)
    oneq_add_test_partition(
        TYPE known_limit DOMAIN flight_dynamic
        SOURCES ${_oneq_fd_known_limit_sources}
        TIMEOUT 180
        LABELS performance known_limit)
    # FD 开发期验证工具（逐帧轨迹 CSV 导出 + 质量分析，非 GTest，不注册
    # ctest）：随测试构建编译，供模块开发/证据分析使用，见该目录 README.md。
    add_subdirectory(unit/flight_dynamic/fd_tools)
    foreach(_target IN ITEMS
        ${PROJECT_NAME}_flight_dynamic_unit_tests
        ${PROJECT_NAME}_flight_dynamic_known_limit_tests)
        target_include_directories(${_target} SYSTEM PRIVATE
            ${CMAKE_SOURCE_DIR}/third_party/jsbsim/src)
        if(TARGET JSBSim::JSBSim)
            target_link_libraries(${_target} PRIVATE JSBSim::JSBSim)
        endif()
        if(DEFINED ONEQ_JSBSIM_DATA_ROOT_DIR AND NOT ONEQ_JSBSIM_DATA_ROOT_DIR STREQUAL "")
            target_compile_definitions(${_target} PRIVATE FD_JSBSIM_ROOT_DIR="${ONEQ_JSBSIM_DATA_ROOT_DIR}")
        else()
            target_compile_definitions(${_target} PRIVATE FD_JSBSIM_ROOT_DIR="${CMAKE_SOURCE_DIR}/third_party/jsbsim")
        endif()
    endforeach()
    add_custom_target(${PROJECT_NAME}_fd_tests)
    add_dependencies(${PROJECT_NAME}_fd_tests
        ${PROJECT_NAME}_flight_dynamic_unit_tests
        ${PROJECT_NAME}_flight_dynamic_known_limit_tests)
    add_dependencies(${PROJECT_NAME}_unit_tests
        ${PROJECT_NAME}_flight_dynamic_unit_tests
        ${PROJECT_NAME}_flight_dynamic_known_limit_tests)
else()
    oneq_register_test_sources("unit.flight_dynamic" ${_oneq_fd_stable_sources} ${_oneq_fd_known_limit_sources})
endif()
