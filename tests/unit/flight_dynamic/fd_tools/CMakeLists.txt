# FD 开发期验证工具：机动模块逐帧轨迹 CSV 导出 + 质量分析（依赖 JSBSim）。
# 本目录由 FlightDynamicPartitions.cmake 的 ONEQ_ENABLE_FLIGHT_DYNAMIC
# 分支 add_subdirectory 纳入，进入此文件时开关已为 ON；随测试构建
# （ENABLE_TESTING）编译，不注册 ctest（确定性断言归 gtest 分区，
# 质量/轨迹分析归本目录工具，见 README.md）。
#
# 开发期工具可使用 src/ 内部头（如 takeoff_land_csv 的
# JsbsimAdapter/EngineManager 与 JSBSim 模型头）——非消费方示例，
# 属模块开发期验证形态。

# 共享装配：链接主库 + 注入 src 与 jsbsim 头路径 + 运行时数据根目录。
macro(setup_fd_example target)
    target_link_libraries(${target} PRIVATE ${PROJECT_ALIAS})
    # 公共静态库不传播组件级 PRIVATE 依赖（src/CMakeLists.txt 链接闭包注释），
    # FD 消费方需显式链接 JSBSim（对齐 tests/cmake/FlightDynamicPartitions.cmake
    # 的测试分区接线；此前仅在 FD ON + ENABLE_EXAMPLES=ON 的全量构建中暴露，
    # nightly 未开 ENABLE_EXAMPLES 故未发现，2026-08-10 迁入 tests 后改由
    # ENABLE_TESTING + ONEQ_ENABLE_FLIGHT_DYNAMIC 门控）。
    target_link_libraries(${target} PRIVATE JSBSim::JSBSim)
    target_include_directories(${target} PRIVATE "${CMAKE_SOURCE_DIR}/src")
    target_include_directories(${target} SYSTEM PRIVATE
        "${CMAKE_SOURCE_DIR}/third_party/jsbsim/src")
    if(DEFINED ONEQ_JSBSIM_DATA_ROOT_DIR AND NOT ONEQ_JSBSIM_DATA_ROOT_DIR STREQUAL "")
        set(FD_JSBSIM_ROOT_DIR "${ONEQ_JSBSIM_DATA_ROOT_DIR}")
    else()
        set(FD_JSBSIM_ROOT_DIR "${CMAKE_SOURCE_DIR}/third_party/jsbsim")
    endif()
    target_compile_definitions(${target} PRIVATE
        FD_JSBSIM_ROOT_DIR="${FD_JSBSIM_ROOT_DIR}")
endmacro()

add_executable(takeoff_land_csv takeoff_land_csv.cpp)
setup_fd_example(takeoff_land_csv)

add_executable(orbit_quality_csv orbit_quality_csv.cpp)
setup_fd_example(orbit_quality_csv)

add_executable(racetrack_trace_csv racetrack_trace_csv.cpp)
setup_fd_example(racetrack_trace_csv)

add_executable(sturn_trace_csv sturn_trace_csv.cpp)
setup_fd_example(sturn_trace_csv)

add_executable(orbit_trace_csv orbit_trace_csv.cpp)
setup_fd_example(orbit_trace_csv)

add_executable(racetrack_approach_trace racetrack_approach_trace.cpp)
setup_fd_example(racetrack_approach_trace)

add_executable(figure8_approach_trace figure8_approach_trace.cpp)
setup_fd_example(figure8_approach_trace)
