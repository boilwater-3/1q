# flight_dynamic 示例：机动模块 CSV 工具与轨迹生成（依赖 JSBSim）
# 本目录由顶层 ONEQ_ENABLE_FLIGHT_DYNAMIC 门控纳入，进入此文件时开关已为 ON。

# 共享装配：链接主库 + 注入 src 与 jsbsim 头路径 + 运行时数据根目录。
macro(setup_fd_example target)
    target_link_libraries(${target} PRIVATE ${PROJECT_ALIAS})
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

add_executable(maneuver_sweep_csv maneuver_sweep_csv.cpp)
setup_fd_example(maneuver_sweep_csv)

add_executable(orbit_quality_csv orbit_quality_csv.cpp)
setup_fd_example(orbit_quality_csv)

add_executable(racetrack_quality_csv racetrack_quality_csv.cpp)
setup_fd_example(racetrack_quality_csv)

add_executable(racetrack_trace_csv racetrack_trace_csv.cpp)
setup_fd_example(racetrack_trace_csv)

add_executable(sturn_trace_csv sturn_trace_csv.cpp)
setup_fd_example(sturn_trace_csv)

add_executable(orbit_trace_csv orbit_trace_csv.cpp)
setup_fd_example(orbit_trace_csv)

add_executable(orbit_long_duration orbit_long_duration.cpp)
setup_fd_example(orbit_long_duration)

add_executable(racetrack_approach_diag racetrack_approach_diag.cpp)
setup_fd_example(racetrack_approach_diag)

add_executable(racetrack_approach_trace racetrack_approach_trace.cpp)
setup_fd_example(racetrack_approach_trace)

add_executable(figure8_approach_trace figure8_approach_trace.cpp)
setup_fd_example(figure8_approach_trace)
