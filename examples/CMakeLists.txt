# 示例程序构建入口

if(NOT TARGET ${PROJECT_ALIAS})
    message(FATAL_ERROR "${PROJECT_ALIAS} target not found. Ensure src/CMakeLists.txt is processed first")
endif()

function(add_domain_example name dir)
    add_executable(${name} "${dir}/session_usage.cpp")
    target_link_libraries(${name} PRIVATE ${PROJECT_ALIAS})
    target_include_directories(${name} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
endfunction()

add_domain_example(airborne_radar_session_usage airborne_radar)
add_domain_example(electro_optical_sensor_session_usage electro_optical)
add_domain_example(electronic_surveillance_radar_session_usage electronic_warfare)
add_domain_example(sar_session_usage sar)
add_executable(ar_config_compare_test airborne_radar/config_compare_test.cpp)
target_link_libraries(ar_config_compare_test PRIVATE ${PROJECT_ALIAS})
target_include_directories(ar_config_compare_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/airborne_radar")

# 集成模式示例：展示 RadarModule 在外部引擎中的使用方式。
add_executable(radar_integration_demo airborne_radar/integration_demo.cpp)
target_link_libraries(radar_integration_demo PRIVATE ${PROJECT_ALIAS})
target_include_directories(radar_integration_demo PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/airborne_radar")

# EOS 集成模式示例：展示 EosModule 在外部引擎中的使用方式。
add_executable(eos_integration_demo electro_optical/integration_demo.cpp)
target_link_libraries(eos_integration_demo PRIVATE ${PROJECT_ALIAS})
target_include_directories(eos_integration_demo PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/electro_optical")

# ESR 集成模式示例：展示 EsrModule 在外部引擎中的使用方式。
add_executable(esr_integration_demo electronic_warfare/integration_demo.cpp)
target_link_libraries(esr_integration_demo PRIVATE ${PROJECT_ALIAS})
target_include_directories(esr_integration_demo PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/electronic_warfare")

# SAR 集成模式示例：展示 SarModule 在外部引擎中的使用方式。
add_executable(sar_integration_demo sar/integration_demo.cpp)
target_link_libraries(sar_integration_demo PRIVATE ${PROJECT_ALIAS})
target_include_directories(sar_integration_demo PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/sar")

# flight_dynamic 示例：依赖机动模块，仅在 ONEQ_ENABLE_FLIGHT_DYNAMIC 开启时构建。
if(ONEQ_ENABLE_FLIGHT_DYNAMIC)

# Shared config for flight_dynamic example targets
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

add_executable(takeoff_land_csv flight_dynamic/takeoff_land_csv.cpp)
setup_fd_example(takeoff_land_csv)

add_executable(maneuver_sweep_csv flight_dynamic/maneuver_sweep_csv.cpp)
setup_fd_example(maneuver_sweep_csv)

add_executable(orbit_quality_csv flight_dynamic/orbit_quality_csv.cpp)
setup_fd_example(orbit_quality_csv)

add_executable(racetrack_quality_csv flight_dynamic/racetrack_quality_csv.cpp)
setup_fd_example(racetrack_quality_csv)

add_executable(racetrack_trace_csv flight_dynamic/racetrack_trace_csv.cpp)
setup_fd_example(racetrack_trace_csv)

add_executable(sturn_trace_csv flight_dynamic/sturn_trace_csv.cpp)
setup_fd_example(sturn_trace_csv)

add_executable(orbit_trace_csv flight_dynamic/orbit_trace_csv.cpp)
setup_fd_example(orbit_trace_csv)

add_executable(orbit_long_duration flight_dynamic/orbit_long_duration.cpp)
setup_fd_example(orbit_long_duration)

add_executable(racetrack_approach_diag flight_dynamic/racetrack_approach_diag.cpp)
setup_fd_example(racetrack_approach_diag)

add_executable(racetrack_approach_trace flight_dynamic/racetrack_approach_trace.cpp)
setup_fd_example(racetrack_approach_trace)

add_executable(figure8_approach_trace flight_dynamic/figure8_approach_trace.cpp)
setup_fd_example(figure8_approach_trace)

endif()  # ONEQ_ENABLE_FLIGHT_DYNAMIC
