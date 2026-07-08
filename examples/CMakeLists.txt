# 示例程序构建入口

if(NOT TARGET ${PROJECT_ALIAS})
    message(FATAL_ERROR "${PROJECT_ALIAS} target not found. Ensure src/CMakeLists.txt is processed first")
endif()

# examples 层共享工具（如 JSON 配置解析）。这些不属于 oneq 库的 public surface，
# 库内部不消费 JSON；JSON 解析是 example 便利层的能力（见 OQ-7 收口记录）。
set(ONEQ_EXAMPLE_COMMON_DIR "${CMAKE_CURRENT_SOURCE_DIR}/common")
set(ONEQ_EXAMPLE_COMMON_SOURCES
    "${ONEQ_EXAMPLE_COMMON_DIR}/json_reader.cpp")
function(_oneq_example_link_common target)
    target_sources(${target} PRIVATE ${ONEQ_EXAMPLE_COMMON_SOURCES})
    target_include_directories(${target} PRIVATE "${ONEQ_EXAMPLE_COMMON_DIR}")
endfunction()

function(add_domain_example name dir)
    add_executable(${name} "${dir}/session_usage.cpp")
    target_link_libraries(${name} PRIVATE ${PROJECT_ALIAS})
    target_include_directories(${name} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    _oneq_example_link_common(${name})
endfunction()

add_domain_example(airborne_radar_session_usage airborne_radar)
add_domain_example(electro_optical_sensor_session_usage electro_optical)
add_domain_example(electronic_surveillance_radar_session_usage electronic_warfare)
add_domain_example(sar_session_usage sar)
add_executable(ar_config_compare_test airborne_radar/config_compare_test.cpp)
target_link_libraries(ar_config_compare_test PRIVATE ${PROJECT_ALIAS})
target_include_directories(ar_config_compare_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/airborne_radar")
_oneq_example_link_common(ar_config_compare_test)

# 集成模式示例：展示 RadarModule 在外部引擎中的使用方式。
add_executable(radar_integration_demo airborne_radar/integration_demo.cpp)
target_link_libraries(radar_integration_demo PRIVATE ${PROJECT_ALIAS})
target_include_directories(radar_integration_demo PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/airborne_radar")
_oneq_example_link_common(radar_integration_demo)

# EOS 集成模式示例：展示 EosModule 在外部引擎中的使用方式。
add_executable(eos_integration_demo electro_optical/integration_demo.cpp)
target_link_libraries(eos_integration_demo PRIVATE ${PROJECT_ALIAS})
target_include_directories(eos_integration_demo PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/electro_optical")
_oneq_example_link_common(eos_integration_demo)

# ESR 集成模式示例：展示 EsrModule 在外部引擎中的使用方式。
add_executable(esr_integration_demo electronic_warfare/integration_demo.cpp)
target_link_libraries(esr_integration_demo PRIVATE ${PROJECT_ALIAS})
target_include_directories(esr_integration_demo PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/electronic_warfare")
_oneq_example_link_common(esr_integration_demo)

# SAR 集成模式示例：展示 SarModule 在外部引擎中的使用方式。
add_executable(sar_integration_demo sar/integration_demo.cpp)
target_link_libraries(sar_integration_demo PRIVATE ${PROJECT_ALIAS})
target_include_directories(sar_integration_demo PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/sar")
_oneq_example_link_common(sar_integration_demo)

# =============================================================================
# 端到端场景示例：多目标对抗剧本 + 真实 session 跑多周期，作为模块级冒烟。
# 与 integration_demo（API 教程）定位不同——这里搭具体战术剧本，看航迹/检测输出。
# 配置复用 examples/configs/（与 batch_validation 同源），通过 SCENE_CONFIG_DIR 注入。
# =============================================================================
function(add_domain_scene name module_dir src)
    add_executable(${name} "${module_dir}/${src}")
    target_link_libraries(${name} PRIVATE ${PROJECT_ALIAS})
    target_include_directories(${name} PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/${module_dir}")
    target_compile_definitions(${name} PRIVATE
        SCENE_CONFIG_DIR="${CMAKE_SOURCE_DIR}/examples/configs")
    _oneq_example_link_common(${name})
endfunction()

add_domain_scene(airborne_radar_scene airborne_radar airborne_radar_scene.cpp)
add_domain_scene(electro_optical_scene electro_optical electro_optical_scene.cpp)
add_domain_scene(electronic_warfare_scene electronic_warfare electronic_warfare_scene.cpp)

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

# =============================================================================
# 批量场景验证：多场景参数扫描 + CSV 指标 + trace 录制/回放回归。
# 每个模块一个独立可执行程序，证明模块在多场景下的泛用性。
# 共享工具头文件位于 batch_validation/，配置目录通过 BATCH_CONFIG_DIR 注入。
# =============================================================================
function(add_batch_validation name module_dir src)
    add_executable(${name} "batch_validation/${src}")
    target_link_libraries(${name} PRIVATE ${PROJECT_ALIAS})
    target_include_directories(${name} PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/batch_validation"
        "${CMAKE_CURRENT_SOURCE_DIR}/${module_dir}")
    target_compile_definitions(${name} PRIVATE
        BATCH_CONFIG_DIR="${CMAKE_SOURCE_DIR}/examples/configs")
    _oneq_example_link_common(${name})
endfunction()

add_batch_validation(ar_batch_validation airborne_radar ar_batch_validation.cpp)
add_batch_validation(eos_batch_validation electro_optical eos_batch_validation.cpp)
add_batch_validation(esr_batch_validation electronic_warfare esr_batch_validation.cpp)
add_batch_validation(sar_batch_validation sar sar_batch_validation.cpp)
