# Unit-type test partitions: one executable per (type=unit, domain).
#
# Phase 2: splits the legacy 1q_unit_tests aggregate executable into per-domain
# 1q_<domain>_unit_tests partitions. The legacy 1q_unit_tests name becomes a
# custom aggregate target depending on every enabled unit partition (set up by
# the caller after including this file).
#
# Source discovery: each partition globs exactly its own type/domain directory
# (tests/unit/<domain>/*_test.cpp). This lets test authors drop a new file in
# without editing a central source list. The registry finalize step proves
# every _test.cpp is owned by exactly one partition.
#
# Note on replay overlap: trace session adapter, replay session and trace
# writer/compression sources currently live under tests/unit/<domain>/ and are
# also compiled into 1q_replay_fast_tests until Phase 3 moves them to
# replay/<domain>/ and removes the duplication. Codec roundtrip sources remain
# excluded from unit partitions, matching the legacy 1q_unit_tests membership.
# The registry's ONEQ_TEST_OVERLAP_ALLOWLIST documents this transition state.

function(_oneq_exclude_legacy_replay_codec_sources out_var)
    set(_sources ${ARGN})
    list(FILTER _sources EXCLUDE REGEX "_replay_codec_roundtrip_test\\.cpp$")
    set(${out_var} "${_sources}" PARENT_SCOPE)
endfunction()

# common: coordinate/estimation/numeric/atmosphere + replay trace writer/compression.
file(GLOB _oneq_unit_common CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/common/*_test.cpp")
if(_oneq_unit_common)
    oneq_add_test_partition(
        TYPE unit DOMAIN common
        SOURCES ${_oneq_unit_common}
        TIMEOUT 60)
endif()

# examples: JsonReader test links examples/common/json_reader.cpp implementation.
file(GLOB _oneq_unit_examples CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/examples/*_test.cpp")
if(_oneq_unit_examples)
    # component_attachment 组件实现（ecs_component_runtime_test 测组件运行时
    # 修改接口；sensor_adapt.h 为 header-only）。
    # demo_log.cpp：组件源文件内 CA_LOG_EVENT / CA_LOG_VIEW 宏引用符号（单测
    # 不初始化 → 未初始化静默跳过路径）。集成端日志直接使用 spdlog：仅在
    # PROJECT_ENABLE_SPDLOG 时编译组件源并链接 spdlog；Windows 上该示例门控
    # 关闭，组件运行时测试一并排除（JsonReader / ECS 核心测试保留，二者无
    # spdlog 依赖）。
    set(_oneq_examples_extra
        "${CMAKE_SOURCE_DIR}/examples/common/json_reader.cpp")
    if(PROJECT_ENABLE_SPDLOG)
        list(APPEND _oneq_examples_extra
            "${CMAKE_SOURCE_DIR}/examples/component_attachment/scene_data.cpp"
            "${CMAKE_SOURCE_DIR}/examples/component_attachment/demo_config.cpp"
            "${CMAKE_SOURCE_DIR}/examples/component_attachment/scene_script.cpp"
            "${CMAKE_SOURCE_DIR}/examples/component_attachment/components/ar_sensor_component.cpp"
            "${CMAKE_SOURCE_DIR}/examples/component_attachment/components/esr_sensor_component.cpp"
            "${CMAKE_SOURCE_DIR}/examples/component_attachment/components/eos_sensor_component.cpp"
            "${CMAKE_SOURCE_DIR}/examples/component_attachment/components/sbirs_sensor_component.cpp"
            "${CMAKE_SOURCE_DIR}/examples/component_attachment/components/sar_sensor_component.cpp"
            "${CMAKE_SOURCE_DIR}/examples/component_attachment/components/flight_component.cpp"
            "${CMAKE_SOURCE_DIR}/examples/component_attachment/components/demo_log.cpp")
        set(_oneq_examples_link_libs ${PROJECT_SPDLOG_TARGET})
    else()
        # 示例整体门控（spdlog 非 Windows 依赖）：组件运行时测试、场景数据与
        # 场景脚本测试随示例一起排除（Windows 上不测试）。
        list(FILTER _oneq_unit_examples
             EXCLUDE REGEX "ecs_component_runtime_test\\.cpp$|scene_data_test\\.cpp$|scene_script_test\\.cpp$")
    endif()
    oneq_add_test_partition(
        TYPE unit DOMAIN examples
        SOURCES ${_oneq_unit_examples}
        TIMEOUT 60
        INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/examples/common"
                     "${CMAKE_SOURCE_DIR}/examples/batch_validation"
                     "${CMAKE_SOURCE_DIR}/examples/component_attachment"
        EXTRA_SOURCES ${_oneq_examples_extra}
        LINK_LIBS ${_oneq_examples_link_libs})
    # demo_config.cpp（场景覆写应用）需要 examples/configs 路径宏，与
    # component_attachment demo 目标同源注入。
    target_compile_definitions(${PROJECT_NAME}_examples_unit_tests PRIVATE
        SCENE_CONFIG_DIR="${CMAKE_SOURCE_DIR}/examples/configs")
    if(ONEQ_ENABLE_FLIGHT_DYNAMIC)
        # 飞行组件 FD 路径（与 examples/component_attachment/CMakeLists.txt 对称）：
        # 静态库不传递依赖，需显式链接 JSBSim；c172x 数据根注入。
        if(DEFINED ONEQ_JSBSIM_DATA_ROOT_DIR AND NOT ONEQ_JSBSIM_DATA_ROOT_DIR STREQUAL "")
            set(_oneq_examples_fd_root "${ONEQ_JSBSIM_DATA_ROOT_DIR}")
        else()
            set(_oneq_examples_fd_root "${CMAKE_SOURCE_DIR}/third_party/jsbsim")
        endif()
        target_compile_definitions(${PROJECT_NAME}_examples_unit_tests PRIVATE
            ONEQ_CA_FLIGHT_DYNAMIC_ENABLED=1
            FD_JSBSIM_ROOT_DIR="${_oneq_examples_fd_root}")
        if(TARGET JSBSim::JSBSim)
            target_link_libraries(${PROJECT_NAME}_examples_unit_tests PRIVATE JSBSim::JSBSim)
        endif()
    endif()
endif()

# airborne_radar (AR).
file(GLOB _oneq_unit_airborne_radar CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/airborne_radar/*_test.cpp")
_oneq_exclude_legacy_replay_codec_sources(_oneq_unit_airborne_radar
    ${_oneq_unit_airborne_radar})
if(_oneq_unit_airborne_radar)
    oneq_add_test_partition(
        TYPE unit DOMAIN airborne_radar
        SOURCES ${_oneq_unit_airborne_radar}
        TIMEOUT 60
        LINK_LIBS SQLite::SQLite3
        INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/unit/airborne_radar"
                     "${CMAKE_CURRENT_BINARY_DIR}/generated")
endif()

# electronic_surveillance_radar (ESR).
file(GLOB _oneq_unit_electronic_surveillance_radar CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/electronic_surveillance_radar/*_test.cpp")
_oneq_exclude_legacy_replay_codec_sources(_oneq_unit_electronic_surveillance_radar
    ${_oneq_unit_electronic_surveillance_radar})
if(_oneq_unit_electronic_surveillance_radar)
    oneq_add_test_partition(
        TYPE unit DOMAIN electronic_surveillance_radar
        SOURCES ${_oneq_unit_electronic_surveillance_radar}
        TIMEOUT 60)
endif()

# electronic_countermeasure (ECM).
file(GLOB _oneq_unit_electronic_countermeasure CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/electronic_countermeasure/*_test.cpp")
if(_oneq_unit_electronic_countermeasure)
    oneq_add_test_partition(
        TYPE unit DOMAIN electronic_countermeasure
        SOURCES ${_oneq_unit_electronic_countermeasure}
        TIMEOUT 60)
endif()

# navigation.
file(GLOB _oneq_unit_navigation CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/navigation/*_test.cpp")
if(_oneq_unit_navigation)
    oneq_add_test_partition(
        TYPE unit DOMAIN navigation
        SOURCES ${_oneq_unit_navigation}
        TIMEOUT 60)
endif()

# fusion.
file(GLOB _oneq_unit_fusion CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/fusion/*_test.cpp")
if(_oneq_unit_fusion)
    oneq_add_test_partition(
        TYPE unit DOMAIN fusion
        SOURCES ${_oneq_unit_fusion}
        TIMEOUT 60)
endif()

# electro_optical_sensor (EOS).
file(GLOB _oneq_unit_electro_optical_sensor CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/electro_optical_sensor/*_test.cpp")
_oneq_exclude_legacy_replay_codec_sources(_oneq_unit_electro_optical_sensor
    ${_oneq_unit_electro_optical_sensor})
if(_oneq_unit_electro_optical_sensor)
    oneq_add_test_partition(
        TYPE unit DOMAIN electro_optical_sensor
        SOURCES ${_oneq_unit_electro_optical_sensor}
        TIMEOUT 60)
endif()

# sbirs_sensor (SBIRS).
file(GLOB _oneq_unit_sbirs_sensor CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/sbirs_sensor/*_test.cpp")
_oneq_exclude_legacy_replay_codec_sources(_oneq_unit_sbirs_sensor
    ${_oneq_unit_sbirs_sensor})
if(_oneq_unit_sbirs_sensor)
    oneq_add_test_partition(
        TYPE unit DOMAIN sbirs_sensor
        SOURCES ${_oneq_unit_sbirs_sensor}
        TIMEOUT 60)
endif()

# sar.
file(GLOB _oneq_unit_sar CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/sar/*_test.cpp")
_oneq_exclude_legacy_replay_codec_sources(_oneq_unit_sar
    ${_oneq_unit_sar})
if(_oneq_unit_sar)
    oneq_add_test_partition(
        TYPE unit DOMAIN sar
        SOURCES ${_oneq_unit_sar}
        TIMEOUT 60
        LABELS ci_required)
endif()
