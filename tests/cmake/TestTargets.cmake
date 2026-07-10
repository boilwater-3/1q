# Test executable source ownership and aggregate build targets.
#
# Phase 2: the legacy 1q_unit_tests executable is replaced by per-domain unit
# partitions defined in partitions/Unit.cmake. 1q_unit_tests becomes a custom
# aggregate target. The FD executable (1q_fd_tests) is replaced by the
# 1q_flight_dynamic_unit_tests partition. replay_fast/integration/contract/
# performance remain legacy executables in this phase; Phase 3 converts them to
# partitions and aggregate targets.

# --- unit partitions -------------------------------------------------------
include(${CMAKE_CURRENT_LIST_DIR}/partitions/Unit.cmake)

# 1q_unit_tests: aggregate target depending on all enabled unit partitions.
add_custom_target(${PROJECT_NAME}_unit_tests)
set(_oneq_unit_partition_targets
    ${PROJECT_NAME}_common_unit_tests
    ${PROJECT_NAME}_examples_unit_tests
    ${PROJECT_NAME}_airborne_radar_unit_tests
    ${PROJECT_NAME}_electronic_surveillance_radar_unit_tests
    ${PROJECT_NAME}_electro_optical_sensor_unit_tests
    ${PROJECT_NAME}_sbirs_sensor_unit_tests
    ${PROJECT_NAME}_sar_unit_tests
    ${PROJECT_NAME}_flight_dynamic_unit_tests)
foreach(_p IN LISTS _oneq_unit_partition_targets)
    if(TARGET ${_p})
        add_dependencies(${PROJECT_NAME}_unit_tests ${_p})
    endif()
endforeach()
unset(_oneq_unit_partition_targets)

# 1q_fd_tests: alias kept for legacy callers; points at the FD unit partition.
if(TARGET ${PROJECT_NAME}_flight_dynamic_unit_tests AND NOT TARGET ${PROJECT_NAME}_fd_tests)
    add_custom_target(${PROJECT_NAME}_fd_tests)
    add_dependencies(${PROJECT_NAME}_fd_tests ${PROJECT_NAME}_flight_dynamic_unit_tests)
endif()

# --- performance (legacy executable, Phase 3 converts to partition) --------
file(GLOB_RECURSE PERFORMANCE_TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/performance/*.cpp")
add_1q_gtest(${PROJECT_NAME}_performance_tests performance 180 ${PERFORMANCE_TEST_SOURCES})
oneq_register_test_sources(performance ${PERFORMANCE_TEST_SOURCES})

# --- integration (legacy executable, Phase 3 converts to partition) --------
file(GLOB_RECURSE INTEGRATION_TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/integration/*.cpp")
add_1q_gtest(${PROJECT_NAME}_integration_tests integration 120 ${INTEGRATION_TEST_SOURCES})
oneq_register_test_sources(integration ${INTEGRATION_TEST_SOURCES})

# --- contract compiled (legacy executable, Phase 3 converts to partition) --
file(GLOB_RECURSE CONTRACT_TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/contract/*.cpp")
add_1q_gtest(${PROJECT_NAME}_contract_tests contract 60 ${CONTRACT_TEST_SOURCES})
oneq_register_test_sources(contract_compiled ${CONTRACT_TEST_SOURCES})

# --- replay-fast (legacy executable, Phase 3 converts to partition) --------
# The replay overlap allowlist in TestRegistry.cmake documents the sources that
# are still compiled into both a unit partition and this replay_fast target.
set(REPLAY_FAST_TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/replay/common/replay_trace_writer_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/replay/airborne_radar/ar_trace_session_adapter_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/replay/airborne_radar/ar_replay_codec_roundtrip_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/electro_optical_sensor/eos_replay_codec_roundtrip_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/electro_optical_sensor/eos_replay_session_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/electronic_surveillance_radar/esr_replay_codec_roundtrip_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/electronic_surveillance_radar/esr_replay_session_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/sar/sar_replay_codec_roundtrip_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/sar/sar_replay_session_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/sbirs_sensor/sbirs_replay_codec_roundtrip_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/replay/common/replay_trace_compression_test.cpp)
add_1q_gtest(${PROJECT_NAME}_replay_fast_tests replay_fast 90 ${REPLAY_FAST_TEST_SOURCES})
oneq_register_test_sources(replay_fast ${REPLAY_FAST_TEST_SOURCES})
if(TARGET oneq_flatbuffers_headers)
    add_dependencies(${PROJECT_NAME}_replay_fast_tests oneq_flatbuffers_headers)
endif()

# --- aggregate build targets ----------------------------------------------
add_custom_target(${PROJECT_NAME}_tests)
set(_oneq_test_deps
    ${PROJECT_NAME}_unit_tests
    ${PROJECT_NAME}_performance_tests
    ${PROJECT_NAME}_integration_tests
    ${PROJECT_NAME}_contract_tests)
add_dependencies(${PROJECT_NAME}_tests ${_oneq_test_deps})
unset(_oneq_test_deps)

add_custom_target(${PROJECT_NAME}_tests_fast)
add_dependencies(${PROJECT_NAME}_tests_fast ${PROJECT_NAME}_replay_fast_tests)
