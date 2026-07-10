# Test executable source ownership and aggregate build targets.
#
# Phase 0 transition: the legacy aggregate executables below are unchanged, but
# each source batch is also recorded via oneq_register_test_sources() so the
# Phase 0 registry can detect orphans/duplicates. Partition keys here are the
# legacy target names; Phase 2/3 replace them with 1q_<domain>_<type>_tests.

set(FD_TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/flight_dynamic/fd_aircraft_performance_derivation_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/flight_dynamic/fd_angle_normalization_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/flight_dynamic/fd_adapter_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/flight_dynamic/fd_aircraft_maneuver_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/flight_dynamic/fd_aircraft_probe_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/flight_dynamic/fd_bare_aircraft_baseline_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/flight_dynamic/fd_orbit_quality_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/flight_dynamic/fd_robustness_test.cpp)
list(REMOVE_ITEM UNIT_TEST_SOURCES ${FD_TEST_SOURCES})

set(REPLAY_CODEC_ROUNDTRIP_TEST_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/airborne_radar/ar_replay_codec_roundtrip_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/electro_optical_sensor/eos_replay_codec_roundtrip_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/electronic_surveillance_radar/esr_replay_codec_roundtrip_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/sar/sar_replay_codec_roundtrip_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/sbirs_sensor/sbirs_replay_codec_roundtrip_test.cpp")
list(REMOVE_ITEM UNIT_TEST_SOURCES ${REPLAY_CODEC_ROUNDTRIP_TEST_SOURCES})

add_1q_gtest(${PROJECT_NAME}_unit_tests unit 60 ${UNIT_TEST_SOURCES})
oneq_register_test_sources(unit ${UNIT_TEST_SOURCES})
target_sources(${PROJECT_NAME}_unit_tests PRIVATE
    "${CMAKE_SOURCE_DIR}/examples/common/json_reader.cpp")
target_include_directories(${PROJECT_NAME}_unit_tests PRIVATE
    "${CMAKE_SOURCE_DIR}/examples/common")

if(ONEQ_ENABLE_FLIGHT_DYNAMIC)
    add_1q_gtest(${PROJECT_NAME}_fd_tests fd 180 ${FD_TEST_SOURCES})
endif()
# Registry ownership is tracked unconditionally so the orphan check stays
# consistent whether or not FD is enabled; only the executable is gated.
oneq_register_test_sources(fd ${FD_TEST_SOURCES})
add_1q_gtest(${PROJECT_NAME}_performance_tests performance 180 ${PERFORMANCE_TEST_SOURCES})
oneq_register_test_sources(performance ${PERFORMANCE_TEST_SOURCES})
add_1q_gtest(${PROJECT_NAME}_integration_tests integration 120 ${INTEGRATION_TEST_SOURCES})
oneq_register_test_sources(integration ${INTEGRATION_TEST_SOURCES})
add_1q_gtest(${PROJECT_NAME}_contract_tests contract 60 ${CONTRACT_TEST_SOURCES})
oneq_register_test_sources(contract_compiled ${CONTRACT_TEST_SOURCES})

set(REPLAY_FAST_TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/common/replay_trace_writer_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/airborne_radar/ar_trace_session_adapter_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/airborne_radar/ar_replay_codec_roundtrip_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/electro_optical_sensor/eos_replay_codec_roundtrip_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/electro_optical_sensor/eos_replay_session_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/electronic_surveillance_radar/esr_replay_codec_roundtrip_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/electronic_surveillance_radar/esr_replay_session_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/sar/sar_replay_codec_roundtrip_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/sar/sar_replay_session_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/sbirs_sensor/sbirs_replay_codec_roundtrip_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/common/replay_trace_compression_test.cpp)
add_1q_gtest(${PROJECT_NAME}_replay_fast_tests replay_fast 90 ${REPLAY_FAST_TEST_SOURCES})
oneq_register_test_sources(replay_fast ${REPLAY_FAST_TEST_SOURCES})
if(TARGET oneq_flatbuffers_headers)
    add_dependencies(${PROJECT_NAME}_replay_fast_tests oneq_flatbuffers_headers)
endif()

add_custom_target(${PROJECT_NAME}_tests)
set(_oneq_test_deps
    ${PROJECT_NAME}_unit_tests
    ${PROJECT_NAME}_performance_tests
    ${PROJECT_NAME}_integration_tests
    ${PROJECT_NAME}_contract_tests)
if(TARGET ${PROJECT_NAME}_fd_tests)
    list(APPEND _oneq_test_deps ${PROJECT_NAME}_fd_tests)
endif()
add_dependencies(${PROJECT_NAME}_tests ${_oneq_test_deps})
unset(_oneq_test_deps)

add_custom_target(${PROJECT_NAME}_tests_fast)
add_dependencies(${PROJECT_NAME}_tests_fast ${PROJECT_NAME}_replay_fast_tests)
