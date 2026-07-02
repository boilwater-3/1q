if(NOT ENABLE_TESTING)
    return()
endif()

if(MSVC AND MSVC_VERSION LESS 1910)
    set(CMAKE_MAP_IMPORTED_CONFIG_DEBUG Release)
    set(CMAKE_MAP_IMPORTED_CONFIG_RELWITHDEBINFO Release)
    set(CMAKE_MAP_IMPORTED_CONFIG_MINSIZEREL Release)
endif()

if(PACKAGE_MANAGER STREQUAL "conan")
    find_package(GTest CONFIG REQUIRED)
    find_package(Boost CONFIG REQUIRED)
    find_package(Eigen3 CONFIG REQUIRED)
    find_package(flatbuffers CONFIG REQUIRED)
    find_package(ZLIB QUIET)
endif()
if(NOT DEFINED flatbuffers_INCLUDE_DIRS_RELEASE)
    # Pull Conan-generated include variables without importing flatc target.
    file(GLOB _oneq_flatbuffers_data_candidates
        "${CMAKE_BINARY_DIR}/build/generators/flatbuffers-*-data.cmake")
    foreach(_oneq_flatbuffers_data IN LISTS _oneq_flatbuffers_data_candidates)
        if(NOT _oneq_flatbuffers_data MATCHES "module-flatbuffers")
            include("${_oneq_flatbuffers_data}")
            break()
        endif()
    endforeach()
    unset(_oneq_flatbuffers_data)
    unset(_oneq_flatbuffers_data_candidates)
endif()

file(GLOB_RECURSE UNIT_TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/*.cpp"
)
file(GLOB_RECURSE INTEGRATION_TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/integration/*.cpp"
)
file(GLOB_RECURSE CONTRACT_TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/contract/*.cpp"
)
file(GLOB_RECURSE PERFORMANCE_TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/performance/*.cpp"
)

set(TEST_SOURCES
    ${UNIT_TEST_SOURCES}
    ${INTEGRATION_TEST_SOURCES}
    ${CONTRACT_TEST_SOURCES}
)

if(NOT TEST_SOURCES)
    message(STATUS "No test sources defined in ${CMAKE_CURRENT_SOURCE_DIR}")
    message(STATUS "  Add .cpp files to unit/integration/contract to enable testing")
    return()
endif()

include(GoogleTest)
function(add_1q_gtest target_name suite_label discovery_timeout)
    if(NOT ARGN)
        return()
    endif()

    add_executable(${target_name} ${ARGN})
    target_link_libraries(${target_name} PRIVATE
        GTest::gtest_main
        GTest::gmock
        Boost::boost
        Eigen3::Eigen
        ${PROJECT_ALIAS}
    )
    if(TARGET ZLIB::ZLIB)
        target_link_libraries(${target_name} PRIVATE ZLIB::ZLIB)
    endif()
    if(TARGET flatbuffers::flatbuffers)
        target_link_libraries(${target_name} PRIVATE flatbuffers::flatbuffers)
    endif()
    target_include_directories(${target_name} PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
    if(DEFINED flatbuffers_INCLUDE_DIRS_RELEASE)
        # Conan multi-config can omit header-only include dirs for tests.
        # Mirror Release include dirs so flexbuffers headers are always resolvable.
        target_include_directories(${target_name} PRIVATE
            ${flatbuffers_INCLUDE_DIRS_RELEASE}
        )
    endif()
    if(WIN32)
        # flatbuffers/1.12 header-only mode on VS2015 needs this to avoid
        # unresolved ClassicLocale symbols from util.h.
        target_compile_definitions(${target_name} PRIVATE
            FLATBUFFERS_LOCALE_INDEPENDENT=0
        )
    endif()

    # Propagate ONEQ_HAVE_ZLIB so compression test guards resolve the same as in the library.
    if(TARGET ZLIB::ZLIB)
        target_compile_definitions(${target_name} PRIVATE ONEQ_HAVE_ZLIB=1)
        if(DEFINED zlib_INCLUDE_DIRS_RELEASE)
            target_include_directories(${target_name} PRIVATE ${zlib_INCLUDE_DIRS_RELEASE})
        endif()
    else()
        target_compile_definitions(${target_name} PRIVATE ONEQ_HAVE_ZLIB=0)
    endif()

    if(ENABLE_PCH)
        target_precompile_headers(${target_name} PRIVATE
            <vector>
            <string>
            <array>
            <map>
            <set>
            <unordered_map>
            <memory>
            <algorithm>
            <utility>
            <cmath>
            <cstdint>
        )
    endif()

    if(WIN32 AND BUILD_SHARED_LIBS)
        add_custom_command(TARGET ${target_name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:${PROJECT_CORE_TARGET}>
                $<TARGET_FILE_DIR:${target_name}>
            COMMENT "Copying ${PROJECT_CORE_TARGET} DLL to test directory"
        )
    endif()

    add_test(NAME "${suite_label}::${target_name}" COMMAND ${target_name})
    set_tests_properties("${suite_label}::${target_name}" PROPERTIES LABELS "${suite_label}")
endfunction()

# --------------------------------------------------------------------------
# flight_dynamic 测试（含 JSBSim）独立二进制，可并行构建与执行
# --------------------------------------------------------------------------
set(FD_TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/fd_aircraft_performance_derivation_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/fd_angle_normalization_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/fd_adapter_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/fd_aircraft_maneuver_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/fd_aircraft_probe_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/fd_bare_aircraft_baseline_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/fd_orbit_quality_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/fd_robustness_test.cpp
)

# 从单元测试源列表中排除 FD 测试
list(REMOVE_ITEM UNIT_TEST_SOURCES ${FD_TEST_SOURCES})

add_1q_gtest(${PROJECT_NAME}_unit_tests unit 60 ${UNIT_TEST_SOURCES})
if(ONEQ_ENABLE_FLIGHT_DYNAMIC)
    add_1q_gtest(${PROJECT_NAME}_fd_tests fd 180 ${FD_TEST_SOURCES})
endif()
add_1q_gtest(${PROJECT_NAME}_performance_tests performance 180 ${PERFORMANCE_TEST_SOURCES})

# --------------------------------------------------------------------------
# flight_dynamic 测试五层分类：smoke / contract / controllability /
# performance / known-limit。每个 CTest 入口运行同一二进制，
# 使用不同 gtest_filter 和 label。CI 默认运行 smoke+contract+
# controllability；performance 和 known-limit 按需启用。
#
# 用法：
#   ctest --preset <p> -L fd_smoke            # Tier 1: 快速冒烟
#   ctest --preset <p> -L fd_contract          # Tier 2: 合同快照
#   ctest --preset <p> -L fd_controllability   # Tier 3: 基本可控
#   ctest --preset <p> -L fd_performance       # Tier 4: 多机型性能
#   ctest --preset <p> -L fd_known_limit       # Tier 5: 已知限制
#   ctest --preset <p> -L fd_ci                # CI 默认 (1+2+3)
#   ctest --preset <p> -L fd                   # 全部
# --------------------------------------------------------------------------
if(ONEQ_ENABLE_FLIGHT_DYNAMIC)

# Tier 1: smoke — 库能否加载、初始化、单步运行？
# ~20 tests, <5s
set(_fd_smoke_filter
    "FlightDynamicTest.AdapterCreatesAndRuns:"
    "FlightDynamicTest.VehicleStatePopulated:"
    "FlightDynamicTest.InitialVelocityDefaultsToBodyFrame:"
    "FlightDynamicTest.AutopilotSetsHeading:"
    "FlightDynamicTest.AutopilotSetsAltitude:"
    "FlightDynamicTest.WaypointManagerAddsWaypoints:"
    "FlightDynamicTest.FlyToWaypointManeuver:"
    "FlightDynamicTest.OrbitManeuver:"
    "FlightDynamicTest.AbortManeuver:"
    "FlightDynamicTest.ResetAndReuse:"
        "*BareAircraftTest.ModelLoadsSuccessfully/*:"
    "*ProfileSnapshotTest.MatchesExpectedProfile/*"
)
string(REPLACE ";" "" _fd_smoke_filter_str "${_fd_smoke_filter}")
add_test(NAME "fd_smoke::1q_fd_tests"
         COMMAND ${PROJECT_NAME}_fd_tests --gtest_filter=${_fd_smoke_filter_str})
set_tests_properties("fd_smoke::1q_fd_tests" PROPERTIES LABELS "fd_smoke;fd_ci")

# Tier 2: contract — profile/IC/trim/XML 合同是否稳定？
# ~30 tests, <10s
set(_fd_contract_filter
    "FlightDynamicTest.AutopilotDetectsOwnApProfile:"
    "FlightDynamicTest.AutopilotDetectsFbwProfile:"
        "FlightDynamicTest.AutopilotDetectsC310Profile:"
    "FlightDynamicTest.AutopilotDetectsConcordeProfile:"
    "FlightDynamicTest.AutopilotWritesDetectedYawInputProperty:"
    "FlightDynamicTest.InitialVelocityCanUseEcefFrame:"
    "FlightDynamicTest.InitialConditionsAcceptEcefPosition:"
    "*BareAircraftTest.ProjectIcTrimOnPassesL1/*:"
    "*BareAircraftTest.ProjectIcTrimOffPassesL1/*:"
    "*BareAircraftDiagnosticTest.*:"
    "*BareAircraftTest.TrimBehaviorRecorded/*:"
    "*ResetXmlBaselineTest.ResetXmlFreeFlightStable/*"
)
string(REPLACE ";" "" _fd_contract_filter_str "${_fd_contract_filter}")
add_test(NAME "fd_contract::1q_fd_tests"
         COMMAND ${PROJECT_NAME}_fd_tests --gtest_filter=${_fd_contract_filter_str})
set_tests_properties("fd_contract::1q_fd_tests" PROPERTIES LABELS "fd_contract;fd_ci")

# Tier 3: controllability — 基本机动命令能否控制飞行器？边界情况？
# ~30 tests, <15s
set(_fd_ctrl_filter
    "FlightDynamicTest.MultipleManeuvers:"
    "FlightDynamicTest.SetPitchManeuver:"
    "FlightDynamicTest.SetRollManeuver:"
    "FlightDynamicTest.SetAltitudeManeuver:"
    "FlightDynamicRobustnessTest.*:"
    "*BareAircraftTest.ProjectIcTrimOnFreeFlightStable/*:"
    "*BareAircraftTest.ProjectIcTrimOffFreeFlightStable/*"
)
string(REPLACE ";" "" _fd_ctrl_filter_str "${_fd_ctrl_filter}")
add_test(NAME "fd_controllability::1q_fd_tests"
         COMMAND ${PROJECT_NAME}_fd_tests --gtest_filter=${_fd_ctrl_filter_str})
set_tests_properties("fd_controllability::1q_fd_tests" PROPERTIES LABELS "fd_controllability;fd_ci")

# Tier 4: performance — 稳定机型机动质量：距离收敛、高度变化、航向收敛、队列完成
# KnownLimitFighterModels removed (f22 deleted).
# ~90 tests, ~10s
set(_fd_perf_filter
    "*AircraftManeuverTest.FlyToWaypoint/*:"
    "*AircraftManeuverTest.SetAltitudeClimb/*:"
    "*AircraftManeuverTest.SetHeading/*:"
    "*AircraftManeuverTest.OrbitTimedCompletion/*:"
    "*AircraftManeuverTest.QueueOrbitThenHeading/*:"
    "*AircraftManeuverTest.QueueFlyToThenOrbit/*"
)
string(REPLACE ";" "" _fd_perf_filter_str "${_fd_perf_filter}")
string(APPEND _fd_perf_filter_str ":-*KnownLimitFighterModels*")
add_test(NAME "fd_performance::1q_fd_tests"
         COMMAND ${PROJECT_NAME}_fd_tests --gtest_filter=${_fd_perf_filter_str})
set_tests_properties("fd_performance::1q_fd_tests" PROPERTIES LABELS "fd_performance")

# Tier 5: known-limit — 长时间 orbit 稳定性、极端参数、env-gated 诊断
# F22 model removed, no longer needed.
# ~130 tests, ~5s (大部分快速)
set(_fd_limit_filter
    "*AircraftManeuverTest.SetRoll/*:"
    "*AircraftManeuverTest.ResetAndReuse/*:"
    "*AircraftManeuverTest.AbortManeuver/*:"
    "*AircraftManeuverTest.InvalidOrbitRadius/*:"
    "*OrbitUnitTest.*/*:"
    "*NewManeuverSmoke.*"
    "*FdAircraftProbe.*"
)
string(REPLACE ";" "" _fd_limit_filter_str "${_fd_limit_filter}")
add_test(NAME "fd_known_limit::1q_fd_tests"
         COMMAND ${PROJECT_NAME}_fd_tests --gtest_filter=${_fd_limit_filter_str})
set_tests_properties("fd_known_limit::1q_fd_tests" PROPERTIES LABELS "fd_known_limit")

# flight_dynamic 测试需要 JSBSim 数据文件路径
if(DEFINED ONEQ_JSBSIM_DATA_ROOT_DIR AND NOT ONEQ_JSBSIM_DATA_ROOT_DIR STREQUAL "")
    set(FD_JSBSIM_ROOT_DIR "${ONEQ_JSBSIM_DATA_ROOT_DIR}")
else()
    set(FD_JSBSIM_ROOT_DIR "${CMAKE_SOURCE_DIR}/third_party/jsbsim")
endif()
# flight_dynamic 测试需要链接 JSBSim（OBJECT 库的 PRIVATE 依赖不传递）
target_include_directories(${PROJECT_NAME}_fd_tests SYSTEM PRIVATE
    ${CMAKE_SOURCE_DIR}/third_party/jsbsim/src)
if(TARGET JSBSim::JSBSim)
  target_link_libraries(${PROJECT_NAME}_fd_tests PRIVATE JSBSim::JSBSim)
endif()
target_compile_definitions(${PROJECT_NAME}_fd_tests
    PRIVATE FD_JSBSIM_ROOT_DIR="${FD_JSBSIM_ROOT_DIR}")

endif()  # ONEQ_ENABLE_FLIGHT_DYNAMIC

add_1q_gtest(${PROJECT_NAME}_integration_tests integration 120 ${INTEGRATION_TEST_SOURCES})
add_1q_gtest(${PROJECT_NAME}_contract_tests contract 60 ${CONTRACT_TEST_SOURCES})

set(REPLAY_FAST_TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/replay_trace_writer_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/ar_trace_session_adapter_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/ar_replay_codec_roundtrip_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/eos_replay_codec_roundtrip_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/eos_replay_session_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/esr_replay_codec_roundtrip_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/esr_replay_session_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/sar_replay_codec_roundtrip_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/sar_replay_session_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/replay_trace_compression_test.cpp
)
add_1q_gtest(${PROJECT_NAME}_replay_fast_tests replay_fast 90 ${REPLAY_FAST_TEST_SOURCES})

# --------------------------------------------------------------------------
# SAR Phase 1/2 验收入口。Phase 2 性能项只记录 Sinc RCMC 独立代价，
# 不代表 Sinc 已批准成为 public Session 默认路径。
# --------------------------------------------------------------------------
set(_sar_unit_filter
    "SarFftBackendTest.*:"
    "SarSignalChainTest.*:"
    "SarWindowFunctionTest.*:"
    "SarGeometryTest.*:"
    "SarGeometryModelTest.*:"
    "SarAntennaPatternTest.*:"
    "SarImageOutputTest.*:"
    "SarEchoTest.*:"
    "SarEchoClutterTest.*:"
    "SarPgaSupportGradientTruthTest.*:"
    "SarPgaPhaseGradientEstimatorTest.*:"
    "SarPgaGradientTruthComparisonTest.*:"
    "SarAutofocusPhaseTruthTest.*:"
    "SarSlowTimeResamplingTest.*:"
    "SarRawHistorySlowTimeResamplingTest.*:"
    "SarVariablePrfQualityMatrixTest.*:"
    "SarExtendedVariablePrfQualityMatrixTest.*:"
    "SarMissingPulseGapDiagnosticsTest.*:"
    "SarMissingPulseRejectionMatrixTest.*:"
    "SarSlowTimeResamplingExecutorTest.*:"
    "PulseRingBufferTest.*:"
    "SarGbpTest.*:"
    "SarImageQualityTest.*:"
    "SarMotionCompensationTest.*:"
    "SarRdaTest.*:"
    "SarSessionPipelineTest.*:"
    "SarCycleInputAdapterBridgeTest.*:"
    "SarDegenerateDiagnosticsTest.*:"
    "SarRawHistoryExternalIqPredicateTest.*:"
    "SarReplayCodecRoundtripTest.*:"
    "SarReplaySessionTest.*:"
    "SarRuntimeConfigResolverTest.*"
)
string(REPLACE ";" "" _sar_unit_filter_str "${_sar_unit_filter}")
add_test(NAME "sar_unit::1q_unit_tests"
         COMMAND ${PROJECT_NAME}_unit_tests --gtest_filter=${_sar_unit_filter_str})
set_tests_properties("sar_unit::1q_unit_tests" PROPERTIES LABELS "sar_unit;sar_ci")

set(_sar_replay_filter
    "SarReplayCodecRoundtripTest.*:"
    "SarReplaySessionTest.*"
)
string(REPLACE ";" "" _sar_replay_filter_str "${_sar_replay_filter}")
add_test(NAME "sar_replay::1q_replay_fast_tests"
         COMMAND ${PROJECT_NAME}_replay_fast_tests --gtest_filter=${_sar_replay_filter_str})
set_tests_properties("sar_replay::1q_replay_fast_tests" PROPERTIES LABELS "sar_replay;sar_ci")

set(_sar_integration_filter
    "SarSessionPipelineTest.*:"
    "SarReplaySessionTest.*"
)
string(REPLACE ";" "" _sar_integration_filter_str "${_sar_integration_filter}")
add_test(NAME "sar_integration::1q_unit_tests"
         COMMAND ${PROJECT_NAME}_unit_tests --gtest_filter=${_sar_integration_filter_str})
set_tests_properties("sar_integration::1q_unit_tests"
                     PROPERTIES LABELS "sar_integration;sar_ci")

add_test(NAME "sar_contract::1q_contract_tests"
         COMMAND ${PROJECT_NAME}_contract_tests
                 --gtest_filter=PublicHeadersSmokeTest.SarPublicSurfaceSupportsMinimalUsage)
set_tests_properties("sar_contract::1q_contract_tests" PROPERTIES LABELS "sar_contract;sar_ci")

add_test(NAME "sar_frozen_sources"
         COMMAND ${CMAKE_COMMAND}
                 -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
                 -P ${CMAKE_CURRENT_SOURCE_DIR}/contract/check_sar_frozen_sources.cmake)
set_tests_properties("sar_frozen_sources" PROPERTIES LABELS "sar_contract;sar_ci")

add_test(NAME "sar_performance::1q_performance_tests"
         COMMAND ${PROJECT_NAME}_performance_tests
                 --gtest_filter=SarPerformanceTest.*)
set_tests_properties("sar_performance::1q_performance_tests" PROPERTIES LABELS "sar_performance")

get_target_property(_sar_eigen_include_dirs Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
list(GET _sar_eigen_include_dirs 0 _sar_eigen_include_dir)
if(MSVC)
    set(_sar_cxx11_runner "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/run_sar_cxx11_compat.bat")
    file(GENERATE OUTPUT "${_sar_cxx11_runner}" CONTENT
"@call \"${CMAKE_GENERATOR_INSTANCE}/VC/Auxiliary/Build/vcvars64.bat\" >nul
@\"${CMAKE_COMMAND}\" -DSOURCE_DIR=\"${CMAKE_SOURCE_DIR}\" -DCXX_COMPILER=\"${CMAKE_CXX_COMPILER}\" -DCXX_COMPILER_ID=MSVC -DEIGEN_INCLUDE_DIR=\"${_sar_eigen_include_dir}\" -P \"${CMAKE_CURRENT_SOURCE_DIR}/contract/check_sar_cxx11_compat.cmake\"
")
    add_test(NAME sar_cxx11_compat COMMAND "${_sar_cxx11_runner}")
else()
    add_test(
        NAME sar_cxx11_compat
        COMMAND ${CMAKE_COMMAND}
                -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
                -DCXX_COMPILER=${CMAKE_CXX_COMPILER}
                -DCXX_COMPILER_ID=${CMAKE_CXX_COMPILER_ID}
                -DEIGEN_INCLUDE_DIR=${_sar_eigen_include_dir}
                -P ${CMAKE_CURRENT_SOURCE_DIR}/contract/check_sar_cxx11_compat.cmake
    )
endif()
set_tests_properties(sar_cxx11_compat PROPERTIES LABELS "sar_cxx11_compat")

add_custom_target(${PROJECT_NAME}_tests)
set(_oneq_test_deps
    ${PROJECT_NAME}_unit_tests
    ${PROJECT_NAME}_performance_tests
    ${PROJECT_NAME}_integration_tests
    ${PROJECT_NAME}_contract_tests
)
if(TARGET ${PROJECT_NAME}_fd_tests)
    list(APPEND _oneq_test_deps ${PROJECT_NAME}_fd_tests)
endif()
add_dependencies(${PROJECT_NAME}_tests ${_oneq_test_deps})
unset(_oneq_test_deps)

add_custom_target(${PROJECT_NAME}_tests_fast)
add_dependencies(${PROJECT_NAME}_tests_fast
    ${PROJECT_NAME}_replay_fast_tests
)

add_test(
    NAME public_api_boundary_guard
    COMMAND ${CMAKE_COMMAND}
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/contract/check_public_api_boundary.cmake
)
set_tests_properties(public_api_boundary_guard PROPERTIES LABELS "contract;sar_contract;sar_ci")

add_test(
    NAME airborne_include_style_guard
    COMMAND ${CMAKE_COMMAND}
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/contract/check_airborne_include_style.cmake
)
add_test(
    NAME esr_include_style_guard
    COMMAND ${CMAKE_COMMAND}
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/contract/check_esr_include_style.cmake
)

add_test(
    NAME eos_include_style_guard
    COMMAND ${CMAKE_COMMAND}
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/contract/check_eos_include_style.cmake
)

add_test(
    NAME airborne_include_direction_guard
    COMMAND ${CMAKE_COMMAND}
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/contract/check_airborne_include_direction.cmake
)
add_test(
    NAME esr_include_direction_guard
    COMMAND ${CMAKE_COMMAND}
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/contract/check_esr_include_direction.cmake
)
add_test(
    NAME eos_include_direction_guard
    COMMAND ${CMAKE_COMMAND}
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/contract/check_eos_include_direction.cmake
)
add_test(
    NAME public_header_external_dependency_isolation_guard
    COMMAND ${CMAKE_COMMAND}
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/contract/check_public_header_external_dependency_isolation.cmake
)

# 跨域命名规约守护：四域 Session 签名裸名 + SNR 门限前缀异名报告。
add_test(
    NAME cross_domain_naming_guard
    COMMAND ${CMAKE_COMMAND}
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/contract/check_cross_domain_naming.cmake
)

# 公共头 C++11 兼容性守护：扫描 include/1q/ 是否出现 C++14/17 特征
# （VS2015 消费方兼容）；与 check_sar_cxx11_compat.cmake（编译式）互补。
add_test(
    NAME public_header_cxx11_guard
    COMMAND ${CMAKE_COMMAND}
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/contract/check_public_header_cxx11.cmake
)

# install 清单守护：确保各 src/*/CMakeLists.txt 的 install(FILES) 清单
# 与 include/1q/ 磁盘头文件一致(无幽灵头、无漏 install)。与
# check_public_api_boundary.cmake 互补:前者管 whitelist↔磁盘,本守护管 install↔磁盘。
add_test(
    NAME install_manifest_guard
    COMMAND ${CMAKE_COMMAND}
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/contract/check_install_manifest.cmake
)

# 守护规范性文档不残留已收口的旧入口术语(SessionFactory / extension:: SPI 等)。
# 本守护只扫描 docs/common 与五个模块目录下的规范性文档。
add_test(
    NAME doc_legacy_term_guard
    COMMAND ${CMAKE_COMMAND}
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/contract/check_doc_legacy_term_guard.cmake
)

add_test(
    NAME sar_doc_governance_guard
    COMMAND ${CMAKE_COMMAND}
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/contract/check_sar_doc_governance.cmake
)

add_test(
    NAME docs_structure_guard
    COMMAND ${CMAKE_COMMAND}
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/contract/check_docs_structure.cmake
)

set_tests_properties(
    public_api_boundary_guard
    airborne_include_style_guard
    esr_include_style_guard
    eos_include_style_guard
    airborne_include_direction_guard
    esr_include_direction_guard
    eos_include_direction_guard
    public_header_external_dependency_isolation_guard
    cross_domain_naming_guard
    public_header_cxx11_guard
    install_manifest_guard
    doc_legacy_term_guard
    sar_doc_governance_guard
    docs_structure_guard
    PROPERTIES LABELS "contract"
)

set_tests_properties(sar_doc_governance_guard PROPERTIES LABELS "contract;sar_contract;sar_ci")

# Scene programs — standalone executables that exercise full module pipelines
file(GLOB SCENE_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/scene/*.cpp")
if(SCENE_SOURCES)
    foreach(_scene_src ${SCENE_SOURCES})
        get_filename_component(_scene_name ${_scene_src} NAME_WE)
        add_executable(${_scene_name} ${_scene_src})
        target_link_libraries(${_scene_name} PRIVATE
            ${PROJECT_ALIAS}
        )
        target_include_directories(${_scene_name} PRIVATE
            ${CMAKE_SOURCE_DIR}/include
            ${CMAKE_SOURCE_DIR}/src
            ${CMAKE_CURRENT_SOURCE_DIR}
        )
        target_compile_definitions(${_scene_name} PRIVATE
            SCENE_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/scene/config"
        )
        # Scene programs include config_loader headers from the examples directory
        if(_scene_name MATCHES "airborne_radar")
            target_include_directories(${_scene_name} PRIVATE
                "${CMAKE_SOURCE_DIR}/examples/airborne_radar")
        elseif(_scene_name MATCHES "electro_optical")
            target_include_directories(${_scene_name} PRIVATE
                "${CMAKE_SOURCE_DIR}/examples/electro_optical")
        elseif(_scene_name MATCHES "electronic_warfare")
            target_include_directories(${_scene_name} PRIVATE
                "${CMAKE_SOURCE_DIR}/examples/electronic_warfare")
        endif()
    endforeach()
endif()
