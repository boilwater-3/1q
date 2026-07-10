# SAR-focused CTest entry points and C++11 compatibility probe.

set(_sar_unit_filter
    "SarFftBackendTest.*:" "SarSignalChainTest.*:" "SarWindowFunctionTest.*:"
    "SarGeometryTest.*:" "SarGeometryModelTest.*:" "SarAntennaPatternTest.*:"
    "SarImageOutputTest.*:" "SarEchoTest.*:" "SarEchoClutterTest.*:"
    "SarPgaSupportGradientTruthTest.*:" "SarPgaPhaseGradientEstimatorTest.*:"
    "SarPgaGradientTruthComparisonTest.*:" "SarAutofocusPhaseTruthTest.*:"
    "SarSlowTimeResamplingTest.*:" "SarRawHistorySlowTimeResamplingTest.*:"
    "SarVariablePrfQualityMatrixTest.*:" "SarExtendedVariablePrfQualityMatrixTest.*:"
    "SarMissingPulseGapDiagnosticsTest.*:" "SarMissingPulseRejectionMatrixTest.*:"
    "SarSlowTimeResamplingExecutorTest.*:" "PulseRingBufferTest.*:"
    "SarGbpTest.*:" "SarImageQualityTest.*:" "SarMotionCompensationTest.*:"
    "SarRdaTest.*:" "SarSessionPipelineTest.*:" "SarCycleInputAdapterBridgeTest.*:"
    "SarDegenerateDiagnosticsTest.*:" "SarRawHistoryExternalIqPredicateTest.*:"
    "SarReplayCodecRoundtripTest.*:" "SarReplaySessionTest.*:"
    "SarRuntimeConfigResolverTest.*")
string(REPLACE ";" "" _sar_unit_filter_str "${_sar_unit_filter}")
add_test(NAME "sar_unit::1q_sar_unit_tests"
         COMMAND ${PROJECT_NAME}_sar_unit_tests --gtest_filter=${_sar_unit_filter_str})
set_tests_properties("sar_unit::1q_sar_unit_tests" PROPERTIES LABELS "sar_unit;sar_ci")

set(_sar_replay_filter "SarReplayCodecRoundtripTest.*:" "SarReplaySessionTest.*")
string(REPLACE ";" "" _sar_replay_filter_str "${_sar_replay_filter}")
add_test(NAME "sar_replay::1q_replay_fast_tests"
         COMMAND ${PROJECT_NAME}_replay_fast_tests --gtest_filter=${_sar_replay_filter_str})
set_tests_properties("sar_replay::1q_replay_fast_tests" PROPERTIES LABELS "sar_replay;sar_ci")

set(_sar_integration_filter "SarSessionPipelineTest.*:" "SarReplaySessionTest.*")
string(REPLACE ";" "" _sar_integration_filter_str "${_sar_integration_filter}")
add_test(NAME "sar_integration::1q_sar_unit_tests"
         COMMAND ${PROJECT_NAME}_sar_unit_tests --gtest_filter=${_sar_integration_filter_str})
set_tests_properties("sar_integration::1q_sar_unit_tests"
                     PROPERTIES LABELS "sar_integration;sar_ci")

add_test(NAME "sar_contract::1q_contract_tests"
         COMMAND ${PROJECT_NAME}_contract_tests
                 --gtest_filter=PublicHeadersSmokeTest.SarPublicSurfaceSupportsMinimalUsage)
set_tests_properties("sar_contract::1q_contract_tests" PROPERTIES LABELS "sar_contract;sar_ci")

add_test(NAME sar_frozen_sources
         COMMAND ${CMAKE_COMMAND}
                 -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
                 -P ${CMAKE_CURRENT_SOURCE_DIR}/contract/check_sar_frozen_sources.cmake)
set_tests_properties(sar_frozen_sources PROPERTIES LABELS "sar_contract;sar_ci")

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
    add_test(NAME sar_cxx11_compat
        COMMAND ${CMAKE_COMMAND}
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -DCXX_COMPILER=${CMAKE_CXX_COMPILER}
            -DCXX_COMPILER_ID=${CMAKE_CXX_COMPILER_ID}
            -DEIGEN_INCLUDE_DIR=${_sar_eigen_include_dir}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/contract/check_sar_cxx11_compat.cmake)
endif()
set_tests_properties(sar_cxx11_compat PROPERTIES LABELS "sar_cxx11_compat")
