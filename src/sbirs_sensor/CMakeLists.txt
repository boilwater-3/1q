# sbirs_sensor 模块源文件与安装清单

set(SBIRS_ENGINE_SOURCES
    foundation/SbirsGeometry.cpp
    foundation/SbirsRadiometry.cpp
    foundation/SbirsErrorModel.cpp
    foundation/SbirsNoiseModel.cpp
    environment/SbirsEnvironmentModel.cpp
    pipeline/SbirsBoresightChain.cpp
    pipeline/SbirsCuePredictor.cpp
    pipeline/SbirsEciScene.cpp
    pipeline/SbirsPipeline.cpp
    pipeline/SbirsNfovAcquisition.cpp
    pipeline/SbirsNfovScheduler.cpp
    pipeline/SbirsPointingActuator.cpp
    pipeline/SbirsPointingDisturbance.cpp
    pipeline/SbirsPointingCoordinator.cpp
    pipeline/SbirsTrackingCoordinator.cpp
)

set(SBIRS_CORE_SOURCES
    session/SbirsSessionConfigBuilder.cpp
    session/SbirsRuntimeConfigBuilder.cpp
    session/SbirsSessionConfigValidation.cpp
    session/SbirsInputValidation.cpp
    session/SbirsCycleInputAdapter.cpp
    session/SbirsCycleOutputAdapter.cpp
    session/SbirsOutputDebugViewBuilder.cpp
    session/SbirsDetectionLifecycleRecorder.cpp
    session/SbirsExclusionCauseRecorder.cpp
    session/SbirsDiagnosticUtils.cpp
    session/SbirsReplayFlatbufferCodec.cpp
    session/SbirsTraceSession.cpp
    session/SbirsReplaySession.cpp
    session/SbirsExternalInputAdapter.cpp
    session/SbirsExternalOutputAdapter.cpp
    runtime/SbirsPipelineConfigMapper.cpp
    runtime/SbirsRuntimeConfigResolver.cpp
    runtime/SbirsController.cpp
    session/SbirsSessionCompositionRoot.cpp
    session/SbirsSession.cpp
)

oneq_add_component(sbirs_engine SOURCES ${SBIRS_ENGINE_SOURCES})
oneq_add_component(sbirs_core SOURCES ${SBIRS_CORE_SOURCES})
# Keep file-local detection lookup helpers in separate translation units under
# Unity Build; the recorder and debug-view builder use identical local names.
set_source_files_properties(session/SbirsDetectionLifecycleRecorder.cpp
    PROPERTIES SKIP_UNITY_BUILD_INCLUSION ON)
target_link_libraries(sbirs_engine PRIVATE Eigen3::Eigen flatbuffers::flatbuffers)
target_link_libraries(sbirs_core PRIVATE Eigen3::Eigen flatbuffers::flatbuffers)
oneq_add_flatbuffers_schemas(
    TARGET sbirs_core
    MODULE sbirs_sensor
    SCHEMAS sbirs_replay.fbs sbirs_session_replay.fbs)
