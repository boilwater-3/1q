# electro_optical_sensor 模块源文件与安装清单

# -- engine: 信号处理、物理模型（foundation + signal，依赖无） -------------------
set(EOS_ENGINE_SOURCES
    foundation/EosLookAngles.cpp
    foundation/EosNoiseModel.cpp
    foundation/EosOpticalCharacteristics.cpp
    foundation/EosPropagation.cpp
    foundation/EosRadiativeTransfer.cpp
    foundation/EosRadiometry.cpp
    foundation/EosSpatialSpectrum.cpp
    foundation/EosStrayLight.cpp
    pipeline/EosPipeline.cpp
)

# -- core: 会话、运行时、环境（粘合层，依赖轻） --------------------------------
set(EOS_CORE_SOURCES
    environment/EosEnvironmentModel.cpp
    session/EosSessionConfigValidation.cpp
    session/EosExternalOutputAdapter.cpp
    session/EosCycleOutputAdapter.cpp
    session/EosInputValidation.cpp
    session/EosDetectionLifecycleRecorder.cpp
    session/EosExclusionCauseRecorder.cpp
    session/EosOutputDebugViewBuilder.cpp
    runtime/EosController.cpp
    runtime/EosPipelineConfigMapper.cpp
    runtime/EosRuntimeConfigResolver.cpp
    session/EosSessionCompositionRoot.cpp
    session/EosSession.cpp
    session/EosTraceSession.cpp
    session/EosReplaySession.cpp
    session/EosDiagnosticUtils.cpp
    session/EosReplayFlatbufferCodec.cpp
)

oneq_add_component(eos_engine SOURCES ${EOS_ENGINE_SOURCES})
oneq_add_component(eos_core SOURCES ${EOS_CORE_SOURCES})
# Keep file-local detection lookup helpers in separate translation units under
# Unity Build; the recorder and debug-view builder use identical local names.
set_source_files_properties(session/EosDetectionLifecycleRecorder.cpp
    PROPERTIES SKIP_UNITY_BUILD_INCLUSION ON)
# eos_engine 经 common/geometry/BoresightChain（GeometryTransform.h 传递包含 Eigen）。
target_link_libraries(eos_engine PRIVATE Eigen3::Eigen flatbuffers::flatbuffers)
target_link_libraries(eos_core PRIVATE flatbuffers::flatbuffers)
oneq_add_flatbuffers_schemas(
    TARGET eos_core
    MODULE electro_optical_sensor
    SCHEMAS eos_replay.fbs eos_session_replay.fbs)
