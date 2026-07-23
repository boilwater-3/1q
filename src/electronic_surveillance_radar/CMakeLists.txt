# electronic_surveillance_radar 模块源文件与安装清单

# -- engine: 信号处理、拦截流水线（算法密集） --------------------------------
set(ESR_ENGINE_SOURCES
    pipeline/MutableEsrContext.cpp
    pipeline/HypothesisAssociator.cpp
    pipeline/InterceptDetectionExecutor.cpp
    pipeline/EsrRfV2FrontEnd.cpp
    pipeline/InterceptPostProcessingExecutor.cpp
    pipeline/InterceptPipeline.cpp
    pipeline/KdTreeClusterer.cpp
    pipeline/ObservationPreprocessor.cpp
)

# -- core: 会话、运行时、环境、输出（粘合层，依赖轻） --------------------------
set(ESR_CORE_SOURCES
    session/EsrSessionConfigBuilder.cpp
    session/EsrExternalInputAdapter.cpp
    session/EsrCycleInputAdapter.cpp
    session/EsrExternalOutputAdapter.cpp
    session/EsrCycleOutputAdapter.cpp
    environment/EsrEnvironmentService.cpp
    runtime/EsrController.cpp
    session/EsrInputValidation.cpp
    session/EsrEmitterLifecycleRecorder.cpp
    session/EsrOutputDebugViewBuilder.cpp
    session/EsrSessionCompositionRoot.cpp
    session/EsrResolutionRules.cpp
    session/EsrSessionConfigResolver.cpp
    session/EsrRuntimeConfigResolver.cpp
    session/EsrSession.cpp
    session/EsrTraceSession.cpp
    session/EsrReplaySession.cpp
    session/EsrReplayFlatbufferCodec.cpp
)

oneq_add_component(esr_engine SOURCES ${ESR_ENGINE_SOURCES})
oneq_add_component(esr_core SOURCES ${ESR_CORE_SOURCES})
# Keep the lifecycle recorder out of Unity Build because it and the debug-view
# builder deliberately use the same file-local association lookup helper.
set_source_files_properties(session/EsrEmitterLifecycleRecorder.cpp
    PROPERTIES SKIP_UNITY_BUILD_INCLUSION ON)
target_link_libraries(esr_engine PRIVATE ${ONEQ_LINK_DEPENDENCIES})
target_link_libraries(esr_core PRIVATE ${ONEQ_LINK_DEPENDENCIES})
oneq_add_flatbuffers_schemas(
    TARGET esr_core
    MODULE electronic_surveillance_radar
    SCHEMAS esr_replay.fbs esr_session_replay.fbs)
