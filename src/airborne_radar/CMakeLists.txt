# airborne_radar 模块源文件与安装清单

# -- engine: 信号处理、检测、关联、跟踪（依赖 Eigen/Boost，算法密集） ---------------
set(AIRBORNE_ENGINE_SOURCES
    signal/association/DataAssociation.cpp
    signal/association/DistanceMetric.cpp
    signal/association/Hypothesiser.cpp
    signal/association/LapjvSolver.cpp
    signal/detection/RadarEquations.cpp
    signal/detection/SignalDetector.cpp
    signal/pipeline/ControlProfileEffects.cpp
    signal/pipeline/CycleContextSupport.cpp
    signal/pipeline/CycleExecutor.cpp
    signal/pipeline/DecisionFrameBuilders.cpp
    signal/pipeline/DetectionExecution.cpp
    signal/pipeline/ImmMatrixDefaults.cpp
    signal/pipeline/JammingEffects.cpp
    signal/pipeline/SignalPipeline.cpp
    signal/pipeline/TrackMeasurementProcessing.cpp
    signal/pipeline/RuntimeAssemblySupport.cpp
    signal/pipeline/ScanScheduleResolver.cpp
    signal/pipeline/SignalComponentFactory.cpp
    signal/tracking/BoostTrackPool.cpp
    signal/tracking/SynchronizedTrackPool.cpp
    signal/tracking/TrackFilter.cpp
    signal/tracking/TrackLifecycleManager.cpp
    signal/tracking/TrackLifecycleSnapshots.cpp
    signal/tracking/TrackStateSnapshotEmitter.cpp
)

# -- core: 会话、运行时、环境、配置、决策、输出（粘合层，依赖轻） -------------------
set(AIRBORNE_CORE_SOURCES
    output/TrackOutputQueries.cpp
    decision/ThreatAssessmentEvaluator.cpp
    decision/LpiEvaluator.cpp
    decision/EccmEvaluator.cpp
    decision/ControlReducer.cpp
    decision/TacticalCoordinator.cpp
    config/mapping/SessionToExecutionMapper.cpp
    config/mapping/RuntimePatchMapper.cpp
    session/MutableArContext.cpp
    session/ArInputValidation.cpp
    session/ArSceneTargetUtils.cpp
    runtime/ControlCommandMapper.cpp
    runtime/ArController.cpp
    session/ArSessionCompositionRoot.cpp
    session/ArExternalInputAdapter.cpp
    session/ArCycleInputAdapter.cpp
    session/ArExternalOutputAdapter.cpp
    session/ArCycleOutputAdapter.cpp
    session/ArTrackOutputDebugViewBuilder.cpp
    session/ArTrackLifecycleRecorder.cpp
    session/ArSessionConfigBuilder.cpp
    session/ArReplaySession.cpp
    session/ArSession.cpp
    environment/FeatureRepository.cpp
    environment/EnvironmentService.cpp
    environment/EnvironmentPhysics.cpp
    environment/SceneManager.cpp
    environment/PropagationModel.cpp
    session/ArReplayFlatbufferCodec.cpp
    session/ArTraceSession.cpp
)

oneq_add_component(airborne_engine SOURCES ${AIRBORNE_ENGINE_SOURCES})
oneq_add_component(airborne_core SOURCES ${AIRBORNE_CORE_SOURCES})
# These two sources intentionally retain file-local lookup helpers with the same
# name. Keep the lifecycle recorder standalone when Unity Build is enabled.
set_source_files_properties(session/ArTrackLifecycleRecorder.cpp
    PROPERTIES SKIP_UNITY_BUILD_INCLUSION ON)
# Both external adapters use an identically named file-local finite-vector
# predicate. Keep one translation unit standalone for every Unity batch size.
set_source_files_properties(session/ArExternalOutputAdapter.cpp
    PROPERTIES SKIP_UNITY_BUILD_INCLUSION ON)
target_link_libraries(airborne_engine PRIVATE ${ONEQ_LINK_DEPENDENCIES})
target_link_libraries(airborne_core PRIVATE ${ONEQ_LINK_DEPENDENCIES})
oneq_add_flatbuffers_schemas(
    TARGET airborne_core
    MODULE airborne_radar
    SCHEMAS airborne_radar_replay.fbs airborne_radar_session_replay.fbs)
