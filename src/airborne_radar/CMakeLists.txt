# airborne_radar 模块源文件与安装清单

# -- engine: 信号处理、检测、关联、跟踪（依赖 Eigen/Boost，算法密集） ---------------
set(AIRBORNE_ENGINE_SOURCES
    airborne_radar/signal/association/DataAssociation.cpp
    airborne_radar/signal/association/DistanceMetric.cpp
    airborne_radar/signal/association/Hypothesiser.cpp
    airborne_radar/signal/association/LapjvSolver.cpp
    airborne_radar/signal/detection/RadarEquations.cpp
    airborne_radar/signal/detection/SignalDetector.cpp
    airborne_radar/signal/pipeline/ControlProfileEffects.cpp
    airborne_radar/signal/pipeline/CycleContextSupport.cpp
    airborne_radar/signal/pipeline/CycleExecutor.cpp
    airborne_radar/signal/pipeline/DecisionFrameBuilders.cpp
    airborne_radar/signal/pipeline/DetectionExecution.cpp
    airborne_radar/signal/pipeline/JammingEffects.cpp
    airborne_radar/signal/pipeline/SignalPipeline.cpp
    airborne_radar/signal/pipeline/TrackMeasurementProcessing.cpp
    airborne_radar/signal/pipeline/RuntimeAssemblySupport.cpp
    airborne_radar/signal/pipeline/ScanScheduleResolver.cpp
    airborne_radar/signal/pipeline/SignalComponentFactory.cpp
    airborne_radar/signal/tracking/BoostTrackPool.cpp
    airborne_radar/signal/tracking/EkfFilter.cpp
    airborne_radar/signal/tracking/ImmFilter.cpp
    airborne_radar/signal/tracking/KalmanPredictor.cpp
    airborne_radar/signal/tracking/KalmanUpdater.cpp
    airborne_radar/signal/tracking/SrifPredictor.cpp
    airborne_radar/signal/tracking/SrifUpdater.cpp
    airborne_radar/signal/tracking/SynchronizedTrackPool.cpp
    airborne_radar/signal/tracking/TrackFilter.cpp
    airborne_radar/signal/tracking/TrackLifecycleManager.cpp
    airborne_radar/signal/tracking/TrackLifecycleSnapshots.cpp
    airborne_radar/signal/tracking/TrackStateSnapshotEmitter.cpp
    airborne_radar/signal/tracking/UdkfPredictor.cpp
    airborne_radar/signal/tracking/UdkfUpdater.cpp
)

# -- core: 会话、运行时、环境、配置、决策、输出（粘合层，依赖轻） -------------------
set(AIRBORNE_CORE_SOURCES
    airborne_radar/output/TrackOutputQueries.cpp
    airborne_radar/decision/ThreatAssessmentEvaluator.cpp
    airborne_radar/decision/LpiEvaluator.cpp
    airborne_radar/decision/EccmEvaluator.cpp
    airborne_radar/decision/ControlReducer.cpp
    airborne_radar/decision/TacticalCoordinator.cpp
    airborne_radar/config/mapping/SessionToExecutionMapper.cpp
    airborne_radar/config/mapping/RuntimePatchMapper.cpp
    airborne_radar/session/MutableRadarContext.cpp
    airborne_radar/session/RadarInputValidation.cpp
    airborne_radar/session/RadarSceneTargetUtils.cpp
    airborne_radar/runtime/ControlCommandMapper.cpp
    airborne_radar/runtime/RadarController.cpp
    airborne_radar/session/RadarSessionCompositionRoot.cpp
    airborne_radar/session/RadarExternalInputAdapter.cpp
    airborne_radar/session/RadarCycleInputAdapter.cpp
    airborne_radar/session/RadarExternalOutputAdapter.cpp
    airborne_radar/session/RadarCycleOutputAdapter.cpp
    airborne_radar/session/RadarTrackOutputDebugViewBuilder.cpp
    airborne_radar/session/RadarTrackLifecycleRecorder.cpp
    airborne_radar/session/RadarSessionConfigBuilder.cpp
    airborne_radar/session/RadarReplaySession.cpp
    airborne_radar/session/RadarSession.cpp
    airborne_radar/environment/FeatureRepository.cpp
    airborne_radar/environment/EnvironmentService.cpp
    airborne_radar/environment/EnvironmentPhysics.cpp
    airborne_radar/environment/SceneManager.cpp
    airborne_radar/environment/PropagationModel.cpp
    airborne_radar/session/RadarReplayFlatbufferCodec.cpp
    airborne_radar/session/RadarTraceSession.cpp
)

if(ENABLE_INSTALL)
    set(PUBLIC_HEADERS_CONFIG_ENTRY
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/RadarEnvironmentConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/RadarHardwareConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/RadarMissionConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/RadarPolicyConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/RadarRuntimeConfigPatch.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/RadarSessionConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/RadarRuntimeConfigBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/RadarSessionConfigBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/RadarOrientationConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/JammingSemantics.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/airborne_radar_config.hpp
    )
    set(PUBLIC_HEADERS_CORE_CONTEXT
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarCycleInput.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarCycleInputAdapter.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarCycleOutputAdapter.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarEnvironmentInput.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarInputValidation.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarExternalInputAdapter.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarExternalOutputAdapter.h
    )
    set(PUBLIC_HEADERS_CORE_SESSION
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarCycleResult.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarOutputTypes.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarSceneTypes.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarSession.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarReplaySession.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarTrackLifecycleRecorder.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarTrackOutputDebugView.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/ITacticalDecisionEngine.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/ControlDirective.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarCommand.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarControlProfile.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/DecisionInputFrame.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/DecisionSourceInfo.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/TrackStateSnapshot.h
    )
    set(PUBLIC_HEADERS_MODULE_ENTRY_AIRBORNE
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/airborne_radar.hpp
    )
    set(PUBLIC_HEADERS_AIRBORNE_TOOLS
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarTraceSession.h
    )

    install(FILES ${PUBLIC_HEADERS_MODULE_ENTRY_AIRBORNE}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar
    )
    install(FILES ${PUBLIC_HEADERS_CONFIG_ENTRY}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/config
    )
    install(FILES ${PUBLIC_HEADERS_CORE_CONTEXT}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/session
    )
    install(FILES ${PUBLIC_HEADERS_CORE_SESSION}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/session
    )
    install(FILES ${PUBLIC_HEADERS_AIRBORNE_TOOLS}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/session
    )
endif()
