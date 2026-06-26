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
    airborne_radar/session/RadarCycleInputBuilder.cpp
    airborne_radar/session/RadarExternalOutputAdapter.cpp
    airborne_radar/session/RadarCycleOutputBuilder.cpp
    airborne_radar/session/RadarTrackOutputDebugViewBuilder.cpp
    airborne_radar/session/RadarTrackLifecycleRecorder.cpp
    airborne_radar/session/RadarSessionConfigBuilder.cpp
    airborne_radar/session/RadarReplaySession.cpp
    airborne_radar/session/RadarSession.cpp
    airborne_radar/environment/FeatureRepository.cpp
    airborne_radar/environment/EnvironmentSceneBuilder.cpp
    airborne_radar/environment/EnvironmentService.cpp
    airborne_radar/environment/EnvironmentPhysics.cpp
    airborne_radar/environment/SceneManager.cpp
    airborne_radar/environment/PropagationModel.cpp
    airborne_radar/session/RadarReplayFlatbufferCodec.cpp
    airborne_radar/session/RadarTraceSession.cpp
)

if(ENABLE_INSTALL)
    set(PUBLIC_HEADERS_COMMON_CONTROL
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/extension/control/ControlDirective.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/extension/control/RadarCommand.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/extension/control/RadarControlProfile.h
    )
    set(PUBLIC_HEADERS_COMMON_MODEL
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/model/DecisionInputFrame.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/model/DecisionSourceInfo.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/model/TrackStateSnapshot.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/model/RadarOrientationConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/model/TargetCategory.h
    )
    set(PUBLIC_HEADERS_COMMON_UTILS
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/model/JammingSemantics.h
    )
    set(PUBLIC_HEADERS_CONFIG_ENTRY
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/RadarEnvironmentConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/RadarHardwareConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/RadarMissionConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/RadarPolicyConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/RadarRuntimeConfigPatch.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/RadarSessionConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/RadarRuntimeConfigBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/RadarSessionConfigBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/airborne_radar_config.hpp
    )
    set(PUBLIC_HEADERS_CORE_CONTEXT
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarCycleInput.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarCycleInputBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarCycleOutputBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarEnvironmentInput.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarEnvironmentInputPatch.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarEnvironmentInputState.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarInputValidation.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarExternalInputAdapter.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarExternalOutputAdapter.h
    )
    set(PUBLIC_HEADERS_CORE_CONTROLLER
    )
    set(PUBLIC_HEADERS_CORE_SESSION
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarCycleResult.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarOutputTypes.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarSceneTypes.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarSessionFactory.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarSession.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarReplaySession.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarTrackLifecycleRecorder.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarTrackOutputDebugView.h
    )
    set(PUBLIC_HEADERS_DECISION
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/extension/ITacticalDecisionEngine.h
    )
    set(PUBLIC_HEADERS_ENVIRONMENT
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/environment/EnvironmentConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/environment/EnvironmentRuntimeConfigPatch.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/environment/EnvironmentSceneBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/environment/EnvironmentTypes.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/environment/airborne_radar_environment.hpp
    )
    set(PUBLIC_HEADERS_SIGNAL_PIPELINE
    )
    set(PUBLIC_HEADERS_EXTENSION_ENTRY
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/extension/airborne_radar_extension.hpp
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
    install(FILES ${PUBLIC_HEADERS_COMMON_CONTROL}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/extension/control
    )
    install(FILES ${PUBLIC_HEADERS_COMMON_MODEL}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/model
    )
    install(FILES ${PUBLIC_HEADERS_COMMON_UTILS}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/model
    )
    # output/ 目录当前无公开头文件，预留安装点
    install(FILES ${PUBLIC_HEADERS_CONFIG_ENTRY}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/config
    )
    install(FILES ${PUBLIC_HEADERS_CORE_CONTEXT}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/session
    )
    install(FILES ${PUBLIC_HEADERS_CORE_CONTROLLER}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/extension
    )
    install(FILES ${PUBLIC_HEADERS_CORE_SESSION}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/session
    )
    install(FILES ${PUBLIC_HEADERS_DECISION}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/extension
    )
    install(FILES ${PUBLIC_HEADERS_ENVIRONMENT}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/environment
    )
    install(FILES ${PUBLIC_HEADERS_SIGNAL_PIPELINE}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/extension
    )
    install(FILES ${PUBLIC_HEADERS_AIRBORNE_TOOLS}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/session
    )
    install(FILES ${PUBLIC_HEADERS_EXTENSION_ENTRY}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/extension
    )
endif()
