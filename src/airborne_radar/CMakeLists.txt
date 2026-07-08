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
    airborne_radar/signal/pipeline/ImmMatrixDefaults.cpp
    airborne_radar/signal/pipeline/JammingEffects.cpp
    airborne_radar/signal/pipeline/SignalPipeline.cpp
    airborne_radar/signal/pipeline/TrackMeasurementProcessing.cpp
    airborne_radar/signal/pipeline/RuntimeAssemblySupport.cpp
    airborne_radar/signal/pipeline/ScanScheduleResolver.cpp
    airborne_radar/signal/pipeline/SignalComponentFactory.cpp
    airborne_radar/signal/tracking/BoostTrackPool.cpp
    airborne_radar/signal/tracking/SynchronizedTrackPool.cpp
    airborne_radar/signal/tracking/TrackFilter.cpp
    airborne_radar/signal/tracking/TrackLifecycleManager.cpp
    airborne_radar/signal/tracking/TrackLifecycleSnapshots.cpp
    airborne_radar/signal/tracking/TrackStateSnapshotEmitter.cpp
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
    airborne_radar/session/MutableArContext.cpp
    airborne_radar/session/ArInputValidation.cpp
    airborne_radar/session/ArSceneTargetUtils.cpp
    airborne_radar/runtime/ControlCommandMapper.cpp
    airborne_radar/runtime/ArController.cpp
    airborne_radar/session/ArSessionCompositionRoot.cpp
    airborne_radar/session/ArExternalInputAdapter.cpp
    airborne_radar/session/ArCycleInputAdapter.cpp
    airborne_radar/session/ArExternalOutputAdapter.cpp
    airborne_radar/session/ArCycleOutputAdapter.cpp
    airborne_radar/session/ArTrackOutputDebugViewBuilder.cpp
    airborne_radar/session/ArTrackLifecycleRecorder.cpp
    airborne_radar/session/ArSessionConfigBuilder.cpp
    airborne_radar/session/ArReplaySession.cpp
    airborne_radar/session/ArSession.cpp
    airborne_radar/environment/FeatureRepository.cpp
    airborne_radar/environment/EnvironmentService.cpp
    airborne_radar/environment/EnvironmentPhysics.cpp
    airborne_radar/environment/SceneManager.cpp
    airborne_radar/environment/PropagationModel.cpp
    airborne_radar/session/ArReplayFlatbufferCodec.cpp
    airborne_radar/session/ArTraceSession.cpp
)

if(ENABLE_INSTALL)
    set(PUBLIC_HEADERS_CONFIG_ENTRY
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/ArEnvironmentConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/ArHardwareConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/ArMissionConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/ArPolicyConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/ArRuntimeConfigPatch.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/ArSessionConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/ArRuntimeConfigBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/ArSessionConfigBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/ArSessionConfigValidation.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/ArOrientationConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/JammingSemantics.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/airborne_radar_config.hpp
    )
    set(PUBLIC_HEADERS_CORE_CONTEXT
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/ArCycleInput.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/ArCycleInputAdapter.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/ArCycleOutputAdapter.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/ArEnvironmentInput.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/ArInputValidation.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/ArExternalInputAdapter.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/ArExternalOutputAdapter.h
    )
    set(PUBLIC_HEADERS_CORE_SESSION
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/ArCycleResult.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/ArOutputTypes.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/ArSceneTypes.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/ArSession.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/ArReplaySession.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/ArTrackLifecycleRecorder.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/ArTrackOutputDebugView.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/ArCommand.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/ArControlProfile.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/ITacticalDecisionEngine.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/ControlDirective.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/DecisionInputFrame.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/DecisionSourceInfo.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/TrackStateSnapshot.h
    )
    set(PUBLIC_HEADERS_MODULE_ENTRY_AIRBORNE
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/airborne_radar.hpp
    )
    set(PUBLIC_HEADERS_AIRBORNE_TOOLS
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/ArTraceSession.h
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
