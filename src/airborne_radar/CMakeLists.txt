# airborne_radar 模块源文件与安装清单

set(AIRBORNE_COMMON_SOURCES
    airborne_radar/model/DecisionTrackSnapshot.cpp
    airborne_radar/output/TrackOutputQueries.cpp
    airborne_radar/model/TargetFeatureUtils.cpp
)

set(AIRBORNE_SIGNAL_SOURCES
    airborne_radar/signal/association/DataAssociation.cpp
    airborne_radar/signal/association/DistanceMetric.cpp
    airborne_radar/signal/association/Hypothesiser.cpp
    airborne_radar/signal/association/LapjvSolver.cpp
    airborne_radar/signal/assembly/DataOutputManager.cpp
    airborne_radar/signal/detection/RadarEquations.cpp
    airborne_radar/signal/detection/SignalDetector.cpp
    airborne_radar/signal/pipeline/core/AssociationExecutionSupport.cpp
    airborne_radar/signal/pipeline/core/ContextBindingSupport.cpp
    airborne_radar/signal/pipeline/effects/ControlProfileEffects.cpp
    airborne_radar/signal/pipeline/core/CycleContextSupport.cpp
    airborne_radar/signal/pipeline/core/CycleExecutor.cpp
    airborne_radar/signal/pipeline/assembly/DecisionFrameBuilders.cpp
    airborne_radar/signal/pipeline/core/DetectionExecution.cpp
    airborne_radar/signal/pipeline/effects/JammingEffects.cpp
    airborne_radar/signal/pipeline/assembly/OutputAssemblySupport.cpp
    airborne_radar/signal/pipeline/core/SignalPipeline.cpp
    airborne_radar/signal/pipeline/assembly/TrackMeasurementProcessing.cpp
    airborne_radar/signal/pipeline/assembly/RuntimeAssemblySupport.cpp
    airborne_radar/signal/pipeline/core/ScanScheduleResolver.cpp
    airborne_radar/signal/pipeline/assembly/SignalComponentFactory.cpp
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
    airborne_radar/signal/tracking/TrackSnapshotEmitter.cpp
    airborne_radar/signal/tracking/UdkfPredictor.cpp
    airborne_radar/signal/tracking/UdkfUpdater.cpp
)

set(AIRBORNE_DECISION_SOURCES
    airborne_radar/decision/evaluators/EmissionControlEvaluator.cpp
    airborne_radar/decision/evaluators/SurvivabilityEvaluator.cpp
    airborne_radar/decision/evaluators/SurvivabilityEvaluatorHelpers.cpp
    airborne_radar/decision/evaluators/ThreatAssessmentEvaluator.cpp
    airborne_radar/decision/pipeline/ControlReducer.cpp
    airborne_radar/decision/pipeline/TacticalCoordinator.cpp
)

set(AIRBORNE_CORE_SOURCES
    airborne_radar/session/MutableRadarContext.cpp
    airborne_radar/session/RadarInputValidation.cpp
    airborne_radar/runtime/ControlCommandMapper.cpp
    airborne_radar/runtime/CycleTelemetryLogger.cpp
    airborne_radar/runtime/RadarCycleOrchestrator.cpp
    airborne_radar/runtime/RadarController.cpp
    airborne_radar/session/RadarSessionCompositionRoot.cpp
    airborne_radar/session/RadarExternalInputAdapter.cpp
    airborne_radar/session/RadarSessionConfigPresets.cpp
    airborne_radar/session/RadarSessionConfigBuilder.cpp
    airborne_radar/session/RuntimeConfigResolver.cpp
    airborne_radar/session/RadarSession.cpp
    airborne_radar/session/RadarTraceSession.cpp
    airborne_radar/environment/FeatureRepository.cpp
    airborne_radar/environment/EnvironmentSceneBuilder.cpp
    airborne_radar/environment/EnvironmentService.cpp
    airborne_radar/environment/SceneManager.cpp
    airborne_radar/environment/PropagationModel.cpp
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
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/model/DecisionTrackSnapshot.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/model/RadarOrientationConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/model/TargetCategory.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/model/TargetFeature.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/model/TargetFeatureBuilder.h
    )
    set(PUBLIC_HEADERS_COMMON_UTILS
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/model/JammingSemantics.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/model/TargetFeatureUtils.h
    )
    set(PUBLIC_HEADERS_COMMON_OUTPUT
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/output/TrackOutputFrame.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/output/TrackOutputQueries.h
    )
    set(PUBLIC_HEADERS_CONFIG_ENTRY
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/RadarSessionConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/RadarRuntimeConfigBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/RadarExpertSessionConfigBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/RadarSessionConfigBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/ConfigModel.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/airborne_radar_config.hpp
    )
    set(PUBLIC_HEADERS_SIGNAL_CONFIG
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/PipelineConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/semantic/AntennaPatternConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/semantic/BeamControlConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/semantic/DetectionConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/semantic/LifecycleConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/semantic/TrackingConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/semantic/profiles/DetectionProfiles.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/semantic/profiles/LifecycleProfiles.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/semantic/profiles/TrackingProfiles.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/presets/RadarSessionConfigPresets.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/presets/PipelineConfigPresets.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/expert/ExpertPipelineConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/expert/beam/BeamControlConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/expert/beam/BeamPointingConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/expert/beam/BeamSchedulerConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/expert/detection/AntennaConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/expert/detection/AntennaPatternConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/expert/detection/DetectionConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/expert/detection/DetectionPolicyConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/expert/detection/RcsPhysicsConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/expert/detection/ReceiverConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/expert/detection/TransmitterConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/expert/lifecycle/ImmConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/expert/lifecycle/LifecycleConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/expert/tracking/AssociationConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/expert/tracking/KalmanConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/expert/tracking/TrackingConfig.h
    )
    set(PUBLIC_HEADERS_CORE_CONTEXT
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarCycleInput.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarInputValidation.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarExternalInputAdapter.h
    )
    set(PUBLIC_HEADERS_CORE_CONTROLLER
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/extension/IRadarContext.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/extension/IRadarOutputReader.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/extension/RadarController.h
    )
    set(PUBLIC_HEADERS_CORE_SESSION
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarCycleResult.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarSessionFactory.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarSession.h
    )
    set(PUBLIC_HEADERS_DECISION
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/extension/ControlReducerTypes.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/extension/ITacticalDecisionEngine.h
    )
    set(PUBLIC_HEADERS_ENVIRONMENT
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/environment/EnvironmentConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/environment/EnvironmentDefaultConfigBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/environment/EnvironmentRuntimeConfigPatch.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/environment/EnvironmentRuntimeConfigPatchBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/environment/EnvironmentSceneBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/environment/EnvironmentTypes.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/environment/IEnvironmentService.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/environment/airborne_radar_environment.hpp
    )
    set(PUBLIC_HEADERS_SIGNAL_PIPELINE
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/extension/ISignalPipeline.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/extension/SignalPipelineResultTypes.h
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
    install(FILES ${PUBLIC_HEADERS_COMMON_OUTPUT}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/output
    )
    install(FILES ${PUBLIC_HEADERS_CONFIG_ENTRY}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/config
    )
    install(FILES ${PUBLIC_HEADERS_SIGNAL_CONFIG}
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
