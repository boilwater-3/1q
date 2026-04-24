# electronic_surveillance_radar 模块源文件与安装清单

set(ESR_MODEL_SOURCES
    electronic_surveillance_radar/session/EsrExternalInputAdapter.cpp
)

set(ESR_ENVIRONMENT_SOURCES
    electronic_surveillance_radar/environment/EsrEnvironmentService.cpp
)

set(ESR_PIPELINE_SOURCES
    electronic_surveillance_radar/pipeline/MutableEsrContext.cpp
    electronic_surveillance_radar/pipeline/HypothesisAssociator.cpp
    electronic_surveillance_radar/pipeline/InterceptDetectionExecutor.cpp
    electronic_surveillance_radar/pipeline/InterceptPostProcessingExecutor.cpp
    electronic_surveillance_radar/pipeline/InterceptPipeline.cpp
    electronic_surveillance_radar/pipeline/KdTreeClusterer.cpp
    electronic_surveillance_radar/pipeline/ObservationPreprocessor.cpp
)

set(ESR_RUNTIME_SOURCES
    electronic_surveillance_radar/runtime/EsrController.cpp
    electronic_surveillance_radar/runtime/EsrCycleTelemetryLogger.cpp
    electronic_surveillance_radar/runtime/components/EsrEnvironmentUpdater.cpp
    electronic_surveillance_radar/runtime/components/EsrOutputFormatter.cpp
    electronic_surveillance_radar/runtime/components/EsrRuntimeHooks.cpp
    electronic_surveillance_radar/runtime/components/EsrSignalProcessor.cpp
    electronic_surveillance_radar/output/EsrOutputManager.cpp
)

set(ESR_SESSION_SOURCES
    electronic_surveillance_radar/session/EsrInputValidation.cpp
    electronic_surveillance_radar/session/EsrSessionCompositionRoot.cpp
    electronic_surveillance_radar/session/EsrSessionConfigResolver.cpp
    electronic_surveillance_radar/session/EsrRuntimeConfigResolver.cpp
    electronic_surveillance_radar/session/EsrSession.cpp
    electronic_surveillance_radar/session/EsrTraceSession.cpp
    electronic_surveillance_radar/session/EsrReplayFlatbufferCodec.cpp
)

if(ENABLE_INSTALL)
    set(PUBLIC_HEADERS_MODULE_ENTRY_ESR
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/electronic_surveillance_radar.hpp
    )
    set(PUBLIC_HEADERS_ESR_MODEL
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/model/EmitterHypothesis.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/model/EmitterObservation.h
    )
    set(PUBLIC_HEADERS_ESR_OUTPUT
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/output/EsrOutputFrame.h
    )
    set(PUBLIC_HEADERS_ESR_EXTENSION
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/extension/EsrController.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/extension/IEsrContext.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/extension/IInterceptPipeline.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/extension/InterceptPipelineTypes.h
    )
    set(PUBLIC_HEADERS_ESR_SESSION
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/session/EsrCycleInput.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/session/EsrCycleResult.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/session/EsrExternalInputAdapter.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/session/EsrInputValidation.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/session/EsrSession.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/session/EsrSessionFactory.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/session/EsrTraceSession.h
    )
    set(PUBLIC_HEADERS_ESR_ENVIRONMENT
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/environment/EsrEnvironmentConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/environment/EsrEnvironmentRuntimeConfigPatch.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/environment/EsrEnvironmentSceneBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/environment/EsrEnvironmentTypes.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/environment/IEsrEnvironmentService.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/environment/electronic_surveillance_radar_environment.hpp
    )
    set(PUBLIC_HEADERS_ESR_CONFIG
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/config/EsrDetectionPolicyConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/config/EsrEnvironmentPolicyConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/config/EsrHardwareConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/config/EsrPolicyConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/config/EsrRuntimeConfigPatch.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/config/EsrRuntimeConfigBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/config/EsrScanPolicyConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/config/EsrSessionConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/config/EsrSessionConfigBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/config/EsrMissionControlConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/config/EsrWorkMode.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/config/electronic_surveillance_radar_config.hpp
    )

    install(FILES ${PUBLIC_HEADERS_MODULE_ENTRY_ESR}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electronic_surveillance_radar
    )
    install(FILES ${PUBLIC_HEADERS_ESR_MODEL}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electronic_surveillance_radar/model
    )
    install(FILES ${PUBLIC_HEADERS_ESR_OUTPUT}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electronic_surveillance_radar/output
    )
    install(FILES ${PUBLIC_HEADERS_ESR_EXTENSION}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electronic_surveillance_radar/extension
    )
    install(FILES ${PUBLIC_HEADERS_ESR_SESSION}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electronic_surveillance_radar/session
    )
    install(FILES ${PUBLIC_HEADERS_ESR_ENVIRONMENT}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electronic_surveillance_radar/environment
    )
    install(FILES ${PUBLIC_HEADERS_ESR_CONFIG}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electronic_surveillance_radar/config
    )
endif()
