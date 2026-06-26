# electro_optical_sensor 模块源文件与安装清单

# -- engine: 信号处理、物理模型（foundation + signal，依赖无） -------------------
set(EOS_ENGINE_SOURCES
    electro_optical_sensor/foundation/EosNoiseModel.cpp
    electro_optical_sensor/foundation/EosOpticalCharacteristics.cpp
    electro_optical_sensor/foundation/EosPropagation.cpp
    electro_optical_sensor/foundation/EosRadiativeTransfer.cpp
    electro_optical_sensor/foundation/EosRadiometry.cpp
    electro_optical_sensor/foundation/EosSpatialSpectrum.cpp
    electro_optical_sensor/foundation/EosStrayLight.cpp
    electro_optical_sensor/pipeline/EosPipeline.cpp
)

# -- core: 会话、运行时、环境（粘合层，依赖轻） --------------------------------
set(EOS_CORE_SOURCES
    electro_optical_sensor/environment/EosEnvironmentModel.cpp
    electro_optical_sensor/session/EosSessionConfigBuilder.cpp
    electro_optical_sensor/session/EosExternalInputAdapter.cpp
    electro_optical_sensor/session/EosCycleInputAdapter.cpp
    electro_optical_sensor/session/EosExternalOutputAdapter.cpp
    electro_optical_sensor/session/EosCycleOutputAdapter.cpp
    electro_optical_sensor/session/EosInputValidation.cpp
    electro_optical_sensor/session/EosDetectionLifecycleRecorder.cpp
    electro_optical_sensor/session/EosOutputDebugViewBuilder.cpp
    electro_optical_sensor/runtime/EosController.cpp
    electro_optical_sensor/runtime/EosPipelineConfigMapper.cpp
    electro_optical_sensor/runtime/EosRuntimeConfigResolver.cpp
    electro_optical_sensor/session/EosSessionCompositionRoot.cpp
    electro_optical_sensor/session/EosSession.cpp
    electro_optical_sensor/session/EosTraceSession.cpp
    electro_optical_sensor/session/EosReplaySession.cpp
    electro_optical_sensor/session/EosReplayFlatbufferCodec.cpp
)

if(ENABLE_INSTALL)
    set(PUBLIC_HEADERS_MODULE_ENTRY_EOS
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/electro_optical_sensor.hpp
    )
    set(PUBLIC_HEADERS_EOS_MODEL)
    set(PUBLIC_HEADERS_EOS_SESSION
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/session/EosCycleInput.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/session/EosCycleInputAdapter.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/session/EosCycleOutputAdapter.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/session/EosCycleResult.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/session/EosDetectionLifecycleRecorder.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/session/EosEnvironmentInput.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/session/EosExternalInputAdapter.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/session/EosExternalOutputAdapter.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/session/EosInputValidation.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/session/EosOutputDebugView.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/session/EosOutputTypes.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/session/EosSceneTypes.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/session/EosSession.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/session/EosTraceSession.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/session/EosReplaySession.h
    )
    set(PUBLIC_HEADERS_EOS_CONFIG
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/config/EosEnvironmentConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/config/EosHardwareConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/config/EosMissionConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/config/EosPolicyConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/config/EosSessionConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/config/EosRuntimeConfigPatch.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/config/EosSessionConfigBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/config/EosRuntimeConfigBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/config/electro_optical_sensor_config.hpp
    )

    install(FILES ${PUBLIC_HEADERS_MODULE_ENTRY_EOS}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electro_optical_sensor
    )
    install(FILES ${PUBLIC_HEADERS_EOS_MODEL}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electro_optical_sensor/model
    )
    install(FILES ${PUBLIC_HEADERS_EOS_SESSION}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electro_optical_sensor/session
    )
    install(FILES ${PUBLIC_HEADERS_EOS_CONFIG}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electro_optical_sensor/config
    )
endif()
