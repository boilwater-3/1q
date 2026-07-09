# sbirs_sensor 模块源文件与安装清单

set(SBIRS_ENGINE_SOURCES
    sbirs_sensor/foundation/SbirsGeometry.cpp
    sbirs_sensor/foundation/SbirsRadiometry.cpp
    sbirs_sensor/foundation/SbirsErrorModel.cpp
    sbirs_sensor/foundation/SbirsNoiseModel.cpp
    sbirs_sensor/environment/SbirsEnvironmentModel.cpp
    sbirs_sensor/pipeline/SbirsPipeline.cpp
    sbirs_sensor/pipeline/SbirsNfovScheduler.cpp
)

set(SBIRS_CORE_SOURCES
    sbirs_sensor/session/SbirsSessionConfigBuilder.cpp
    sbirs_sensor/session/SbirsRuntimeConfigBuilder.cpp
    sbirs_sensor/session/SbirsSessionConfigValidation.cpp
    sbirs_sensor/session/SbirsInputValidation.cpp
    sbirs_sensor/session/SbirsCycleInputAdapter.cpp
    sbirs_sensor/session/SbirsCycleOutputAdapter.cpp
    sbirs_sensor/session/SbirsOutputDebugViewBuilder.cpp
    sbirs_sensor/session/SbirsDetectionLifecycleRecorder.cpp
    sbirs_sensor/session/SbirsReplayFlatbufferCodec.cpp
    sbirs_sensor/session/SbirsTraceSession.cpp
    sbirs_sensor/session/SbirsReplaySession.cpp
    sbirs_sensor/session/SbirsExternalInputAdapter.cpp
    sbirs_sensor/session/SbirsExternalOutputAdapter.cpp
    sbirs_sensor/runtime/SbirsPipelineConfigMapper.cpp
    sbirs_sensor/runtime/SbirsRuntimeConfigResolver.cpp
    sbirs_sensor/runtime/SbirsController.cpp
    sbirs_sensor/session/SbirsSessionCompositionRoot.cpp
    sbirs_sensor/session/SbirsSession.cpp
)

if(ENABLE_INSTALL)
    set(PUBLIC_HEADERS_MODULE_ENTRY_SBIRS
        ${CMAKE_SOURCE_DIR}/include/1q/sbirs_sensor/sbirs_sensor.hpp
    )
    set(PUBLIC_HEADERS_SBIRS_CONFIG
        ${CMAKE_SOURCE_DIR}/include/1q/sbirs_sensor/config/SbirsEnvironmentConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/sbirs_sensor/config/SbirsHardwareConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/sbirs_sensor/config/SbirsMissionConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/sbirs_sensor/config/SbirsPolicyConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/sbirs_sensor/config/SbirsRuntimeConfigBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/sbirs_sensor/config/SbirsRuntimeConfigPatch.h
        ${CMAKE_SOURCE_DIR}/include/1q/sbirs_sensor/config/SbirsSessionConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/sbirs_sensor/config/SbirsSessionConfigBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/sbirs_sensor/config/SbirsSessionConfigValidation.h
        ${CMAKE_SOURCE_DIR}/include/1q/sbirs_sensor/config/sbirs_sensor_config.hpp
    )
    set(PUBLIC_HEADERS_SBIRS_SESSION
        ${CMAKE_SOURCE_DIR}/include/1q/sbirs_sensor/session/SbirsCycleInput.h
        ${CMAKE_SOURCE_DIR}/include/1q/sbirs_sensor/session/SbirsCycleInputAdapter.h
        ${CMAKE_SOURCE_DIR}/include/1q/sbirs_sensor/session/SbirsCycleOutputAdapter.h
        ${CMAKE_SOURCE_DIR}/include/1q/sbirs_sensor/session/SbirsCycleResult.h
        ${CMAKE_SOURCE_DIR}/include/1q/sbirs_sensor/session/SbirsDetectionLifecycleRecorder.h
        ${CMAKE_SOURCE_DIR}/include/1q/sbirs_sensor/session/SbirsEnvironmentInput.h
        ${CMAKE_SOURCE_DIR}/include/1q/sbirs_sensor/session/SbirsExternalInputAdapter.h
        ${CMAKE_SOURCE_DIR}/include/1q/sbirs_sensor/session/SbirsExternalOutputAdapter.h
        ${CMAKE_SOURCE_DIR}/include/1q/sbirs_sensor/session/SbirsInputValidation.h
        ${CMAKE_SOURCE_DIR}/include/1q/sbirs_sensor/session/SbirsOutputDebugView.h
        ${CMAKE_SOURCE_DIR}/include/1q/sbirs_sensor/session/SbirsOutputTypes.h
        ${CMAKE_SOURCE_DIR}/include/1q/sbirs_sensor/session/SbirsReplaySession.h
        ${CMAKE_SOURCE_DIR}/include/1q/sbirs_sensor/session/SbirsSceneTypes.h
        ${CMAKE_SOURCE_DIR}/include/1q/sbirs_sensor/session/SbirsSession.h
        ${CMAKE_SOURCE_DIR}/include/1q/sbirs_sensor/session/SbirsTraceSession.h
    )

    install(FILES ${PUBLIC_HEADERS_MODULE_ENTRY_SBIRS}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/sbirs_sensor
    )
    install(FILES ${PUBLIC_HEADERS_SBIRS_CONFIG}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/sbirs_sensor/config
    )
    install(FILES ${PUBLIC_HEADERS_SBIRS_SESSION}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/sbirs_sensor/session
    )
endif()
