# sar 模块源文件与安装清单

# -- engine: 信号处理、几何、回波、成像与输出内核 -----------------------------
set(SAR_ENGINE_SOURCES
    sar/calibration/SarRadiometricCalibration.cpp
    sar/echo/SarEcho.cpp
    sar/geometry/SarGeometry.cpp
    sar/imaging/SarAutofocusPhaseTruth.cpp
    sar/imaging/SarCsaGeometry.cpp
    sar/imaging/SarGbp.cpp
    sar/imaging/SarFocusingSelector.cpp
    sar/imaging/SarImageQuality.cpp
    sar/imaging/SarMotionCompensation.cpp
    sar/imaging/SarOmegaKGeometry.cpp
    sar/imaging/SarRda.cpp
    sar/imaging/SarSlowTimeResampling.cpp
    sar/imaging/SarSlowTimeResamplingExecutor.cpp
    sar/runtime/PulseRingBuffer.cpp
    sar/signal/SarFft.cpp
    sar/signal/SarWaveform.cpp
)

# -- core: 会话与运行时粘合层 ------------------------------------------------
set(SAR_CORE_SOURCES
    sar/session/SarReplayFlatbufferCodec.cpp
    sar/session/SarSession.cpp
    sar/session/SarTraceSession.cpp
    sar/session/SarReplaySession.cpp
)

if(ENABLE_INSTALL)
    set(PUBLIC_HEADERS_SAR_MODULE_ENTRY
        ${CMAKE_SOURCE_DIR}/include/1q/sar/sar.hpp
    )
    set(PUBLIC_HEADERS_SAR_CONFIG
        ${CMAKE_SOURCE_DIR}/include/1q/sar/config/SarEnvironmentConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/sar/config/SarHardwareConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/sar/config/SarMissionConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/sar/config/SarPolicyConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/sar/config/SarRuntimeConfigPatch.h
        ${CMAKE_SOURCE_DIR}/include/1q/sar/config/SarSessionConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/sar/config/sar_config.hpp
    )
    set(PUBLIC_HEADERS_SAR_SESSION
        ${CMAKE_SOURCE_DIR}/include/1q/sar/session/SarCycleInput.h
        ${CMAKE_SOURCE_DIR}/include/1q/sar/session/SarCycleResult.h
        ${CMAKE_SOURCE_DIR}/include/1q/sar/session/SarReplaySession.h
        ${CMAKE_SOURCE_DIR}/include/1q/sar/session/SarSession.h
        ${CMAKE_SOURCE_DIR}/include/1q/sar/session/SarSessionFactory.h
        ${CMAKE_SOURCE_DIR}/include/1q/sar/session/SarTraceSession.h
    )

    install(FILES ${PUBLIC_HEADERS_SAR_MODULE_ENTRY}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/sar
    )
    install(FILES ${PUBLIC_HEADERS_SAR_CONFIG}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/sar/config
    )
    install(FILES ${PUBLIC_HEADERS_SAR_SESSION}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/sar/session
    )
endif()
