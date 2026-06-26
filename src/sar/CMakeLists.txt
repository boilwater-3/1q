# sar 模块源文件与安装清单

# -- engine: 信号处理、几何、回波、成像与输出内核 -----------------------------
# 注：CSA 已标注【未进行设计需求，不再扩展】，相关源文件不再纳入构建；
# 如需恢复，参见 git 历史。
# SarFocusingSelector（内部确定性聚焦算法建议器）、SarRadiometricCalibration
# （内部图像响应辐射定标）与 Omega-K 部件链（Stolt 插值 / 真值验收 / SHA-256 摘要等）
# 已解冻并纳入构建。
include("${CMAKE_CURRENT_LIST_DIR}/SarSources.cmake")

if(MSVC AND MSVC_VERSION EQUAL 1900)
    add_executable(sar_legacy_toolchain_smoke
        sar/smoke/SarLegacyToolchainSmoke.cpp
        sar/imaging/SarPgaSupportGradientTruth.cpp
        sar/imaging/SarPgaPhaseGradientEstimator.cpp
        sar/imaging/SarPgaGradientTruthComparison.cpp
    )
    target_include_directories(sar_legacy_toolchain_smoke PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}"
    )
    set_target_properties(sar_legacy_toolchain_smoke PROPERTIES
        CXX_STANDARD 11
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF
    )
endif()

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
        ${CMAKE_SOURCE_DIR}/include/1q/sar/config/SarRuntimeConfigBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/sar/config/SarSessionConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/sar/config/sar_config.hpp
    )
    set(PUBLIC_HEADERS_SAR_SESSION
        ${CMAKE_SOURCE_DIR}/include/1q/sar/session/SarCycleInput.h
        ${CMAKE_SOURCE_DIR}/include/1q/sar/session/SarCycleResult.h
        ${CMAKE_SOURCE_DIR}/include/1q/sar/session/SarProductDebugView.h
        ${CMAKE_SOURCE_DIR}/include/1q/sar/session/SarProductLifecycleRecorder.h
        ${CMAKE_SOURCE_DIR}/include/1q/sar/session/SarReplaySession.h
        ${CMAKE_SOURCE_DIR}/include/1q/sar/session/SarSession.h
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
