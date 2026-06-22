# SAR source manifest shared by production targets and contract checks.

set(SAR_ENGINE_SOURCES
    sar/echo/SarEcho.cpp
    sar/geometry/SarAntenna.cpp
    sar/geometry/SarGeometry.cpp
    sar/imaging/SarAutofocusPhaseTruth.cpp
    sar/imaging/SarGbp.cpp
    sar/imaging/SarImageQuality.cpp
    sar/imaging/SarMotionCompensation.cpp
    sar/imaging/SarPhaseReference.cpp
    sar/imaging/SarPgaSupportGradientTruth.cpp
    sar/imaging/SarPgaPhaseGradientEstimator.cpp
    sar/imaging/SarPgaGradientTruthComparison.cpp
    sar/imaging/SarRda.cpp
    sar/output/ImageFormatter.cpp
    sar/imaging/SarSlowTimeResampling.cpp
    sar/imaging/SarSlowTimeResamplingExecutor.cpp
    sar/runtime/PulseRingBuffer.cpp
    sar/signal/SarFft.cpp
    sar/signal/SarWaveform.cpp
)

set(SAR_CORE_SOURCES
    sar/session/SarRawHistoryBuilder.cpp
    sar/session/SarReplayFlatbufferCodec.cpp
    sar/session/SarRuntimeConfigValidation.cpp
    sar/session/SarSession.cpp
    sar/session/SarTraceSession.cpp
    sar/session/SarReplaySession.cpp
)
