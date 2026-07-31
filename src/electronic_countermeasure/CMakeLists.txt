# electronic_countermeasure 模块源文件清单。

oneq_add_component(ecm_core SOURCES
    DeceptionEngagementManager.cpp
    EcmConfigValidator.cpp
    EcmResourceLedger.cpp
    EcmWaveformFactory.cpp
    EcmEsrAdapter.cpp
    EcmReplayFlatbufferCodec.cpp
    EcmReplaySession.cpp
    EcmTraceSession.cpp
    EcmSession.cpp)
target_link_libraries(ecm_core PRIVATE ${ONEQ_LINK_DEPENDENCIES})
oneq_add_flatbuffers_schemas(
    TARGET ecm_core
    MODULE electronic_countermeasure
    SCHEMAS ecm_replay.fbs)
