# remote_identification_radar 模块源文件与安装清单

# -- engine: 识别算法（特征提取/积累/匹配/数据库加载；依赖 SQLite） --------------
set(RIR_ENGINE_SOURCES
    internal/RirRadarEquations.cpp
    recognition/RcsFeatureExtractor.cpp
    recognition/MotionFeatureExtractor.cpp
    recognition/PolarizationFeatureExtractor.cpp
    recognition/RangeProfileFeatureExtractor.cpp
    recognition/RecognitionObservationBuilder.cpp
    recognition/RecognitionFeatureDatabase.cpp
    recognition/RecognitionMatcher.cpp
    recognition/RecognitionTracker.cpp
)

# -- core: 会话、配置、运行时（粘合层，依赖轻） ---------------------------------
set(RIR_CORE_SOURCES
    session/RirSessionConfigValidation.cpp
    session/RirInputValidation.cpp
    session/RirSession.cpp
    session/RirReplayFlatbufferCodec.cpp
    runtime/RirController.cpp
)

oneq_add_component(rir_engine SOURCES ${RIR_ENGINE_SOURCES})
oneq_add_component(rir_core SOURCES ${RIR_CORE_SOURCES})
# 识别特征库加载（SQLite 只读读取器）；仅 remote_identification_radar 内部消费，
# PRIVATE 不进公共链接面。
target_link_libraries(rir_engine PRIVATE SQLite::SQLite3)
# 检测物理子集依赖 boost::math（header-only，含于公共链接依赖；阶段 2-M M1）。
target_link_libraries(rir_engine PRIVATE ${ONEQ_LINK_DEPENDENCIES})
target_link_libraries(rir_core PRIVATE ${ONEQ_LINK_DEPENDENCIES})
oneq_add_flatbuffers_schemas(
    TARGET rir_core
    MODULE remote_identification_radar
    SCHEMAS rir_replay.fbs)
