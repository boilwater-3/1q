# remote_identification_radar 模块源文件与安装清单

# -- core: 会话、配置、运行时（粘合层，依赖轻） ---------------------------------
# 识别算法组件（recognition/ + internal/radar_equations，依赖 SQLite）随阶段 1
# 步骤 3 落地时新增 rir_engine 组件；届时本组件源列表同步扩充会话实现。
set(RIR_CORE_SOURCES
    session/RirSessionConfigValidation.cpp
)

oneq_add_component(rir_core SOURCES ${RIR_CORE_SOURCES})
target_link_libraries(rir_core PRIVATE ${ONEQ_LINK_DEPENDENCIES})
