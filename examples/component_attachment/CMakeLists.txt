# examples/component_attachment/CMakeLists.txt
# 自定义实体-组件示例：core/ 提供组件基类 / 实体 / 世界 / 信号；事件机制使用
# C++ 常见开源事件库 Boost.Signals2（header-only，boost 已在 conanfile 基础
# 依赖）——零自定义分发层；components/ 每个仿真模块对应一个组件（飞行 / AR /
# ESR / EOS / SBIRS / SAR / 融合 / 威胁评估），挂载到平台实体后按挂载序周期步进。
# 集成端日志设施（logger/logger.h）直接使用 spdlog（conanfile 非 Windows
# 依赖，ConanPackages.cmake 的 PROJECT_ENABLE_SPDLOG）；Windows 上 spdlog 关闭，
# 本示例整体不构建。

if(NOT PROJECT_ENABLE_SPDLOG)
    message(STATUS "component_attachment: disabled (spdlog unavailable on this platform)")
    return()
endif()

# 验收日志通道提示：SBIRS 宽窄视场验收量（覆盖区/驻留/焦平面脱靶量/信号能量/
# 连续命中计数器）经 [SbirsAccept] 事件流写入库日志，开关为库编译期选项。
if(NOT ONEQ_ENABLE_SBIRS_ACCEPTANCE_LOG)
    message(STATUS "component_attachment: ONEQ_ENABLE_SBIRS_ACCEPTANCE_LOG=OFF -> no "
        "[SbirsAccept] events; reconfigure with -DONEQ_ENABLE_SBIRS_ACCEPTANCE_LOG=ON "
        "to enable (events land in <output-dir>/1q_library.log)")
endif()

add_executable(component_attachment_demo
    component_attachment_demo.cpp
    demo_config.cpp
    scene_data.cpp
    area_division.cpp
    scene_script.cpp
    demo_output.cpp
    components/flight_component.cpp
    components/ar_sensor_component.cpp
    components/esr_sensor_component.cpp
    components/eos_sensor_component.cpp
    components/sbirs_sensor_component.cpp
    components/sar_sensor_component.cpp
    components/fusion_component.cpp
    components/inference_component.cpp
    components/threat_component.cpp
    logger/logger.cpp)

# Boost.Signals2 事件库：与 tests 分区同源解析（tests/cmake/TestSupport.cmake）；
# spdlog：集成端日志后端（logger.h 的 CA_LOG_EVENT / CA_LOG_VIEW）。
find_package(Boost CONFIG REQUIRED)
target_link_libraries(component_attachment_demo PRIVATE
    ${PROJECT_ALIAS}
    Boost::boost
    ${PROJECT_SPDLOG_TARGET})

target_include_directories(component_attachment_demo PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${ONEQ_EXAMPLE_COMMON_DIR}")

# 共享配置 JSON（examples/configs/）+ 场景描述目录
# （scenes/*.json，scene_data.h 的 --scene 默认值）+ 默认输出目录
# （demo_config.h 的 kDefaultOutputDir：仓库内 examples/component_attachment/log/，
# 运行时产物不入版本控制，见 .gitignore）。
target_compile_definitions(component_attachment_demo PRIVATE
    SCENE_CONFIG_DIR="${CMAKE_SOURCE_DIR}/examples/configs"
    CA_SCENE_DIR="${CMAKE_SOURCE_DIR}/examples/component_attachment/scenes"
    CA_DEFAULT_OUTPUT_DIR="${CMAKE_SOURCE_DIR}/examples/component_attachment/log")

# 集成端日志模式（编译期宏，见 logger/logger_modes.h）：
# CA_VIEW_LOG_MODE —— summary（每周期摘要）| nonnominal（只落非标称行）|
#                    delta（跨周期增量，默认，同 logger_modes.h）
# CA_EVENT_LOG_MODE —— all（逐条全量）| key（只记关键，默认，同 logger_modes.h）|
#                      aggregate（周期聚合）
# 不传（空）时用 logger_modes.h 源码默认；显式传值则覆盖源码默认。
set(CA_VIEW_LOG_MODE "" CACHE STRING
    "component_attachment 视图日志模式: summary|nonnominal|delta (空=用 logger_modes.h 默认)")
set(CA_EVENT_LOG_MODE "" CACHE STRING
    "component_attachment 事件日志模式: all|key|aggregate (空=用 logger_modes.h 默认)")

if(CA_VIEW_LOG_MODE)
    if(CA_VIEW_LOG_MODE STREQUAL "summary")
        target_compile_definitions(component_attachment_demo PRIVATE CA_VIEW_LOG_MODE_SUMMARY)
    elseif(CA_VIEW_LOG_MODE STREQUAL "nonnominal")
        target_compile_definitions(component_attachment_demo PRIVATE CA_VIEW_LOG_MODE_NONNOMINAL)
    elseif(CA_VIEW_LOG_MODE STREQUAL "delta")
        target_compile_definitions(component_attachment_demo PRIVATE CA_VIEW_LOG_MODE_DELTA)
    else()
        message(FATAL_ERROR "CA_VIEW_LOG_MODE 非法值 '${CA_VIEW_LOG_MODE}'（可选 summary|nonnominal|delta）")
    endif()
endif()

if(CA_EVENT_LOG_MODE)
    if(CA_EVENT_LOG_MODE STREQUAL "all")
        target_compile_definitions(component_attachment_demo PRIVATE CA_EVENT_LOG_MODE_ALL)
    elseif(CA_EVENT_LOG_MODE STREQUAL "key")
        target_compile_definitions(component_attachment_demo PRIVATE CA_EVENT_LOG_MODE_KEY)
    elseif(CA_EVENT_LOG_MODE STREQUAL "aggregate")
        target_compile_definitions(component_attachment_demo PRIVATE CA_EVENT_LOG_MODE_AGGREGATE)
    else()
        message(FATAL_ERROR "CA_EVENT_LOG_MODE 非法值 '${CA_EVENT_LOG_MODE}'（可选 all|key|aggregate）")
    endif()
endif()

target_sources(component_attachment_demo PRIVATE ${ONEQ_EXAMPLE_COMMON_SOURCES})

# 飞行组件：ONEQ_ENABLE_FLIGHT_DYNAMIC 开启时接入 JSBSim 真实飞行仿真
# （运动学回退路径始终可用）。
if(ONEQ_ENABLE_FLIGHT_DYNAMIC)
    if(NOT DEFINED ONEQ_JSBSIM_DATA_ROOT_DIR)
        set(ONEQ_JSBSIM_DATA_ROOT_DIR "${CMAKE_SOURCE_DIR}/third_party/jsbsim")
    endif()
    target_compile_definitions(component_attachment_demo PRIVATE
        ONEQ_CA_FLIGHT_DYNAMIC_ENABLED=1
        FD_JSBSIM_ROOT_DIR="${ONEQ_JSBSIM_DATA_ROOT_DIR}")
    target_link_libraries(component_attachment_demo PRIVATE JSBSim::JSBSim)
endif()

# 示例冒烟不进 ctest（示例是集成参考而非测试门禁，release 全量跑 ~25 s、
# debug 下 400 周期 FD 全链可到 2-4 分钟，与单测时间预期不匹配）。手动运行：
#   ./build/<preset>/bin/component_attachment_demo
#       [--scene <path>] [--cycles <n>] [--output-dir <dir>]   （基线 400 周期）
#   ./build/<preset>/bin/component_attachment_demo --scene \
#       examples/component_attachment/scenes/fleet_patrol_multi_zone/fleet_patrol_multi_zone.json \
#       --cycles 120 --output-dir <dir>                        （多机巡逻冒烟）
#   python3 examples/common/viz/build_viewer.py --check <dir>  （可视化契约回归）
