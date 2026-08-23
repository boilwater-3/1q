# examples/CMakeLists.txt
# 示例程序构建入口（主示例本体 + 每场景可执行 + 共享便利层）：
#   core/        组件基类 / 实体 / 世界 / 信号（Boost.Signals2，header-only，
#                零自定义分发层）+ 场景共享状态
#   components/  每个仿真模块一个组件（飞行 / AR / ESR / ECM / EOS / SBIRS /
#                SAR / RIR / 融合 / 推演 / 威胁评估），挂载到实体后按挂载序步进
#   logger/      集成端日志设施（双后端：spdlog 平台直用；Windows 走
#                std::ofstream 文件后端，CA_LOG_BACKEND_SPDLOG=0，仿库内
#                ProjectFileLog 口径，全平台均可构建）
#   app/         装配层：通用 runner（component_attachment_demo，--scene 跑
#                任意场景）+ 可视化 CSV 落盘 + 指令路由
#   scenes/      场景描述 JSON/MD + 场景加载代码（scene_data/scene_script/
#                area_division）
#   basic_config/六域基础 session 配置模板 + RIR 识别库资产
#   common/      examples 与 tests 共享便利层（JSON 解析 / CSV / 配置加载器 /
#                可视化查看器），不属于 oneq 库的 public surface（库内部不
#                消费 JSON；见 OQ-7 收口记录）
# 评估层示例（precision_evaluation_demo，双星定位精度评估）的场景位于
# scenes/sbirs_dual_sat_fix/。

if(NOT TARGET ${PROJECT_ALIAS})
    message(FATAL_ERROR "${PROJECT_ALIAS} target not found. Ensure src/CMakeLists.txt is processed first")
endif()

# 共享变量：内联使用，无需函数传递。
set(ONEQ_EXAMPLE_COMMON_DIR "${CMAKE_CURRENT_SOURCE_DIR}/common")
set(ONEQ_EXAMPLE_COMMON_SOURCES
    "${ONEQ_EXAMPLE_COMMON_DIR}/json_reader.cpp")

# ---------------------------------------------------------------------------
# 通用 runner（对外 target 名保持不变）：--scene 跑任意场景 JSON。
# 验收日志通道提示：SBIRS 宽窄视场验收量写入 sbirs_acceptance.log，开关为库编译期选项。
if(NOT ONEQ_ENABLE_SBIRS_ACCEPTANCE_LOG)
    message(STATUS "examples: ONEQ_ENABLE_SBIRS_ACCEPTANCE_LOG=OFF -> no "
        "sbirs_acceptance.log; reconfigure with -DONEQ_ENABLE_SBIRS_ACCEPTANCE_LOG=ON")
endif()

add_executable(component_attachment_demo
    app/main.cpp
    app/demo_config.cpp
    app/outputs.cpp
    app/command_routing.cpp
    scenes/scene_data.cpp
    scenes/area_division.cpp
    scenes/scene_script.cpp
    components/flight_component.cpp
    components/esr_sensor_component.cpp
    components/ecm_sensor_component.cpp
    components/ar_sensor_component.cpp
    components/eos_sensor_component.cpp
    components/sbirs_sensor_component.cpp
    components/sar_sensor_component.cpp
    components/rir_sensor_component.cpp
    components/fusion_component.cpp
    components/inference_component.cpp
    components/threat_component.cpp
    logger/logger.cpp)

# Boost.Signals2 事件库：与 tests 分区同源解析（tests/cmake/TestSupport.cmake）。
# conan 路径走 find_package（包配置文件由 conan 生成）；none 路径下
# VendorPackages.cmake 已直接定义 Boost::boost INTERFACE 目标，无需再 find。
if(NOT TARGET Boost::boost)
    find_package(Boost CONFIG REQUIRED)
endif()
target_link_libraries(component_attachment_demo PRIVATE
    ${PROJECT_ALIAS}
    Boost::boost)
# spdlog：集成端日志后端（logger.h 的 CA_LOG_EVENT / CA_LOG_VIEW），仅平台启用
# 时链接；Windows 文件后端分支只依赖标准库。示例源含中文 UTF-8 串字面量，
# MSVC 的 /utf-8 经 core 的 INTERFACE 选项（CompilerMSVC.cmake）随链接传播。
if(PROJECT_ENABLE_SPDLOG)
    target_link_libraries(component_attachment_demo PRIVATE ${PROJECT_SPDLOG_TARGET})
endif()
# 日志后端开关注入（仿库内 oneq_add_component 的 PROJECT_LOG_BACKEND_* 先例，
# 见 ProjectTargets.cmake）。
target_compile_definitions(component_attachment_demo PRIVATE
    CA_LOG_BACKEND_SPDLOG=$<BOOL:${PROJECT_ENABLE_SPDLOG}>
    CA_LIBRARY_FILE_LOG=$<BOOL:${ONEQ_ENABLE_FILE_LOG}>)

target_include_directories(component_attachment_demo PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${ONEQ_EXAMPLE_COMMON_DIR}")

# 基础配置模板（examples/basic_config/）+ 场景描述目录
# （scenes/*.json，app/main.cpp 的 --scene 默认值）+ 默认输出目录
# （examples/log/，运行时产物不入版本控制，见 .gitignore）。
target_compile_definitions(component_attachment_demo PRIVATE
    SCENE_CONFIG_DIR="${CMAKE_SOURCE_DIR}/examples/basic_config"
    CA_SCENE_DIR="${CMAKE_SOURCE_DIR}/examples/scenes"
    CA_DEFAULT_OUTPUT_DIR="${CMAKE_SOURCE_DIR}/examples/log"
    CA_RIR_DATABASE_PATH="${CMAKE_SOURCE_DIR}/examples/basic_config/remote_identification_radar/target_feature_database_v1.1.db")

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

# 集成端日志模式（编译期宏，见 logger/logger_modes.h）：
# CA_VIEW_LOG_MODE —— summary（每周期摘要，默认）| nonnominal | delta
#                    落盘频率用 --view-every / 场景 view_log_every_cycles，不靠换模式
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

# ---------------------------------------------------------------------------
# 评估层示例（对外 target 名保持不变）：双星 SBIRS + 融合 + 推演的真值对照
# 五项误差（角度/双星交会/速度/落点/发射点）+ AHP 综合评分。评估会话内部自持
# 双星 + 融合 + 推演整链（评估层编排，非传感器组件模型）。几何与周期由
# scenes/sbirs_dual_sat_fix/*.json 驱动。
add_executable(precision_evaluation_demo
    precision_evaluation/precision_evaluation_demo.cpp)
target_link_libraries(precision_evaluation_demo PRIVATE ${PROJECT_ALIAS})
target_include_directories(precision_evaluation_demo PRIVATE
    "${ONEQ_EXAMPLE_COMMON_DIR}")
target_sources(precision_evaluation_demo PRIVATE ${ONEQ_EXAMPLE_COMMON_SOURCES})
target_compile_definitions(precision_evaluation_demo PRIVATE
    PE_DEFAULT_SCENE_FILE="${CMAKE_SOURCE_DIR}/examples/scenes/sbirs_dual_sat_fix/sbirs_dual_sat_fix.json"
    PE_DEFAULT_OUTPUT_DIR="${CMAKE_SOURCE_DIR}/examples/log"
    PE_ACCEPTANCE_LOG_ENABLED=$<BOOL:${ONEQ_ENABLE_PRECISION_EVALUATION_LOG}>)

if(NOT ONEQ_ENABLE_PRECISION_EVALUATION_LOG)
    message(STATUS "precision_evaluation: ONEQ_ENABLE_PRECISION_EVALUATION_LOG=OFF -> no "
        "precision_acceptance.log; reconfigure with -DONEQ_ENABLE_PRECISION_EVALUATION_LOG=ON")
endif()
if(NOT ONEQ_ENABLE_SBIRS_ACCEPTANCE_LOG OR NOT ONEQ_ENABLE_FUSION_ACCEPTANCE_LOG
    OR NOT ONEQ_ENABLE_INFERENCE_ACCEPTANCE_LOG)
    message(STATUS "precision_evaluation: also enable "
        "ONEQ_ENABLE_SBIRS_ACCEPTANCE_LOG / FUSION / INFERENCE for "
        "sbirs/fusion/inference_acceptance.log under the scene output dir")
endif()

# 示例冒烟不进 ctest（示例是集成参考而非测试门禁，release 全量跑 ~25 s、
# debug 下 400 周期 FD 全链可到 2-4 分钟，与单测时间预期不匹配）。手动运行：
#   ./build/<preset>/bin/component_attachment_demo
#       [--scene <path>] [--cycles <n>] [--output-dir <dir>]   （基线 400 周期）
#   ./build/<preset>/bin/component_attachment_demo --scene \
#       examples/scenes/fleet_patrol_multi_zone/fleet_patrol_multi_zone.json \
#       --cycles 120 --output-dir <dir>                        （多机巡逻冒烟）
#   python3 examples/common/viz/build_viewer.py --check <dir>  （可视化契约回归）
