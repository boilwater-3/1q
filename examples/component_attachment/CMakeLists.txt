# examples/component_attachment/CMakeLists.txt
# 第二种示例模式：自定义实体-组件（不依赖 EnTT）。
# core/ 提供组件基类 / 实体 / 世界 / 信号；事件机制使用 C++ 常见开源事件库
# Boost.Signals2（header-only，boost 已在 conanfile 基础依赖）——零自定义分发层；
# components/ 每个仿真模块对应一个组件（飞行 / AR / ESR / EOS / 融合），
# 挂载到平台实体后按挂载序周期步进。
# 集成端日志设施（components/demo_log.h）直接使用 spdlog（conanfile 非 Windows
# 依赖，ConanPackages.cmake 的 PROJECT_ENABLE_SPDLOG）；Windows 上 spdlog 关闭，
# 本示例整体不构建。

if(NOT PROJECT_ENABLE_SPDLOG)
    message(STATUS "component_attachment: disabled (spdlog unavailable on this platform)")
    return()
endif()

add_executable(component_attachment_demo
    component_attachment_demo.cpp
    demo_config.cpp
    scene_data.cpp
    scene_script.cpp
    demo_output.cpp
    components/flight_component.cpp
    components/ar_sensor_component.cpp
    components/esr_sensor_component.cpp
    components/eos_sensor_component.cpp
    components/sbirs_sensor_component.cpp
    components/sar_sensor_component.cpp
    components/fusion_component.cpp
    components/threat_component.cpp
    components/demo_log.cpp)

# Boost.Signals2 事件库：与 tests 分区同源解析（tests/cmake/TestSupport.cmake）；
# spdlog：集成端日志后端（demo_log.h 的 CA_LOG_EVENT / CA_LOG_VIEW）。
find_package(Boost CONFIG REQUIRED)
target_link_libraries(component_attachment_demo PRIVATE
    ${PROJECT_ALIAS}
    Boost::boost
    ${PROJECT_SPDLOG_TARGET})

target_include_directories(component_attachment_demo PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${ONEQ_EXAMPLE_COMMON_DIR}")

# 共享配置 JSON（与 component_entt 同源 examples/configs/）+ 场景描述目录
# （scenes/*.json，scene_data.h 的 --scene 默认值）+ 默认输出目录
# （demo_config.h 的 kDefaultOutputDir：仓库内 examples/component_attachment/log/，
# 运行时产物不入版本控制，见 .gitignore）。
target_compile_definitions(component_attachment_demo PRIVATE
    SCENE_CONFIG_DIR="${CMAKE_SOURCE_DIR}/examples/configs"
    CA_SCENE_DIR="${CMAKE_SOURCE_DIR}/examples/component_attachment/scenes"
    CA_DEFAULT_OUTPUT_DIR="${CMAKE_SOURCE_DIR}/examples/component_attachment/log")

# 集成端日志模式（编译期宏，见 components/demo_log_modes.h）：
# CA_VIEW_LOG_MODE —— summary（每周期摘要）| nonnominal（只落非标称行）|
#                    delta（跨周期增量，默认，同 demo_log_modes.h）
# CA_EVENT_LOG_MODE —— all（逐条全量）| key（只记关键，默认，同 demo_log_modes.h）|
#                      aggregate（周期聚合）
# 不传（空）时用 demo_log_modes.h 源码默认；显式传值则覆盖源码默认。
set(CA_VIEW_LOG_MODE "" CACHE STRING
    "component_attachment 视图日志模式: summary|nonnominal|delta (空=用 demo_log_modes.h 默认)")
set(CA_EVENT_LOG_MODE "" CACHE STRING
    "component_attachment 事件日志模式: all|key|aggregate (空=用 demo_log_modes.h 默认)")

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
# （运动学回退路径始终可用，与 component_entt flight_system 同构）。
if(ONEQ_ENABLE_FLIGHT_DYNAMIC)
    if(NOT DEFINED ONEQ_JSBSIM_DATA_ROOT_DIR)
        set(ONEQ_JSBSIM_DATA_ROOT_DIR "${CMAKE_SOURCE_DIR}/third_party/jsbsim")
    endif()
    target_compile_definitions(component_attachment_demo PRIVATE
        ONEQ_CA_FLIGHT_DYNAMIC_ENABLED=1
        FD_JSBSIM_ROOT_DIR="${ONEQ_JSBSIM_DATA_ROOT_DIR}")
    target_link_libraries(component_attachment_demo PRIVATE JSBSim::JSBSim)
endif()

if(BUILD_TESTING)
    add_test(NAME examples::component_attachment_demo
        COMMAND $<TARGET_FILE:component_attachment_demo>
            --output-dir "${CMAKE_BINARY_DIR}/component_attachment_viz")
    set_tests_properties(examples::component_attachment_demo PROPERTIES
        LABELS "examples;component_attachment"
        TIMEOUT 60)
    # 多机区域巡逻场景冒烟（3 机各自区域任务 + 空中/地面目标）：
    # --cycles 120 控制时长（完整 600 周期留给手动跑；FD 模式 3 机子步进开销
    # 按 release 本机实测 120 周期 ~20 s）。
    add_test(NAME examples::component_attachment_fleet_patrol
        COMMAND $<TARGET_FILE:component_attachment_demo>
            --scene "${CMAKE_SOURCE_DIR}/examples/component_attachment/scenes/fleet_patrol_multi_zone.json"
            --cycles 120 --output-dir "${CMAKE_BINARY_DIR}/component_attachment_fleet_viz")
    set_tests_properties(examples::component_attachment_fleet_patrol PROPERTIES
        LABELS "examples;component_attachment"
        TIMEOUT 60)
    # 可视化导出回归：共享查看器 build_viewer.py --check（统一契约 v2：
    # 多机行数 == 周期数 × 机数 + 表头 + ENU 原点不变量）。Python3 缺失时不注册。
    find_package(Python3 COMPONENTS Interpreter QUIET)
    if(Python3_Interpreter_FOUND)
        add_test(NAME examples::component_attachment_viewer_check
            COMMAND ${Python3_EXECUTABLE}
                "${CMAKE_SOURCE_DIR}/examples/common/viz/build_viewer.py" --check
                "${CMAKE_BINARY_DIR}/component_attachment_fleet_viz")
        set_tests_properties(examples::component_attachment_viewer_check PROPERTIES
            LABELS "examples;component_attachment"
            DEPENDS examples::component_attachment_fleet_patrol
            TIMEOUT 60)
    endif()
endif()
