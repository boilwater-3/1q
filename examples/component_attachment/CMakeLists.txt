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
    scene_script.cpp
    demo_output.cpp
    components/flight_component.cpp
    components/ar_sensor_component.cpp
    components/esr_sensor_component.cpp
    components/eos_sensor_component.cpp
    components/sbirs_sensor_component.cpp
    components/sar_sensor_component.cpp
    components/fusion_component.cpp
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

# 共享配置 JSON（与 behavior_layer 同源 examples/configs/）。
target_compile_definitions(component_attachment_demo PRIVATE
    SCENE_CONFIG_DIR="${CMAKE_SOURCE_DIR}/examples/configs")

target_sources(component_attachment_demo PRIVATE ${ONEQ_EXAMPLE_COMMON_SOURCES})

# 飞行组件：ONEQ_ENABLE_FLIGHT_DYNAMIC 开启时接入 JSBSim 真实飞行仿真
# （运动学回退路径始终可用，与 behavior_layer flight_system 同构）。
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
endif()
