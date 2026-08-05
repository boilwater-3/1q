# behavior_layer 示例：EnTT 驱动的消费方业务层参考实现（组件/系统/装配）。
# EnTT 为 example 侧依赖（conanfile.py 基础清单，header-only，C++17），
# 不进入库本体；契约见 docs/review/Bahavior.md §5 与本目录 README.md。
#
# 三域配置加载器位于 examples/common/config_loaders/<域>/（共享便利层），
# 经 ONEQ_EXAMPLE_COMMON_DIR 解析；配置 JSON 经 SCENE_CONFIG_DIR 注入
# （examples/configs/，与 scene 先例一致）。
#
# flight_dynamic 接入（可选）：ONEQ_ENABLE_FLIGHT_DYNAMIC=ON 时注入编译开关
# 与 JSBSim aircraft 数据根目录（FD 路径编译进 flight_system.cpp）；关闭时
# flight_system 回退运动学推进，本 target 照常构建（对齐 ci.yml 路径）。

find_package(EnTT CONFIG REQUIRED)

add_executable(behavior_layer_demo
    behavior_layer_demo.cpp
    systems.cpp
    assembly.cpp
    flight_system.cpp)
target_link_libraries(behavior_layer_demo PRIVATE ${PROJECT_ALIAS} EnTT::EnTT)
target_include_directories(behavior_layer_demo PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${ONEQ_EXAMPLE_COMMON_DIR}")
target_compile_definitions(behavior_layer_demo PRIVATE
    SCENE_CONFIG_DIR="${CMAKE_SOURCE_DIR}/examples/configs")
target_sources(behavior_layer_demo PRIVATE ${ONEQ_EXAMPLE_COMMON_SOURCES})

if(ONEQ_ENABLE_FLIGHT_DYNAMIC)
    # FD 数据根目录解析（与 examples/flight_dynamic setup_fd_example 同款：
    # ONEQ_JSBSIM_DATA_ROOT_DIR 回退 third_party/jsbsim）。
    if(DEFINED ONEQ_JSBSIM_DATA_ROOT_DIR AND NOT ONEQ_JSBSIM_DATA_ROOT_DIR STREQUAL "")
        set(FD_JSBSIM_ROOT_DIR "${ONEQ_JSBSIM_DATA_ROOT_DIR}")
    else()
        set(FD_JSBSIM_ROOT_DIR "${CMAKE_SOURCE_DIR}/third_party/jsbsim")
    endif()
    target_compile_definitions(behavior_layer_demo PRIVATE
        ONEQ_BL_FLIGHT_DYNAMIC_ENABLED=1
        FD_JSBSIM_ROOT_DIR="${FD_JSBSIM_ROOT_DIR}")
    # 公共静态库不传播组件级 PRIVATE 依赖（src/CMakeLists.txt 链接闭包注释），
    # FD 消费方需显式链接 JSBSim（对齐 tests/cmake/FlightDynamicPartitions.cmake）。
    target_link_libraries(behavior_layer_demo PRIVATE JSBSim::JSBSim)
endif()

if(BUILD_TESTING)
    # EnTT 依赖链冒烟：仅当 ENABLE_EXAMPLES=ON 时该 target 存在（对齐 batch_validation 先例）。
    # FD ON 时 demo 运行真实飞行仿真（2000 子步 JSBSim；nightly debug 模式吃紧，
    # 超时放宽到与 fd 测试分区一致；release 本地运行远快于此）。
    add_test(NAME examples::behavior_layer_demo COMMAND $<TARGET_FILE:behavior_layer_demo>)
    set_tests_properties(examples::behavior_layer_demo PROPERTIES
        LABELS "examples;behavior_layer"
        TIMEOUT 180)
endif()
