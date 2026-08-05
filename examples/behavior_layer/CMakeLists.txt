# behavior_layer 示例：EnTT 驱动的消费方业务层参考实现（组件/系统/装配）。
# EnTT 为 example 侧依赖（conanfile.py 基础清单，header-only，C++17），
# 不进入库本体；契约见 docs/review/Bahavior.md §5 与本目录 README.md。
#
# 三域 config_loader（airborne_radar / electronic_warfare / electro_optical）
# 同名文件，demo 以相对路径 include 避免歧义；配置 JSON 经 SCENE_CONFIG_DIR
# 注入（examples/configs/，与 scene 先例一致）。

find_package(EnTT CONFIG REQUIRED)

add_executable(behavior_layer_demo
    behavior_layer_demo.cpp
    systems.cpp
    assembly.cpp)
target_link_libraries(behavior_layer_demo PRIVATE ${PROJECT_ALIAS} EnTT::EnTT)
target_include_directories(behavior_layer_demo PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${ONEQ_EXAMPLE_COMMON_DIR}")
target_compile_definitions(behavior_layer_demo PRIVATE
    SCENE_CONFIG_DIR="${CMAKE_SOURCE_DIR}/examples/configs")
target_sources(behavior_layer_demo PRIVATE ${ONEQ_EXAMPLE_COMMON_SOURCES})

if(BUILD_TESTING)
    # EnTT 依赖链冒烟：仅当 ENABLE_EXAMPLES=ON 时该 target 存在（对齐 batch_validation 先例）。
    add_test(NAME examples::behavior_layer_demo COMMAND $<TARGET_FILE:behavior_layer_demo>)
    set_tests_properties(examples::behavior_layer_demo PROPERTIES
        LABELS "examples;behavior_layer"
        TIMEOUT 60)
endif()
