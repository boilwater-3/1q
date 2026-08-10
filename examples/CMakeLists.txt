# examples/CMakeLists.txt
# 示例程序构建入口：编排各子目录，不直接声明 target
# examples 层共享工具（如 JSON 配置解析）不属于 oneq 库的 public surface，
# 库内部不消费 JSON；JSON 解析是 example 便利层的能力（见 OQ-7 收口记录）。

if(NOT TARGET ${PROJECT_ALIAS})
    message(FATAL_ERROR "${PROJECT_ALIAS} target not found. Ensure src/CMakeLists.txt is processed first")
endif()

# 共享变量：子目录通过目录作用域继承，内联使用，无需函数传递。
set(ONEQ_EXAMPLE_COMMON_DIR "${CMAKE_CURRENT_SOURCE_DIR}/common")
set(ONEQ_EXAMPLE_COMMON_SOURCES
    "${ONEQ_EXAMPLE_COMMON_DIR}/json_reader.cpp")

# SAR 域示例已删除（旧式 SarModule 包装示例；SAR 集成由
# component_attachment 的 SarSensorComponent + 场景验证工作流覆盖）。

# 消费方集成参考示例：自定义实体-组件（组件基类 + 挂载 + Boost.Signals2 事件）。
add_subdirectory(component_attachment)

# 机动模块示例：仅在 ONEQ_ENABLE_FLIGHT_DYNAMIC 开启时纳入。
if(ONEQ_ENABLE_FLIGHT_DYNAMIC)
    add_subdirectory(flight_dynamic)
endif()

# 批量场景验证：多场景参数扫描 + CSV 指标 + trace 录制/回放回归。
add_subdirectory(batch_validation)
