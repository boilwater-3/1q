# CMakeLists.txt
# 项目顶层 CMakeLists.txt：项目主入口，定义基础配置并协调各模块加载顺序
# 仅负责编排生命周期调用顺序，所有选项/工具链/编译器策略/项目 API 均由 cmake/project 统一封装

cmake_minimum_required(VERSION 3.16...3.30)

include(cmake/project/ProjectSetup.cmake)
# 准备阶段：校验 toolchain、确定 C++ 标准、启用 CMP0091/CMP0092 策略。
oneq_prepare_project()

# 项目声明
project(1q
    VERSION 1.0.0
    DESCRIPTION "1q sensor simulation library"
    LANGUAGES CXX C
)

# 配置阶段：加载特性层、依赖、编译器策略与项目 API（单一生命周期入口）。
oneq_configure_project()

# 子目录包含
# src/ 为必需模块；缺失时直接报错，不降级为空构建。
if(EXISTS "${CMAKE_SOURCE_DIR}/src/CMakeLists.txt")
    add_subdirectory(src)
else()
    message(FATAL_ERROR "Required module missing: ${CMAKE_SOURCE_DIR}/src/CMakeLists.txt")
endif()

# 安装阶段：受 ENABLE_INSTALL 门控，须在产物 target 创建之后调用。
oneq_install_project()

# tests / examples 为可选子目录，按开关与存在性按需纳入。
if(ENABLE_TESTING AND EXISTS "${CMAKE_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()

if(ENABLE_EXAMPLES AND EXISTS "${CMAKE_SOURCE_DIR}/examples/CMakeLists.txt")
    add_subdirectory(examples)
endif()

# 汇总阶段：所有产物目录添加完毕后打印最终配置状态。
oneq_print_project_summary()
