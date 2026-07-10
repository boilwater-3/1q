# 项目顶层 CMakeLists.txt
# 本文件是项目的主入口，定义项目基础配置并协调各个模块

cmake_minimum_required(VERSION 3.16...3.30)

include(cmake/project/ProjectSetup.cmake)
oneq_prepare_project()

# 项目声明
project(1q
    VERSION 1.0.0
    DESCRIPTION "1q sensor simulation library"
    LANGUAGES CXX C
)

# All project setup, options, tooling, compiler policy and project APIs are
# loaded behind one lifecycle entry point; this file only orchestrates order.
oneq_configure_project()

# 子目录包含
if(EXISTS "${CMAKE_SOURCE_DIR}/src/CMakeLists.txt")
    add_subdirectory(src)
else()
    message(FATAL_ERROR "Required module missing: ${CMAKE_SOURCE_DIR}/src/CMakeLists.txt")
endif()

oneq_install_project()

if(ENABLE_TESTING AND EXISTS "${CMAKE_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()

if(ENABLE_EXAMPLES AND EXISTS "${CMAKE_SOURCE_DIR}/examples/CMakeLists.txt")
    add_subdirectory(examples)
endif()

if(EXISTS "${CMAKE_SOURCE_DIR}/tools/CMakeLists.txt")
    add_subdirectory(tools)
endif()

oneq_print_project_summary()
