# Test lifecycle entry point. Detailed registrations are grouped by ownership
# under tests/cmake/ so this file remains a stable orchestration surface.
#
# Test registration is type × domain based. New C++ tests must be added to the
# owning partition and registered through oneq_add_test_partition().

if(NOT ENABLE_TESTING)
    return()
endif()

# 识别特征数据库权威 DDL 单源：schemas/recognition/recognition_feature_database.sql。
# 生成测试用头（R"sql(...)" 内嵌字符串），保证测试构造与加载器/建库工具共用同一 schema。
# .sql 变更自动触发 reconfigure（否则生成头陈旧会静默用旧 schema）。
file(READ "${CMAKE_SOURCE_DIR}/schemas/recognition/recognition_feature_database.sql"
     RECOGNITION_FEATURE_DATABASE_SCHEMA_SQL)
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/recognition_feature_database_schema.h.in"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/recognition_feature_database_schema.h"
    @ONLY)
set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" APPEND
             PROPERTY CMAKE_CONFIGURE_DEPENDS
             "${CMAKE_SOURCE_DIR}/schemas/recognition/recognition_feature_database.sql")

include(${CMAKE_CURRENT_LIST_DIR}/cmake/TestSupport.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/TestRegistry.cmake)
if(NOT TEST_SOURCES)
    message(STATUS "No test sources defined in ${CMAKE_CURRENT_SOURCE_DIR}")
    message(STATUS "  Add .cpp files to unit/integration/contract to enable testing")
    return()
endif()

include(${CMAKE_CURRENT_LIST_DIR}/cmake/TestTargets.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/FlightDynamicPartitions.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/ContractGuards.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/CompatibilityChecks.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/CoverageRunner.cmake)

# 批量场景验证框架（consumer 分区，树内注册的独立可执行程序；非 GoogleTest
# 分区，见 tests/README.md 注记）。依赖 examples 层共享便利层与配置 JSON，
# 单向依赖关系与 tests/unit/examples 直接编译 examples 源码的先例一致。
add_subdirectory(consumer/batch_validation)

# Phase 0 layout guard: reject flat _test.cpp at tests/ root and unknown type dirs.
oneq_add_script_guard(test_layout_guard check_test_layout.cmake)
set_tests_properties(test_layout_guard PROPERTIES LABELS "contract")

# Finalize registry: orphan/duplicate checks + partition statistics output.
oneq_finalize_test_registry()
