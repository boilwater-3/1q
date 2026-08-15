# Performance-type test partitions.

file(GLOB _oneq_performance_sar CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/performance/sar/*_test.cpp")
if(_oneq_performance_sar)
    oneq_add_test_partition(
        TYPE performance
        DOMAIN sar
        SOURCES ${_oneq_performance_sar}
        TIMEOUT 180)
endif()

# cross_domain 性能测试的 P95/堆增长预算面向现代编译器（CI: macOS clang）：
# 老 MSVC（v141 14.16）优化强度实测低约 14x，测试在本机既超时又超预算，
# 故 Windows 不注册（本分区不参与 Windows 构建/ctest）。
# TestRegistry 孤儿检查要求磁盘上的 _test.cpp 全部登记：被平台门控排除的
# 文件登记到 performance.platform_excluded 分区（不参与编译，仅满足登记完整性）。
file(GLOB _oneq_performance_cross_domain CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/performance/cross_domain/*_test.cpp")
if(_oneq_performance_cross_domain)
    if(WIN32)
        oneq_register_test_sources("performance.platform_excluded" ${_oneq_performance_cross_domain})
    else()
        oneq_add_test_partition(
            TYPE performance
            DOMAIN cross_domain
            SOURCES ${_oneq_performance_cross_domain}
            TIMEOUT 180)
        set_tests_properties("performance::cross_domain" PROPERTIES RUN_SERIAL TRUE)
    endif()
endif()
