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

file(GLOB _oneq_performance_cross_domain CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/performance/cross_domain/*_test.cpp")
if(_oneq_performance_cross_domain)
    oneq_add_test_partition(
        TYPE performance
        DOMAIN cross_domain
        SOURCES ${_oneq_performance_cross_domain}
        TIMEOUT 180)
    set_tests_properties("performance::cross_domain" PROPERTIES RUN_SERIAL TRUE)
endif()
