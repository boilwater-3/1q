# Performance-type test partitions. Current owner: SAR FFT benchmark.

file(GLOB _oneq_performance_sar CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/performance/sar/*_test.cpp")
if(_oneq_performance_sar)
    oneq_add_test_partition(
        TYPE performance
        DOMAIN sar
        SOURCES ${_oneq_performance_sar}
        TIMEOUT 180)
endif()
