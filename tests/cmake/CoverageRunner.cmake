# Coverage-only mapping executable.  It owns every non-consumer test source
# exactly once, but is disabled in CTest so normal coverage runs collect
# profraw from the real type×domain partitions rather than executing twice.

if(NOT ENABLE_COVERAGE)
    return()
endif()

file(GLOB_RECURSE _oneq_coverage_sources CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/*_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/integration/*_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/replay/*_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/contract/*_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/performance/*_test.cpp")
if(NOT ONEQ_ENABLE_FLIGHT_DYNAMIC)
    list(FILTER _oneq_coverage_sources EXCLUDE REGEX "/unit/flight_dynamic/")
endif()

add_1q_gtest(${PROJECT_NAME}_coverage_tests coverage_runner 0 ${_oneq_coverage_sources})
target_sources(${PROJECT_NAME}_coverage_tests PRIVATE
    "${CMAKE_SOURCE_DIR}/examples/common/json_reader.cpp")
target_include_directories(${PROJECT_NAME}_coverage_tests PRIVATE
    "${CMAKE_SOURCE_DIR}/examples/common")
if(TARGET oneq_flatbuffers_headers)
    add_dependencies(${PROJECT_NAME}_coverage_tests oneq_flatbuffers_headers)
endif()
set_tests_properties("coverage_runner::${PROJECT_NAME}_coverage_tests"
    PROPERTIES DISABLED TRUE)
