# Integration-type test partitions: one executable per owner domain.

function(_oneq_add_integration_partition domain)
    cmake_parse_arguments(_oneq_int_part "" "" "LINK_LIBS;INCLUDE_DIRS" ${ARGN})
    file(GLOB _sources CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/integration/${domain}/*_test.cpp")
    if(_sources)
        oneq_add_test_partition(
            TYPE integration
            DOMAIN ${domain}
            SOURCES ${_sources}
            TIMEOUT 120
            LABELS ci_required
            LINK_LIBS ${_oneq_int_part_LINK_LIBS}
            INCLUDE_DIRS ${_oneq_int_part_INCLUDE_DIRS})
    endif()
endfunction()

# airborne_radar：识别场景测试用 SQLite 构造特征库（helper 位于 unit/airborne_radar/）。
_oneq_add_integration_partition(airborne_radar
    LINK_LIBS SQLite::SQLite3
    INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/unit/airborne_radar"
                 "${CMAKE_CURRENT_BINARY_DIR}/generated")
if(TARGET ${PROJECT_NAME}_airborne_radar_integration_tests)
    # 示例识别基线路径（建库工具生成物，提交入库）。
    target_compile_definitions(${PROJECT_NAME}_airborne_radar_integration_tests PRIVATE
        "ONEQ_RECOGNITION_EXAMPLE_DATABASE_PATH=\"${CMAKE_SOURCE_DIR}/examples/configs/recognition/target_feature_database_v1.1.db\"")
endif()
_oneq_add_integration_partition(electro_optical_sensor)
_oneq_add_integration_partition(electronic_surveillance_radar)
_oneq_add_integration_partition(sbirs_sensor)
_oneq_add_integration_partition(cross_domain)

# remote_identification_radar：识别场景测试用 SQLite 构造特征库（helper 位于
# unit/remote_identification_radar/，DDL 生成头经 tests 层 configure_file 注入）。
_oneq_add_integration_partition(remote_identification_radar
    LINK_LIBS SQLite::SQLite3
    INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/unit/remote_identification_radar"
                 "${CMAKE_CURRENT_BINARY_DIR}/generated")

if(ONEQ_ENABLE_FLIGHT_DYNAMIC AND TARGET ${PROJECT_NAME}_cross_domain_integration_tests)
    target_compile_definitions(${PROJECT_NAME}_cross_domain_integration_tests PRIVATE
        ONEQ_TEST_FLIGHT_DYNAMIC_ENABLED=1)
    if(TARGET JSBSim::JSBSim)
        target_link_libraries(${PROJECT_NAME}_cross_domain_integration_tests PRIVATE
            JSBSim::JSBSim)
    endif()
    if(DEFINED ONEQ_JSBSIM_DATA_ROOT_DIR AND NOT ONEQ_JSBSIM_DATA_ROOT_DIR STREQUAL "")
        target_compile_definitions(${PROJECT_NAME}_cross_domain_integration_tests PRIVATE
            FD_JSBSIM_ROOT_DIR="${ONEQ_JSBSIM_DATA_ROOT_DIR}")
    else()
        target_compile_definitions(${PROJECT_NAME}_cross_domain_integration_tests PRIVATE
            FD_JSBSIM_ROOT_DIR="${CMAKE_SOURCE_DIR}/third_party/jsbsim")
    endif()
endif()
