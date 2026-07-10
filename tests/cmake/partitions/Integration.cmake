# Integration-type test partitions: one executable per owner domain.

function(_oneq_add_integration_partition domain)
    file(GLOB _sources CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/integration/${domain}/*_test.cpp")
    if(_sources)
        oneq_add_test_partition(
            TYPE integration
            DOMAIN ${domain}
            SOURCES ${_sources}
            TIMEOUT 120)
    endif()
endfunction()

_oneq_add_integration_partition(airborne_radar)
_oneq_add_integration_partition(electro_optical_sensor)
_oneq_add_integration_partition(electronic_surveillance_radar)
_oneq_add_integration_partition(sbirs_sensor)
_oneq_add_integration_partition(cross_domain)
