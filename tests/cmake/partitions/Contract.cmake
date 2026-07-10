# Compiled contract-test partitions. Script guards remain registered separately.

function(_oneq_add_contract_partition domain)
    file(GLOB _sources CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/contract/${domain}/*_test.cpp")
    if(_sources)
        set(_labels)
        if(domain STREQUAL "public_api")
            list(APPEND _labels ci_required)
        endif()
        oneq_add_test_partition(
            TYPE contract
            DOMAIN ${domain}
            SOURCES ${_sources}
            TIMEOUT 60
            LABELS ${_labels})
    endif()
endfunction()

_oneq_add_contract_partition(public_api)
_oneq_add_contract_partition(airborne_radar)
_oneq_add_contract_partition(electro_optical_sensor)
_oneq_add_contract_partition(electronic_surveillance_radar)
_oneq_add_contract_partition(sar)
_oneq_add_contract_partition(sbirs_sensor)
