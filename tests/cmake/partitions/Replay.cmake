# Replay-type test partitions: one executable per (type=replay, domain).
# Each partition retains the legacy replay_fast label until Phase 4 migrates CI
# selection to the common execution-policy labels.

set(_oneq_replay_depends)
if(TARGET oneq_flatbuffers_headers)
    list(APPEND _oneq_replay_depends oneq_flatbuffers_headers)
endif()

function(_oneq_add_replay_partition domain)
    file(GLOB _sources CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/replay/${domain}/*_test.cpp")
    if(_sources)
        oneq_add_test_partition(
            TYPE replay
            DOMAIN ${domain}
            SOURCES ${_sources}
            TIMEOUT 90
            LABELS fast replay_fast ci_required
            DEPENDS ${_oneq_replay_depends})
    endif()
endfunction()

_oneq_add_replay_partition(common)
_oneq_add_replay_partition(airborne_radar)
_oneq_add_replay_partition(electro_optical_sensor)
_oneq_add_replay_partition(electronic_surveillance_radar)
_oneq_add_replay_partition(sar)
_oneq_add_replay_partition(sbirs_sensor)

unset(_oneq_replay_depends)
