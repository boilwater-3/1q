# Test source registry: single source of truth for which _test.cpp belongs to
# which compilation partition. Introduced in Phase 0 to freeze the baseline and
# guard the migration through Phase 1-6.
#
# Responsibilities:
#   1. Record every registered _test.cpp into ONEQ_TEST_REGISTRY.
#   2. Detect orphan _test.cpp files (exist on disk but never registered).
#   3. Detect duplicate registration (same source compiled into two partitions).
#   4. Allowlist known Phase 0 overlap (replay sources compiled into both
#      1q_unit_tests and 1q_replay_fast_tests) until Phase 3 eliminates it.
#   5. Finalize prints partition/source statistics for review-level drift watch.
#
# Contract: oneq_register_test_sources(<partition_key> <source...>) appends to
# the registry; oneq_finalize_test_registry() runs at the end of the test scope.

# Global ordered list of "partition_key|abs_source_path" entries.
define_property(GLOBAL PROPERTY ONEQ_TEST_REGISTRY
    BRIEF_DOCS "Ordered registry of partition_key|source_path test registrations."
    FULL_DOCS "Populated by oneq_register_test_sources(); consumed by oneq_finalize_test_registry().")

# Partition -> sorted source list, for statistics and target ownership checks.
define_property(GLOBAL PROPERTY ONEQ_TEST_PARTITION_MAP
    BRIEF_DOCS "Map partition_key -> list of absolute source paths."
    FULL_DOCS "Populated alongside ONEQ_TEST_REGISTRY for partition statistics.")

# Known overlap allowlist: sources that during Phase 0-2 are intentionally
# compiled into more than one target. Phase 3 must remove these entries as it
# eliminates the duplicate compilation. Each entry is an absolute source path.
set(ONEQ_TEST_OVERLAP_ALLOWLIST
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/ar_replay_codec_roundtrip_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/eos_replay_codec_roundtrip_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/esr_replay_codec_roundtrip_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/sar_replay_codec_roundtrip_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/sbirs_replay_codec_roundtrip_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/replay_trace_writer_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/ar_trace_session_adapter_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/eos_replay_session_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/esr_replay_session_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/sar_replay_session_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/replay_trace_compression_test.cpp")

# Record a batch of sources under a partition key (e.g. "unit", "replay_fast").
# Usage: oneq_register_test_sources(<partition_key> source1 source2 ...)
function(oneq_register_test_sources partition_key)
    if(NOT ARGN)
        return()
    endif()
    foreach(_src IN LISTS ARGN)
        get_filename_component(_abs "${_src}" ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
        set_property(GLOBAL APPEND PROPERTY ONEQ_TEST_REGISTRY "${partition_key}|${_abs}")
        set_property(GLOBAL APPEND PROPERTY ONEQ_TEST_PARTITION_MAP "${partition_key}=${_abs}")
        set(_oneq_prop_oneq_test_partition_map_${partition_key}
            "${_oneq_prop_oneq_test_partition_map_${partition_key}};${_abs}"
            CACHE INTERNAL "oneq partition sources")
    endforeach()
endfunction()

# Finalize: run orphan/duplicate checks and emit partition statistics.
# Call this once at the end of the tests/CMakeLists.txt orchestration.
function(oneq_finalize_test_registry)
    get_property(_entries GLOBAL PROPERTY ONEQ_TEST_REGISTRY)
    # 1. Build source -> list(partitions) map.
    set(_registered_sources "")
    foreach(_entry IN LISTS _entries)
        string(FIND "${_entry}" "|" _sep)
        math(EXPR _key_end "${_sep}")
        string(SUBSTRING "${_entry}" 0 "${_key_end}" _key)
        math(EXPR _path_start "${_sep} + 1")
        string(SUBSTRING "${_entry}" "${_path_start}" -1 _path)
        list(APPEND _registered_sources "${_path}")
        set(_own "${_oneq_partition_for_${_path}}")
        if(_own STREQUAL "")
            set(_oneq_partition_for_${_path} "${_key}")
        else()
            set(_oneq_partition_for_${_path} "${_own};${_key}")
        endif()
    endforeach()

    # 2. Duplicate detection (excluding allowlist).
    set(_duplicates "")
    foreach(_src IN LISTS _registered_sources)
        set(_parts "${_oneq_partition_for_${_src}}")
        list(REMOVE_DUPLICATES _parts)
        list(LENGTH _parts _n)
        if(_n GREATER 1)
            list(FIND ONEQ_TEST_OVERLAP_ALLOWLIST "${_src}" _allowed)
            if(_allowed EQUAL -1)
                list(APPEND _duplicates "${_src} [${_parts}]")
            endif()
        endif()
    endforeach()
    if(_duplicates)
        list(REMOVE_DUPLICATES _duplicates)
        message(FATAL_ERROR
            "ONEQ test registry: duplicate partition ownership detected (not on allowlist):\n"
            "  ${_duplicates}\n"
            "Each _test.cpp must belong to exactly one partition, or be listed in "
            "ONEQ_TEST_OVERLAP_ALLOWLIST during the transition.")
    endif()

    # 3. Orphan detection: scan known type roots for _test.cpp not registered.
    set(_type_roots unit integration contract performance)
    set(_orphans "")
    foreach(_root IN LISTS _type_roots)
        file(GLOB _disk_sources CONFIGURE_DEPENDS
            "${CMAKE_CURRENT_SOURCE_DIR}/${_root}/*_test.cpp")
        foreach(_src IN LISTS _disk_sources)
            list(FIND _registered_sources "${_src}" _found)
            if(_found EQUAL -1)
                list(APPEND _orphans "${_src}")
            endif()
        endforeach()
    endforeach()
    if(_orphans)
        list(REMOVE_DUPLICATES _orphans)
        message(FATAL_ERROR
            "ONEQ test registry: orphan _test.cpp not registered in any partition:\n"
            "  ${_orphans}\n"
            "Every _test.cpp under tests/{unit,integration,contract,performance} "
            "must be registered via oneq_register_test_sources().")
    endif()

    # 4. Statistics output for review drift watch (Phase 0 baseline freeze).
    list(LENGTH _registered_sources _total)
    message(STATUS "ONEQ test registry: ${_total} registered _test.cpp sources")
    foreach(_entry IN LISTS _entries)
        string(FIND "${_entry}" "|" _sep)
        string(SUBSTRING "${_entry}" 0 "${_sep}" _key)
    endforeach()
    # Per-partition counts.
    set(_partition_keys "")
    foreach(_entry IN LISTS _entries)
        string(FIND "${_entry}" "|" _sep)
        string(SUBSTRING "${_entry}" 0 "${_sep}" _key)
        list(APPEND _partition_keys "${_key}")
    endforeach()
    if(_partition_keys)
        list(REMOVE_DUPLICATES _partition_keys)
        foreach(_key IN LISTS _partition_keys)
            set(_count 0)
            foreach(_entry IN LISTS _entries)
                string(FIND "${_entry}" "|" _sep)
                string(SUBSTRING "${_entry}" 0 "${_sep}" _ekey)
                if(_ekey STREQUAL _key)
                    math(EXPR _count "${_count} + 1")
                endif()
            endforeach()
            message(STATUS "  partition '${_key}': ${_count} sources")
        endforeach()
    endif()
endfunction()
