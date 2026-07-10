# Test layout guard (Phase 0): enforce the first layer of the target layout.
#
# Phase 0 scope (per test_architecture_replan.md):
#   - Reject any _test.cpp placed directly under tests/ root.
#   - Do NOT yet reject the current flat structure under tests/unit (that is
#     the Phase 1 migration target).
#   - Warn (not fail) on non-empty tests/ root _test.cpp to keep the baseline
#     frozen while Phase 1 directories are being created.
#
# This guard is tightened in Phase 1/6 to reject flat sources, unknown domains,
# duplicate ownership, and hand-written domain filters.

cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED SOURCE_DIR)
    message(FATAL_ERROR "SOURCE_DIR must be defined (-DSOURCE_DIR=...).")
endif()

set(_test_root "${SOURCE_DIR}/tests")

# Phase 0 hard rule: no _test.cpp directly under tests/ root.
file(GLOB _root_tests "${_test_root}/*_test.cpp")
if(_root_tests)
    list(JOIN _root_tests "\n  " _root_list)
    message(FATAL_ERROR
        "test_layout_guard: _test.cpp must not live directly under tests/. Found:\n  ${_root_list}\n"
        "Place them under a type directory: tests/{unit,integration,contract,performance,replay,compatibility,consumer}/<domain>/")
endif()

# Informational: report current flat-structure count under tests/unit for
# migration tracking. This is a status line, not a failure, until Phase 6.
file(GLOB _flat_unit_tests "${_test_root}/unit/*_test.cpp")
list(LENGTH _flat_unit_tests _flat_count)
message(STATUS "test_layout_guard: ${_flat_count} flat _test.cpp under tests/unit/ (Phase 1 migration pending)")

# Ensure type subdirectories that hold tests match the allowed vocabulary.
# Phase 0 only validates the four currently-active type roots; infrastructure
# directories (cmake/, support/) are explicitly allowed and are not test types.
set(_allowed_type_roots unit integration contract performance consumer)
set(_allowed_infra_dirs cmake support)
file(GLOB _entries RELATIVE "${_test_root}" "${_test_root}/*")
foreach(_e IN LISTS _entries)
    if(IS_DIRECTORY "${_test_root}/${_e}")
        list(FIND _allowed_type_roots "${_e}" _type_idx)
        list(FIND _allowed_infra_dirs "${_e}" _infra_idx)
        if(_type_idx EQUAL -1 AND _infra_idx EQUAL -1)
            message(FATAL_ERROR
                "test_layout_guard: unknown directory tests/${_e}/. "
                "Allowed type roots: ${_allowed_type_roots}; "
                "infrastructure: ${_allowed_infra_dirs}. "
                "(replay/compatibility are Phase 1 targets.)")
        endif()
    endif()
endforeach()

message(STATUS "test_layout_guard: Phase 0 layout check passed.")
