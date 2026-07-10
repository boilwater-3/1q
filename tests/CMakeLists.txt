# Test lifecycle entry point. Detailed registrations are grouped by ownership
# under tests/cmake/ so this file remains a stable orchestration surface.
#
# Migration state (see docs/review/test_architecture_replan.md):
#   - Phase 0: registry/layout guard active; provisional per-module split under
#     tests/cmake/ is marked as transition and must not be extended.
#   - The SarTests.cmake / FlightDynamicTests.cmake hand-written filters are
#     frozen here; new test partitions should go through oneq_register_test_sources()
#     rather than adding suite filters to those files.

if(NOT ENABLE_TESTING)
    return()
endif()

include(${CMAKE_CURRENT_LIST_DIR}/cmake/TestSupport.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/TestRegistry.cmake)
if(NOT TEST_SOURCES)
    message(STATUS "No test sources defined in ${CMAKE_CURRENT_SOURCE_DIR}")
    message(STATUS "  Add .cpp files to unit/integration/contract to enable testing")
    return()
endif()

include(${CMAKE_CURRENT_LIST_DIR}/cmake/TestTargets.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/FlightDynamicTests.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/SarTests.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/ContractGuards.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/CompatibilityChecks.cmake)

# Phase 0 layout guard: reject flat _test.cpp at tests/ root and unknown type dirs.
oneq_add_script_guard(test_layout_guard check_test_layout.cmake)
set_tests_properties(test_layout_guard PROPERTIES LABELS "contract")

# Finalize registry: orphan/duplicate checks + partition statistics output.
oneq_finalize_test_registry()
