# Test lifecycle entry point. Detailed registrations are grouped by ownership
# under tests/cmake/ so this file remains a stable orchestration surface.
#
# Test registration is type × domain based. New C++ tests must be added to the
# owning partition and registered through oneq_add_test_partition().

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
include(${CMAKE_CURRENT_LIST_DIR}/cmake/FlightDynamicPartitions.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/ContractGuards.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/CompatibilityChecks.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/CoverageRunner.cmake)

# Phase 0 layout guard: reject flat _test.cpp at tests/ root and unknown type dirs.
oneq_add_script_guard(test_layout_guard check_test_layout.cmake)
set_tests_properties(test_layout_guard PROPERTIES LABELS "contract")

# Finalize registry: orphan/duplicate checks + partition statistics output.
oneq_finalize_test_registry()
