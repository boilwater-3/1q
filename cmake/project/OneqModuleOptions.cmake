# 1q module-specific build options.

option(ONEQ_ENABLE_FLIGHT_DYNAMIC "Build the flight_dynamic (maneuver) module" OFF)
if(ONEQ_ENABLE_FLIGHT_DYNAMIC)
    message(STATUS "flight_dynamic module: ENABLED")
else()
    message(STATUS "flight_dynamic module: disabled (set -DONEQ_ENABLE_FLIGHT_DYNAMIC=ON to enable)")
endif()
