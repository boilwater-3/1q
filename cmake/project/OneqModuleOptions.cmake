# 1q module-specific build options.

option(ONEQ_ENABLE_HDF5_OUTPUT "Enable SAR HDF5 image output (requires HighFive)" OFF)

option(ONEQ_ENABLE_FLIGHT_DYNAMIC "Build the flight_dynamic (maneuver) module" OFF)
if(ONEQ_ENABLE_FLIGHT_DYNAMIC)
    message(STATUS "flight_dynamic module: ENABLED")
else()
    message(STATUS "flight_dynamic module: disabled (set -DONEQ_ENABLE_FLIGHT_DYNAMIC=ON to enable)")
endif()
