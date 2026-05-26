set(FD_ENGINE_SOURCES
    ${CMAKE_SOURCE_DIR}/src/flight_dynamic/adapter/JsbsimAdapter.cpp
    ${CMAKE_SOURCE_DIR}/src/flight_dynamic/FlightManager.cpp
)

set(FD_CORE_SOURCES
    ${CMAKE_SOURCE_DIR}/src/flight_dynamic/model/VehicleStateMapper.cpp
    ${CMAKE_SOURCE_DIR}/src/flight_dynamic/autopilot/Autopilot.cpp
    ${CMAKE_SOURCE_DIR}/src/flight_dynamic/guidance/WaypointManager.cpp
    ${CMAKE_SOURCE_DIR}/src/flight_dynamic/guidance/Maneuver.cpp
)
