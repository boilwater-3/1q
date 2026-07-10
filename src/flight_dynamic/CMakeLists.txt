set(FD_ENGINE_SOURCES
    adapter/JsbsimAdapter.cpp
    FlightManager.cpp
)

set(FD_CORE_SOURCES
    model/VehicleStateMapper.cpp
    AircraftPerformanceDerivation.cpp
    autopilot/Autopilot.cpp
    propulsion/EngineManager.cpp
    guidance/WaypointManager.cpp
    guidance/Maneuver.cpp
)

oneq_add_component(fd_engine SOURCES ${FD_ENGINE_SOURCES})
oneq_add_component(fd_core SOURCES ${FD_CORE_SOURCES})
target_link_libraries(fd_engine PRIVATE JSBSim::JSBSim)
target_link_libraries(fd_core PRIVATE JSBSim::JSBSim)
