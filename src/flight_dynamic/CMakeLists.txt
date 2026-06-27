set(FD_ENGINE_SOURCES
    ${CMAKE_SOURCE_DIR}/src/flight_dynamic/adapter/JsbsimAdapter.cpp
    ${CMAKE_SOURCE_DIR}/src/flight_dynamic/FlightManager.cpp
)

set(FD_CORE_SOURCES
    ${CMAKE_SOURCE_DIR}/src/flight_dynamic/model/VehicleStateMapper.cpp
    ${CMAKE_SOURCE_DIR}/src/flight_dynamic/AircraftPerformanceDerivation.cpp
    ${CMAKE_SOURCE_DIR}/src/flight_dynamic/autopilot/Autopilot.cpp
    ${CMAKE_SOURCE_DIR}/src/flight_dynamic/propulsion/EngineManager.cpp
    ${CMAKE_SOURCE_DIR}/src/flight_dynamic/guidance/WaypointManager.cpp
    ${CMAKE_SOURCE_DIR}/src/flight_dynamic/guidance/Maneuver.cpp
)

if(ENABLE_INSTALL)
    set(PUBLIC_HEADERS_FD_ENTRY
        ${CMAKE_SOURCE_DIR}/include/1q/flight_dynamic/FlightManager.h
    )
    set(PUBLIC_HEADERS_FD_CONFIG
        ${CMAKE_SOURCE_DIR}/include/1q/flight_dynamic/config/FlightDynamicConfig.h
    )
    set(PUBLIC_HEADERS_FD_MODEL
        ${CMAKE_SOURCE_DIR}/include/1q/flight_dynamic/model/VehicleState.h
    )
    set(PUBLIC_HEADERS_FD_AUTOPILOT
        ${CMAKE_SOURCE_DIR}/include/1q/flight_dynamic/autopilot/Autopilot.h
    )
    set(PUBLIC_HEADERS_FD_GUIDANCE
        ${CMAKE_SOURCE_DIR}/include/1q/flight_dynamic/guidance/Maneuver.h
        ${CMAKE_SOURCE_DIR}/include/1q/flight_dynamic/guidance/Waypoint.h
        ${CMAKE_SOURCE_DIR}/include/1q/flight_dynamic/guidance/WaypointManager.h
    )

    install(FILES ${PUBLIC_HEADERS_FD_ENTRY}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/flight_dynamic
    )
    install(FILES ${PUBLIC_HEADERS_FD_CONFIG}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/flight_dynamic/config
    )
    install(FILES ${PUBLIC_HEADERS_FD_MODEL}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/flight_dynamic/model
    )
    install(FILES ${PUBLIC_HEADERS_FD_AUTOPILOT}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/flight_dynamic/autopilot
    )
    install(FILES ${PUBLIC_HEADERS_FD_GUIDANCE}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/flight_dynamic/guidance
    )
endif()
