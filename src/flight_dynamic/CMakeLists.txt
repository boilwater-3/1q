# flight_dynamic 模块源文件与安装清单

# -- engine: JSBSim 适配器、运动学积分、机动控制（依赖 JSBSim，算法密集） ----------
set(FD_ENGINE_SOURCES
    flight_dynamic/adapter/JsbsimAdapter.cpp
    flight_dynamic/model/VehicleStateMapper.cpp
    flight_dynamic/maneuver/ManeuverController.cpp
)

# -- core: 会话、工厂（粘合层，依赖轻，仅 JSBSim 通过 Impl 间接引用） ----------
set(FD_CORE_SOURCES
    flight_dynamic/session/FlightDynamicSession.cpp
    flight_dynamic/session/FlightDynamicSessionFactory.cpp
)

if(ENABLE_INSTALL)
    set(PUBLIC_HEADERS_FD_CONFIG
        ${CMAKE_SOURCE_DIR}/include/1q/flight_dynamic/config/AircraftDefinition.h
        ${CMAKE_SOURCE_DIR}/include/1q/flight_dynamic/config/FlightDynamicConfig.h
    )
    set(PUBLIC_HEADERS_FD_MODEL
        ${CMAKE_SOURCE_DIR}/include/1q/flight_dynamic/model/VehicleState.h
        ${CMAKE_SOURCE_DIR}/include/1q/flight_dynamic/model/FlightDynamicInput.h
        ${CMAKE_SOURCE_DIR}/include/1q/flight_dynamic/model/FlightDynamicOutput.h
    )
    set(PUBLIC_HEADERS_FD_MANEUVER
        ${CMAKE_SOURCE_DIR}/include/1q/flight_dynamic/maneuver/ManeuverTypes.h
        ${CMAKE_SOURCE_DIR}/include/1q/flight_dynamic/maneuver/ManeuverController.h
    )
    set(PUBLIC_HEADERS_FD_SESSION
        ${CMAKE_SOURCE_DIR}/include/1q/flight_dynamic/session/FlightDynamicSession.h
        ${CMAKE_SOURCE_DIR}/include/1q/flight_dynamic/session/FlightDynamicSessionFactory.h
    )
    set(PUBLIC_HEADERS_FD_ENTRY
        ${CMAKE_SOURCE_DIR}/include/1q/flight_dynamic/flight_dynamic.hpp
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
    install(FILES ${PUBLIC_HEADERS_FD_MANEUVER}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/flight_dynamic/maneuver
    )
    install(FILES ${PUBLIC_HEADERS_FD_SESSION}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/flight_dynamic/session
    )
endif()
