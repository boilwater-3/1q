# common 模块源文件与安装清单

set(ONEQ_COMMON_SOURCES
    common/atmosphere/AtmospherePhysics.cpp
    common/geometry/GeometryTransform.cpp
    common/coordinate/PositionTransform.cpp
    common/coordinate/VelocityTransform.cpp
    common/coordinate/AttitudeTransform.cpp
    common/numerics/SpectralNumerics.cpp
    common/rcs/RcsPhysics.cpp
    common/replay/ReplayTrace.cpp
    common/trace/TraceSink.cpp
    common/timing/TimingRegimeModel.cpp
)

if(ENABLE_INSTALL)
    set(PUBLIC_HEADERS_FOUNDATION
        ${CMAKE_SOURCE_DIR}/include/1q/foundation/pose_types.h
        ${CMAKE_SOURCE_DIR}/include/1q/foundation/scan_schedule_types.h
    )
    set(PUBLIC_HEADERS_COORDINATE
        ${CMAKE_SOURCE_DIR}/include/1q/coordinate/types.h
        ${CMAKE_SOURCE_DIR}/include/1q/coordinate/position_transform.h
        ${CMAKE_SOURCE_DIR}/include/1q/coordinate/velocity_transform.h
        ${CMAKE_SOURCE_DIR}/include/1q/coordinate/attitude_transform.h
    )
    set(PUBLIC_HEADERS_REPLAY
        ${CMAKE_SOURCE_DIR}/include/1q/replay/ReplayTrace.h
    )
    set(PUBLIC_HEADERS_TRACE
        ${CMAKE_SOURCE_DIR}/include/1q/trace/TraceSink.h
    )

    install(FILES ${PUBLIC_HEADERS_FOUNDATION}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/foundation
    )
    install(FILES ${PUBLIC_HEADERS_COORDINATE}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/coordinate
    )
    install(FILES ${PUBLIC_HEADERS_TRACE}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/trace
    )
    install(FILES ${PUBLIC_HEADERS_REPLAY}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/replay
    )
endif()
