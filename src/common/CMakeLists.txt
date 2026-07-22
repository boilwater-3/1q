# common 模块源文件与安装清单

set(ONEQ_COMMON_SOURCES
    atmosphere/AtmospherePhysics.cpp
    atmosphere/StandardAtmosphere.cpp
    electromagnetics/RfLinkBudget.cpp
    environment/PropagationPhysicsImpl.cpp
    geometry/GeometryTransform.cpp
    coordinate/PositionTransform.cpp
    coordinate/VelocityTransform.cpp
    coordinate/AttitudeTransform.cpp
    numerics/SpectralNumerics.cpp
    rcs/RcsPhysics.cpp
    replay/ReplayTrace.cpp
    trace/TraceSink.cpp
    timing/TimingRegimeModel.cpp
)

oneq_add_component(oneq_common SOURCES ${ONEQ_COMMON_SOURCES})
target_link_libraries(oneq_common PRIVATE
    ${ONEQ_LINK_DEPENDENCIES})
