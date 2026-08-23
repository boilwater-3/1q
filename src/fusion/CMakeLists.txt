# fusion 模块源文件清单。

oneq_add_component(fusion_core SOURCES
    FusionEngine.cpp
    FusionAcceptanceRecords.cpp
    SensorAdapters.cpp)
target_link_libraries(fusion_core PRIVATE nanoflann::nanoflann Eigen3::Eigen)
if(ONEQ_ENABLE_FUSION_ACCEPTANCE_LOG)
    target_compile_definitions(fusion_core PRIVATE ONEQ_ENABLE_FUSION_ACCEPTANCE_LOG=1)
endif()
