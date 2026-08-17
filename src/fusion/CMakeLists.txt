# fusion 模块源文件清单。

oneq_add_component(fusion_core SOURCES
    FusionEngine.cpp
    SensorAdapters.cpp)
target_link_libraries(fusion_core PRIVATE nanoflann::nanoflann Eigen3::Eigen)
