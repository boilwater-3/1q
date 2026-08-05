# fusion 模块源文件清单。

oneq_add_component(fusion_core SOURCES
    FusionEngine.cpp)
target_link_libraries(fusion_core PRIVATE nanoflann::nanoflann)
