# navigation 模块源文件清单。

oneq_add_component(navigation_core SOURCES
    AreaCoveragePlanner.cpp)
target_link_libraries(navigation_core PRIVATE Eigen3::Eigen)
