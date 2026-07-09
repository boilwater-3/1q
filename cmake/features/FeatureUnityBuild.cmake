# Unity Build 配置
# Unity Build 将多个源文件合并编译以加速构建

if(ENABLE_UNITY_BUILD)
    set(CMAKE_UNITY_BUILD ON)
    
    # Unity Build 批次大小：每批合并编译的源文件数量
    # 较小值：减少单个编译单元体积，但合并效果降低
    # 较大值：更激进的合并，但可能导致编译器内存占用过高
    if(NOT DEFINED CMAKE_UNITY_BUILD_BATCH_SIZE)
        set(CMAKE_UNITY_BUILD_BATCH_SIZE 16 CACHE STRING 
            "Number of source files to combine in Unity Build")
    endif()
    
    message(STATUS "Unity Build: Enabled")
    message(STATUS "  └─ Batch size: ${CMAKE_UNITY_BUILD_BATCH_SIZE} files per unity")
    
    # MSVC 需要 /bigobj 处理大型对象文件
    if(MSVC)
        add_compile_options(/bigobj)
        message(STATUS "  └─ Added /bigobj flag for MSVC")
    endif()
else()
    message(STATUS "Unity Build: Disabled")
endif()
