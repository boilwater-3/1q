# GCC/Clang 编译器配置
# 本文件包含所有 GCC 和 Clang 特定的编译和链接选项

if(MSVC)
    message(FATAL_ERROR "This file should only be included when using GCC/Clang compiler")
endif()

message(STATUS "Configuring for GCC/Clang compiler")
message(STATUS "  └─ Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")

# 栈大小设置（通过链接器标志）
# 注意：macOS 不支持标准的栈大小设置，仅在 Linux 上生效
if(NOT APPLE)
    if(STACK_SIZE_OPTION STREQUAL "DEFAULT")
        message(STATUS "  └─ Stack size: Using system default")
    elseif(STACK_SIZE_OPTION STREQUAL "RECOMMENDED")
        add_link_options(-Wl,-z,stack-size=2097152)    # 2MB
        message(STATUS "  └─ Stack size: 2MB (Recommended)")
    elseif(STACK_SIZE_OPTION STREQUAL "LARGE_PROJECT")
        add_link_options(-Wl,-z,stack-size=4194304)    # 4MB
        message(STATUS "  └─ Stack size: 4MB (Large Project)")
    elseif(STACK_SIZE_OPTION STREQUAL "EXTREME_RECURSION")
        add_link_options(-Wl,-z,stack-size=8388608)    # 8MB
        message(STATUS "  └─ Stack size: 8MB (Extreme Recursion)")
    endif()
else()
    message(STATUS "  └─ Stack size: macOS uses system default (adjustable via ulimit)")
endif()

# 通用编译选项
set(CMAKE_POSITION_INDEPENDENT_CODE ON)  # 位置无关代码
add_compile_options(
    -fvisibility=hidden     # 默认隐藏符号
)

# 警告配置
if(ENABLE_WARNINGS)
    add_compile_options(
        -Wall                   # 所有常见警告
        -Wextra                 # 额外警告
        -Wpedantic              # 严格标准一致性
        -Wshadow                # 变量遮蔽
        -Wnon-virtual-dtor      # 非虚析构函数
        -Wold-style-cast        # C 风格类型转换
        -Wcast-align            # 指针对齐转换
        -Wunused                # 未使用的变量/函数
        -Woverloaded-virtual    # 虚函数遮蔽
        -Wconversion            # 隐式类型转换
        -Wsign-conversion       # 符号转换
        -Wdouble-promotion      # float提升为double
        -Wformat=2              # printf格式字符串检查
        -Wimplicit-fallthrough  # switch缺少break
    )
    
    # GCC 特有警告
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        add_compile_options(
            -Wmisleading-indentation    # 误导性缩进
            -Wduplicated-cond           # 重复条件
            -Wduplicated-branches       # 重复分支
            -Wlogical-op                # 逻辑运算符误用
            -Wnull-dereference          # 空指针解引用
            -Wuseless-cast              # 无用的类型转换
        )
    endif()
    
    message(STATUS "  └─ Warnings: Enhanced (-Wall -Wextra + additional)")
else()
    add_compile_options(-Wall)
    message(STATUS "  └─ Warnings: Standard (-Wall)")
endif()

# Debug 模式配置
add_compile_options($<$<CONFIG:Debug>:-g3>)        # 最详细调试信息
add_compile_options($<$<CONFIG:Debug>:-O0>)        # 禁用优化
add_compile_options($<$<CONFIG:Debug>:-fno-omit-frame-pointer>)  # 保留帧指针
add_compile_options($<$<CONFIG:Debug>:-fno-inline>)              # 禁用内联

# Release 模式配置
add_compile_options($<$<CONFIG:Release>:-O3>)      # 最大优化
add_compile_options($<$<CONFIG:Release>:-DNDEBUG>) # 禁用断言
add_compile_options($<$<CONFIG:Release>:-flto>)    # 链接时优化（LTO）
add_compile_options($<$<CONFIG:Release>:-fomit-frame-pointer>)  # 省略帧指针
add_link_options($<$<CONFIG:Release>:-flto>)       # LTO链接

# MasOs
if(APPLE)
    add_link_options($<$<CONFIG:Release>:-Wl,-dead_strip>)  # macOS: 移除未使用符号
else()
    add_link_options($<$<CONFIG:Release>:-Wl,--gc-sections>)  # Linux: 移除未使用节
endif()

# RelWithDebInfo 模式配置
add_compile_options($<$<CONFIG:RelWithDebInfo>:-O2>)       # 平衡优化
add_compile_options($<$<CONFIG:RelWithDebInfo>:-g>)        # 调试信息
add_compile_options($<$<CONFIG:RelWithDebInfo>:-DNDEBUG>)  # 禁用断言
add_compile_options($<$<CONFIG:RelWithDebInfo>:-fno-omit-frame-pointer>)

# MinSizeRel 模式配置
add_compile_options($<$<CONFIG:MinSizeRel>:-Os>)       # 体积优化
add_compile_options($<$<CONFIG:MinSizeRel>:-DNDEBUG>)  # 禁用断言
add_compile_options($<$<CONFIG:MinSizeRel>:-flto>)     # LTO
add_link_options($<$<CONFIG:MinSizeRel>:-flto>)

# MasOs
if(APPLE)
    add_link_options($<$<CONFIG:MinSizeRel>:-Wl,-dead_strip>)
else()
    add_link_options($<$<CONFIG:MinSizeRel>:-Wl,--gc-sections>)
endif()
