# MSVC 编译器配置 (Visual Studio, ClangCL)
# 本文件包含所有 MSVC 特定的编译和链接选项

if(NOT MSVC)
    message(FATAL_ERROR "This file should only be included when using MSVC compiler")
endif()

message(STATUS "Configuring for MSVC compiler")
message(STATUS "  └─ Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")

# 栈大小设置（通过链接器标志）
if(STACK_SIZE_OPTION STREQUAL "DEFAULT")
    message(STATUS "  └─ Stack size: Using system default (1MB)")
elseif(STACK_SIZE_OPTION STREQUAL "RECOMMENDED")
    add_link_options(/STACK:2097152)    # 2MB
    message(STATUS "  └─ Stack size: 2MB (Recommended)")
elseif(STACK_SIZE_OPTION STREQUAL "LARGE_PROJECT")
    add_link_options(/STACK:4194304)    # 4MB
    message(STATUS "  └─ Stack size: 4MB (Large Project)")
elseif(STACK_SIZE_OPTION STREQUAL "EXTREME_RECURSION")
    add_link_options(/STACK:8388608)    # 8MB
    message(STATUS "  └─ Stack size: 8MB (Extreme Recursion)")
endif()

# 通用编译选项
add_compile_options(
    /Zc:referenceBinding    # 严格的引用绑定规则
    /Zc:inline              # 移除未引用的函数/数据
    /utf-8                  # UTF-8 源文件和执行字符集
    /MP                     # 多处理器并行编译
    /permissive-            # 标准一致性模式（推荐）
)

# 警告配置
if(ENABLE_WARNINGS)
    add_compile_options(
        /W4                     # 最高警告级别
        /WX-                    # 警告不视为错误（可改为 /WX 严格模式）
        /w14242                 # 类型转换可能丢失数据
        /w14254                 # 位域类型不同
        /w14263                 # 成员函数不覆盖任何基类虚函数
        /w14265                 # 类有虚函数但析构函数非虚
        /w14287                 # 无符号/负常量不匹配
        /we4289                 # 使用非标准扩展（循环变量）
        /w14296                 # 表达式始终为真/假
        /w14311                 # 指针截断
        /w14545                 # 逗号前的表达式为无效函数
        /w14546                 # 逗号前缺少参数列表
        /w14547                 # 逗号前的运算符无效
        /w14549                 # 逗号前的运算符无效
        /w14555                 # 表达式无效
        /w14619                 # 编译指示警告编号无效
        /w14640                 # 线程安全静态成员初始化
        /w14826                 # 从窄类型转换为宽类型（性能）
        /w14905                 # 宽字符串字面值强制转换
        /w14906                 # 字符串字面值强制转换
        /w14928                 # 非法的复制初始化
    )
    message(STATUS "  └─ Warnings: Enhanced (/W4 + additional checks)")
else()
    add_compile_options(/W3)
    message(STATUS "  └─ Warnings: Standard (/W3)")
endif()

# Debug 模式配置
add_compile_options($<$<CONFIG:Debug>:/Zi>)        # 完整调试信息
add_compile_options($<$<CONFIG:Debug>:/Od>)        # 禁用优化
add_compile_options($<$<CONFIG:Debug>:/RTC1>)      # 运行时错误检查
add_compile_options($<$<CONFIG:Debug>:/JMC>)       # Just My Code调试
add_link_options($<$<CONFIG:Debug>:/DEBUG:FULL>)   # 完整调试信息
add_link_options($<$<CONFIG:Debug>:/INCREMENTAL>)  # 增量链接

# Release 模式配置（最大性能）
add_compile_options($<$<CONFIG:Release>:/O2>)      # 最大速度优化
add_compile_options($<$<CONFIG:Release>:/Ot>)      # 倾向速度而非大小
add_compile_options($<$<CONFIG:Release>:/Ob2>)     # 内联展开
add_compile_options($<$<CONFIG:Release>:/Oi>)      # 内置函数
add_compile_options($<$<CONFIG:Release>:/GL>)      # 全程序优化
add_compile_options($<$<CONFIG:Release>:/Gy>)      # 函数级链接
add_compile_options($<$<CONFIG:Release>:/GS->)     # 禁用缓冲区安全检查（性能）
add_link_options($<$<CONFIG:Release>:/LTCG>)       # 链接时代码生成
add_link_options($<$<CONFIG:Release>:/OPT:REF>)    # 移除未引用函数
add_link_options($<$<CONFIG:Release>:/OPT:ICF>)    # 合并相同函数
add_link_options($<$<CONFIG:Release>:/INCREMENTAL:NO>)  # 禁用增量链接

# RelWithDebInfo 模式配置（性能 + 调试）
add_compile_options($<$<CONFIG:RelWithDebInfo>:/O2>)   # 速度优化
add_compile_options($<$<CONFIG:RelWithDebInfo>:/Ob2>)  # 内联展开
add_compile_options($<$<CONFIG:RelWithDebInfo>:/Oi>)   # 内置函数
add_compile_options($<$<CONFIG:RelWithDebInfo>:/Zi>)   # 调试信息
add_link_options($<$<CONFIG:RelWithDebInfo>:/DEBUG:FULL>)
add_link_options($<$<CONFIG:RelWithDebInfo>:/OPT:REF>)
add_link_options($<$<CONFIG:RelWithDebInfo>:/OPT:ICF>)
add_link_options($<$<CONFIG:RelWithDebInfo>:/INCREMENTAL:NO>)

# MinSizeRel 模式配置（最小体积）
add_compile_options($<$<CONFIG:MinSizeRel>:/O1>)       # 最小体积优化
add_compile_options($<$<CONFIG:MinSizeRel>:/Os>)       # 倾向体积
add_link_options($<$<CONFIG:MinSizeRel>:/OPT:REF>)
add_link_options($<$<CONFIG:MinSizeRel>:/OPT:ICF>)
