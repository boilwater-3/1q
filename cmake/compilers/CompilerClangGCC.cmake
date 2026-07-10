# CompilerClangGCC.cmake
# 为 GCC/Clang 编译器定义 target 级编译/链接选项应用函数。
# 本文件仅定义函数，不直接调用 add_compile_options() / add_link_options()，

function(apply_clang_gcc_options)
    set(one_value_args ENABLE_WARNINGS STACK_SIZE_OPTION)
    set(multi_value_args TARGETS LINK_TARGETS)
    cmake_parse_arguments(ARG "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

    if(MSVC)
        message(FATAL_ERROR "apply_clang_gcc_options() requires a GCC/Clang compiler")
    endif()
    if(NOT ARG_TARGETS)
        message(FATAL_ERROR "apply_clang_gcc_options() requires TARGETS")
    endif()
    if(NOT DEFINED ARG_STACK_SIZE_OPTION OR ARG_STACK_SIZE_OPTION STREQUAL "")
        set(ARG_STACK_SIZE_OPTION "DEFAULT")
    endif()
    if(NOT ARG_LINK_TARGETS)
        set(ARG_LINK_TARGETS ${ARG_TARGETS})
    endif()

    message(STATUS "Configuring for GCC/Clang compiler")
    message(STATUS "  └─ Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")

    foreach(_target IN LISTS ARG_TARGETS)
        if(NOT TARGET "${_target}")
            message(FATAL_ERROR "Compiler target does not exist: ${_target}")
        endif()
        # 启用位置无关代码（PIC），供共享库与可执行文件 ASLR 使用
        set_target_properties("${_target}" PROPERTIES POSITION_INDEPENDENT_CODE ON)
        # 默认隐藏符号可见性，仅显式导出的符号对外可见（缩减符号表、强化 ABI 边界）
        target_compile_options("${_target}" PRIVATE -fvisibility=hidden)

        if(ARG_ENABLE_WARNINGS)
            target_compile_options("${_target}" PRIVATE
                -Wall                     # 启用常用警告（基线）
                -Wextra                   # 在 -Wall 之上再启用一批额外警告
                -Wpedantic                # 严格遵守 ISO C++ 标准，禁用编译器扩展
                -Wshadow                  # 内层作用域变量遮蔽外层同名变量
                -Wnon-virtual-dtor        # 多态基类析构函数未声明 virtual（易资源泄漏）
                -Wold-style-cast          # 使用 C 风格转换 (T)x，建议改用 *_cast
                -Wcast-align              # 指针转换可能改变对齐要求，运行期崩坏风险
                -Wunused                  # 未使用的变量 / 函数 / 标签 / typedef 等
                -Woverloaded-virtual      # 派生类同名函数隐藏了基类虚函数重载
                -Wconversion              # 可能改变数值的隐式类型转换
                -Wsign-conversion         # 有符号 / 无符号之间的隐式转换
                -Wdouble-promotion        # float 隐式提升为 double（性能损耗）
                -Wformat=2                # printf/scanf 格式串与实参类型严格校验
                -Wimplicit-fallthrough)   # switch case 未 break 的隐式直通
            if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
                target_compile_options("${_target}" PRIVATE
                    -Wmisleading-indentation  # if/else 体缩进误导（缺大括号易误读）
                    -Wduplicated-cond         # if/else if 链中出现重复条件
                    -Wduplicated-branches     # if/else 两个分支代码完全相同
                    -Wlogical-op              # 逻辑运算符疑似误用（如 && 写成 &）
                    -Wnull-dereference        # 解引用经分析判定为空的指针
                    -Wuseless-cast)           # 冗余转换（源/目标类型实际相同）
            endif()
        else()
            target_compile_options("${_target}" PRIVATE -Wall)  # 仅启用常用警告基线
        endif()

        target_compile_options("${_target}" PRIVATE
            $<$<CONFIG:Debug>:-g3>                          # 生成最详细调试信息（含宏定义）
            $<$<CONFIG:Debug>:-O0>                          # 关闭所有优化，便于单步调试
            $<$<CONFIG:Debug>:-fno-omit-frame-pointer>      # 保留帧指针，便于栈回溯/profiler
            $<$<CONFIG:Debug>:-fno-inline>                  # 禁止内联，保持调用栈可读
            $<$<CONFIG:Release>:-O3>                        # 启用激进优化（含向量化等）
            $<$<CONFIG:Release>:-DNDEBUG>                   # 定义 NDEBUG，关闭 assert 与调试断言
            $<$<CONFIG:Release>:-flto>                      # 链接期优化（跨翻译单元内联/消除）
            $<$<CONFIG:Release>:-fomit-frame-pointer>       # 省略帧指针，多出一个可用寄存器
            $<$<CONFIG:RelWithDebInfo>:-O2>                 # 标准优化，兼顾性能与可调试性
            $<$<CONFIG:RelWithDebInfo>:-g>                  # 生成调试信息（不含宏，默认级别）
            $<$<CONFIG:RelWithDebInfo>:-DNDEBUG>            # 关闭 assert 与调试断言
            $<$<CONFIG:RelWithDebInfo>:-fno-omit-frame-pointer>  # 保留帧指针，便于采样/排查
            $<$<CONFIG:MinSizeRel>:-Os>                     # 以代码体积为目标优化
            $<$<CONFIG:MinSizeRel>:-DNDEBUG>                # 关闭 assert 与调试断言
            $<$<CONFIG:MinSizeRel>:-flto>)                  # 链接期优化，进一步缩减体积
    endforeach()

    foreach(_target IN LISTS ARG_LINK_TARGETS)
        if(NOT TARGET "${_target}")
            message(FATAL_ERROR "Compiler link target does not exist: ${_target}")
        endif()
        if(NOT APPLE)
            if(ARG_STACK_SIZE_OPTION STREQUAL "RECOMMENDED")
                # 设置主线程栈为 2MB（默认仅 8MB 在 Linux 已够，此为保守推荐）
                target_link_options("${_target}" PRIVATE -Wl,-z,stack-size=2097152)
            elseif(ARG_STACK_SIZE_OPTION STREQUAL "LARGE_PROJECT")
                # 设置主线程栈为 4MB，适配较深调用栈的大中型工程
                target_link_options("${_target}" PRIVATE -Wl,-z,stack-size=4194304)
            elseif(ARG_STACK_SIZE_OPTION STREQUAL "EXTREME_RECURSION")
                # 设置主线程栈为 8MB，适配深度递归 / 模板实例化场景
                target_link_options("${_target}" PRIVATE -Wl,-z,stack-size=8388608)
            endif()
        endif()

        # 链接期 LTO，与编译期 -flto 配合实现跨翻译单元优化
        target_link_options("${_target}" PRIVATE
            $<$<CONFIG:Release>:-flto>
            $<$<CONFIG:MinSizeRel>:-flto>)
        if(APPLE)
            # 移除未引用代码/数据（ld64 dead_strip），缩减最终二进制体积
            target_link_options("${_target}" PRIVATE
                $<$<CONFIG:Release>:-Wl,-dead_strip>
                $<$<CONFIG:MinSizeRel>:-Wl,-dead_strip>)
        else()
            # 链接器回收未引用段（需编译期 -ffunction-sections/-fdata-sections 配合）
            target_link_options("${_target}" PRIVATE
                $<$<CONFIG:Release>:-Wl,--gc-sections>
                $<$<CONFIG:MinSizeRel>:-Wl,--gc-sections>)
        endif()
    endforeach()

    if(APPLE)
        message(STATUS "  └─ Stack size: macOS uses system default (adjustable via ulimit)")
    elseif(ARG_STACK_SIZE_OPTION STREQUAL "DEFAULT")
        message(STATUS "  └─ Stack size: Using system default")
    elseif(ARG_STACK_SIZE_OPTION STREQUAL "RECOMMENDED")
        message(STATUS "  └─ Stack size: 2MB (Recommended)")
    elseif(ARG_STACK_SIZE_OPTION STREQUAL "LARGE_PROJECT")
        message(STATUS "  └─ Stack size: 4MB (Large Project)")
    elseif(ARG_STACK_SIZE_OPTION STREQUAL "EXTREME_RECURSION")
        message(STATUS "  └─ Stack size: 8MB (Extreme Recursion)")
    endif()

    if(ARG_ENABLE_WARNINGS)
        message(STATUS "  └─ Warnings: Enhanced (-Wall -Wextra + additional)")
    else()
        message(STATUS "  └─ Warnings: Standard (-Wall)")
    endif()
endfunction()
