# 代码覆盖率插桩配置 (LLVM source-based coverage)
# 本文件在 ENABLE_COVERAGE=ON 时注入 -fprofile-instr-generate -fcoverage-mapping
#
# 与传统 gcov (-fprofile-arcs -ftest-coverage) 相比，source-based coverage
# 基于 LLVM 源码 region，分支覆盖率精度更高，是 Clang/Apple 官方推荐路径。
# 报告生成见 tools/coverage_report.sh (llvm-profdata merge + llvm-cov export/show)。

if(NOT ENABLE_COVERAGE)
    return()
endif()

# 编译器校验：source-based coverage 仅 Clang 系支持 (Clang / AppleClang)
if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    message(FATAL_ERROR
        "ENABLE_COVERAGE requires a Clang-based compiler (Clang/AppleClang) for "
        "LLVM source-based coverage, but CMAKE_CXX_COMPILER_ID is "
        "'${CMAKE_CXX_COMPILER_ID}'. Use an llvm-ninja-* preset or pass "
        "-DCMAKE_CXX_COMPILER=clang++ explicitly.")
endif()

# 注入插桩标志。
# - 编译期: -fprofile-instr-generate (生成 .profraw 运行时计数器)
#           -fcoverage-mapping       (生成源码 region 映射，供 llvm-cov 解析)
# - 链接期: -fprofile-instr-generate (链接运行时库 libclang_rt.profile)
#
# 采用全局 add_compile_options/add_link_options，与项目现有 -flto / -g3 等
# 全局标志风格一致。third_party (JSBSim 等) 通过 Conan 预编译或独立 target
# 链接，不经过本工程重新编译，故不受影响。
add_compile_options(-fprofile-instr-generate -fcoverage-mapping)
add_link_options(-fprofile-instr-generate)

message(STATUS "Coverage: Enabled (LLVM source-based)")
message(STATUS "  └─ Compile flags: -fprofile-instr-generate -fcoverage-mapping")
message(STATUS "  └─ Report: ./tools/coverage_report.sh --preset llvm-ninja-coverage")
