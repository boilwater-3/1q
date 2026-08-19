# ProjectOptions.cmake
# 项目构建选项入口：集中声明所有用户可配置开关
# cmake/project 的唯一选项边界，业务模块不重复定义缓存变量

# 共享库 / 测试 / 示例 / 安装四个顶层开关。
option(BUILD_SHARED_LIBS "Build shared libraries instead of static" ON)
option(ENABLE_TESTING "Enable testing support" OFF)
option(ENABLE_EXAMPLES "Build example programs" OFF)
option(ENABLE_INSTALL "Enable installation rules" OFF)

# Unity Build 依赖 set_source_files_properties 的 UNITY_BUILD 属性，CMake 3.16 引入。
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.16")
    option(ENABLE_UNITY_BUILD "Enable Unity Build for faster compilation" OFF)
else()
    set(ENABLE_UNITY_BUILD OFF CACHE INTERNAL "Unity Build requires CMake 3.16+")
endif()

# ccache 是 UNIX 下的编译缓存工具，Windows 原生不支持。
if(UNIX)
    option(USE_CCACHE "Use ccache to accelerate rebuilds" ON)
else()
    set(USE_CCACHE OFF CACHE INTERNAL "ccache is primarily for Linux/macOS")
endif()

# 预编译头同样依赖 CMake 3.16 的 target_precompile_headers。
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.16")
    option(ENABLE_PCH "Enable precompiled headers" OFF)
    mark_as_advanced(ENABLE_PCH)
else()
    set(ENABLE_PCH OFF CACHE INTERNAL "Precompiled headers require CMake 3.16+")
endif()

# 编译期告警 / 覆盖率 / 静态分析三项开关。
option(ENABLE_WARNINGS "Enable additional compiler warnings" ON)
option(ENABLE_COVERAGE "Enable LLVM source-based coverage instrumentation (Clang only)" OFF)
mark_as_advanced(ENABLE_COVERAGE)
option(ENABLE_CLANG_TIDY "Enable clang-tidy static analysis" OFF)
mark_as_advanced(ENABLE_CLANG_TIDY)

# clang-tidy 检查项白名单：聚焦 readability/modernize/performance/bugprone 四类，
# 保留可直接机械修复的项，排除需人工判断语义的高噪声检查。
set(CLANG_TIDY_CHECKS
    "-*,readability-braces-around-statements,readability-else-after-return,readability-isolate-declaration,readability-qualified-auto,readability-redundant-access-specifiers,readability-redundant-control-flow,readability-redundant-preprocessor,readability-static-accessed-through-instance,modernize-redundant-void-arg,modernize-use-bool-literals,modernize-use-default-member-init,modernize-use-equals-default,modernize-use-equals-delete,modernize-use-nullptr,modernize-use-override,modernize-use-using,performance-for-range-copy,performance-implicit-conversion-in-loop,performance-move-constructor-init,performance-unnecessary-copy-initialization,performance-unnecessary-value-param,bugprone-macro-parentheses,bugprone-string-constructor,bugprone-string-integer-assignment"
    CACHE STRING "clang-tidy checks whitelist")
mark_as_advanced(CLANG_TIDY_CHECKS)

# 线程栈大小档位：DEFAULT 保留平台默认，RECOMMENDED 适合常规项目，
# LARGE_PROJECT 适配多线程大栈场景，EXTREME_RECURSION 仅用于深递归算法模块。
set(STACK_SIZE_OPTION "RECOMMENDED" CACHE STRING "Stack size configuration")
set_property(CACHE STACK_SIZE_OPTION PROPERTY STRINGS
    "DEFAULT" "RECOMMENDED" "LARGE_PROJECT" "EXTREME_RECURSION")
mark_as_advanced(STACK_SIZE_OPTION)

# 包管理器 / 依赖 provider：
#   conan —— 标准方式，由 scripts/bootstrap_conan.sh 完成 install 并生成 toolchain/CMakeDeps。
#   none  —— 不走 conan：第三方依赖来自 third_party/ 源码，由 scripts/fetch_third_party.bat
#            git clone 到位，dependencies/VendorPackages.cmake 以 add_subdirectory / header-only 消费。
# 两个 provider 对外契约相同（相同的 imported target 名与 ONEQ_LINK_DEPENDENCIES 等输出变量），
# 故 src/、tests/、install 等下游消费点不感知 provider 切换。
set(PACKAGE_MANAGER "conan" CACHE STRING
    "1q dependency provider: 'conan' (run scripts/bootstrap_conan.sh) or 'none' (run scripts/fetch_third_party.bat)")
set_property(CACHE PACKAGE_MANAGER PROPERTY STRINGS "conan" "none")
if(NOT PACKAGE_MANAGER STREQUAL "conan" AND NOT PACKAGE_MANAGER STREQUAL "none")
    message(FATAL_ERROR
        "Unsupported PACKAGE_MANAGER: '${PACKAGE_MANAGER}'. "
        "1q supports 'conan' (run scripts/bootstrap_conan.sh <preset>) or 'none' "
        "(run scripts/fetch_third_party.bat to populate third_party/).")
endif()
if(PACKAGE_MANAGER STREQUAL "none")
    message(STATUS "Package Manager: none (deps from third_party/, run scripts/fetch_third_party.bat)")
else()
    message(STATUS "Package Manager: Conan")
endif()

# 机动（maneuver）模块开关：默认关闭，按需启用。
option(ONEQ_ENABLE_FLIGHT_DYNAMIC "Build the flight_dynamic (maneuver) module" OFF)
if(ONEQ_ENABLE_FLIGHT_DYNAMIC)
    message(STATUS "flight_dynamic module: ENABLED")
else()
    message(STATUS "flight_dynamic module: disabled (set -DONEQ_ENABLE_FLIGHT_DYNAMIC=ON to enable)")
endif()

# 库内内置文件日志后端（ProjectFileLog）：Windows 上 spdlog 关闭时承载
# PROJECT_LOG_* 落盘 CWD/1q_library.log；Unix 上默认休眠但可测（SPDLOG 分支优先）。
# 总开关关闭时宏回到空操作且 sink 不编译。
option(ONEQ_ENABLE_FILE_LOG "Enable built-in file logging backend (used when spdlog is unavailable)" ON)
if(ONEQ_ENABLE_FILE_LOG)
    message(STATUS "file log backend: ENABLED (ProjectFileLog)")
else()
    message(STATUS "file log backend: disabled (PROJECT_LOG_* are no-ops)")
endif()

# SBIRS 验收信息日志（[SbirsAccept] 事件流）：开启后 sbirs_sensor 按周期输出
# WFOV 地面覆盖区/驻留时间、疑似目标与信号能量、宽窄切换连续命中、NFOV 捕获/跟踪、
# 焦平面脱靶量与通道协同事件（需求映射 3.2.1.3 章节验收量）。默认关闭：
# 宏与派生计算一并剪除，零开销。
option(ONEQ_ENABLE_SBIRS_ACCEPTANCE_LOG
    "Emit SBIRS acceptance information logs (ground footprint, dwell time, signal energy, focal-plane offsets, handover counters)" OFF)
if(ONEQ_ENABLE_SBIRS_ACCEPTANCE_LOG)
    message(STATUS "sbirs acceptance log: ENABLED ([SbirsAccept] events via PROJECT_LOG_INFO)")
else()
    message(STATUS "sbirs acceptance log: disabled (set -DONEQ_ENABLE_SBIRS_ACCEPTANCE_LOG=ON to enable)")
endif()

# 精度评估验收日志（[PrecisionEval] 事件流）：开启后评估会话逐周期输出红外角度误差、
# 双星交会位置误差、速度误差、落点/发射点预测误差样本与 AHP 综合评分（需求映射
# 3.2.1.6.3 章节）。默认关闭：宏与派生计算一并剪除，零开销。
option(ONEQ_ENABLE_PRECISION_EVALUATION_LOG
    "Emit precision evaluation logs (angular errors, dual-sat fix, velocity and key-point errors, AHP score)" OFF)
if(ONEQ_ENABLE_PRECISION_EVALUATION_LOG)
    message(STATUS "precision evaluation log: ENABLED ([PrecisionEval] events via PROJECT_LOG_INFO)")
else()
    message(STATUS "precision evaluation log: disabled (set -DONEQ_ENABLE_PRECISION_EVALUATION_LOG=ON to enable)")
endif()
