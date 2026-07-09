# CMake 配置模块

本目录包含项目的 CMake 配置模块，采用**分层模块化设计**，将可复用的通用
构建能力与项目特定的胶水代码物理分离，便于在未来项目中复用。

## 目录结构

```
cmake/
├── README.md                          本文件
├── compilers/                         编译器特定配置（完全可复用）
│   ├── CompilerClangGCC.cmake         GCC / Clang 共用配置
│   └── CompilerMSVC.cmake             MSVC 专用配置
├── features/                          可开关构建特性（完全可复用）
│   ├── FeatureCCache.cmake            ccache 编译缓存
│   ├── FeatureClangFormat.cmake       clang-format 代码格式化
│   ├── FeatureClangTidy.cmake         clang-tidy 静态分析
│   ├── FeatureCoverage.cmake          LLVM source-based 覆盖率插桩
│   ├── FeaturePrecompiledHeaders.cmake 预编译头
│   └── FeatureUnityBuild.cmake        Unity Build 加速编译
├── packaging/                         包管理器 / 工具链引导
│   ├── ConanBootstrapToolchain.cmake  Conan install 引导工具链
│   └── PackageManagerConan.cmake      Conan 依赖集成入口
└── project/                           项目特定配置与胶水
    ├── EnsureCRLF.cmake               VS2015 CRLF + BOM 归一化
    ├── FeatureFlatBuffers.cmake       FlatBuffers 代码生成（oneq_ 前缀 + sensor 布局）
    ├── ProjectDependencies.cmake      第三方依赖加载与链接
    ├── ProjectInstall.cmake           安装规则
    ├── ProjectLanguageDefaults.cmake  C++ 标准单一事实源
    ├── ProjectOptions.cmake           用户可配置选项
    ├── ProjectTemplateConfig.cmake.in PackageConfig 模板
    └── TargetProperties.cmake         目标属性公共配置
```

## 分层模型

| 层 | 目录 | 复用性 | 边界契约 |
|----|------|--------|---------|
| **编译器层** | `compilers/` | 完全可复用 | 零项目引用，消费 `STACK_SIZE_OPTION`、`ENABLE_WARNINGS` 等通用选项 |
| **特性层** | `features/` | 完全可复用 | 零项目引用，每个 Feature 自治（含开关、工具发现、标志注入） |
| **打包层** | `packaging/` | 大部分可复用 | `ConanBootstrapToolchain` → `project/` 的跨目录依赖（cppstd 取值） |
| **项目层** | `project/` | 项目特定 | 消费 `PROJECT_CORE_TARGET`、`ONEQ_OBJECT_TARGETS` 等项目变量 |

**干净的接缝**：`compilers/` 与 `features/` 中的文件不碰任何 `ONEQ_*` 变量，
未来复用时可直接复制这两个目录到新项目，无需修改。

> `FeatureFlatBuffers.cmake` 虽以 `Feature` 开头，但含 `oneq_` 函数前缀和
> `<sensor>/session/generated/` 项目特定布局，故归入 `project/` 而非 `features/`。

## 模块详解

### compilers/ — 编译器特定配置

#### CompilerClangGCC.cmake

**用途**: **GCC 和 Clang 共用**的编译器特定配置
> 注：文件名虽包含 "GCC"，但实际服务 GCC + Clang 两种编译器
> （`CMakeLists.txt` 用 `if(MSVC) ... else()` 二选一，Clang 走此分支）。
> 两者命令行标志高度兼容，仅在 6 个 GCC 专属警告处分叉
> （已用 `if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")` 隔离）。

**包含**:

- 栈大小配置（通过 `-Wl,-z,stack-size`，仅 Linux；macOS 用系统默认）
- 13 个通用警告（GCC/Clang 共享）+ 6 个 GCC 特定警告
- Debug/Release/RelWithDebInfo/MinSizeRel 各 build type 的优化/调试标志
- LTO（`-flto`）与死代码消除（macOS `-dead_strip` / Linux `--gc-sections`）
- 位置无关代码（PIC）与符号隐藏（`-fvisibility=hidden`）

#### CompilerMSVC.cmake

**用途**: MSVC 编译器特定配置
**包含**:

- 栈大小配置（通过 `/STACK` 标志）
- 19 个额外警告标志
- Debug/Release/RelWithDebInfo/MinSizeRel 优化配置
- LTCG（链接时代码生成）
- 函数级链接优化

### features/ — 可开关构建特性

#### FeatureUnityBuild.cmake

- 根据 `ENABLE_UNITY_BUILD` 选项启用/禁用
- 配置批次大小（默认 16 个文件/批次）
- MSVC 自动添加 `/bigobj` 标志

#### FeatureCCache.cmake

- 自动搜索 ccache 可执行文件
- 配置 C/C++ 编译器启动器
- 显示版本信息和友好错误提示

#### FeaturePrecompiledHeaders.cmake

- 预编译头配置通知（实际的 PCH 应在各目标中单独配置）

#### FeatureClangTidy.cmake

- 多版本搜索（clang-tidy-20/19/18/17/16）
- macOS 优先搜索 Homebrew LLVM 路径
- 支持低风险白名单检查项配置（`CLANG_TIDY_CHECKS`）
- 自动检测 `.clang-tidy` 配置文件
- 显示安装说明

#### FeatureClangFormat.cmake

- 多版本搜索（clang-format-20/19/18/17/16）
- macOS 优先搜索 Homebrew LLVM 路径
- 提供 `format` 与 `format-check` 构建目标

#### FeatureCoverage.cmake

- `ENABLE_COVERAGE=ON` 时注入 `-fprofile-instr-generate -fcoverage-mapping`
- 强制要求 Clang 系编译器（Clang/AppleClang），其他编译器直接 `FATAL_ERROR`
- 报告生成见 `tools/coverage_report.sh`（`llvm-profdata merge` + `llvm-cov export/show`）
- 配套 preset 为 `llvm-ninja-coverage`

### packaging/ — 包管理器 / 工具链引导

#### ConanBootstrapToolchain.cmake

**用途**: 作为 `CMAKE_TOOLCHAIN_FILE` 的引导脚本，按需自动执行 `conan install`
**说明**:

- 比对 `conanfile.py` SHA256、build type、cppstd、fingerprint，决定是否重新 install
- 最终 `include` Conan 生成的 `conan_toolchain.cmake`
- 处理 MSVC 多配置生成器、VS2015 cppstd 映射等边界情况
- ⚠️ **跨目录依赖**: `include("${CMAKE_CURRENT_LIST_DIR}/../project/ProjectLanguageDefaults.cmake")`
  以保证 cppstd 取值与项目层一致

#### PackageManagerConan.cmake

**用途**: Conan 包管理器依赖配置（在 target 创建之后包含）
**说明**: 当前为集成入口 stub，实际依赖装配在 `project/ProjectDependencies.cmake`

### project/ — 项目特定配置

#### ProjectLanguageDefaults.cmake

**用途**: 语言/C++ 标准单一事实源（当前 `C++17`）
**说明**: `cppstd=17` 由 `jsbsim/1.3.1` 依赖要求驱动；公共头 `include/1q/`
守 C++11 子集以保证 VS2015 消费方可编译

#### ProjectOptions.cmake

**用途**: 集中定义所有用户可配置选项（**必须最先包含**）
**包含选项**:

- `BUILD_SHARED_LIBS` - 构建动态库/静态库
- `ENABLE_TESTING` - 启用测试支持
- `ENABLE_EXAMPLES` - 构建示例程序
- `ENABLE_INSTALL` - 启用安装规则
- `ENABLE_UNITY_BUILD` - Unity Build 加速编译
- `USE_CCACHE` - ccache 缓存加速
- `ENABLE_PCH` - 预编译头
- `ENABLE_WARNINGS` - 额外警告
- `ENABLE_COVERAGE` - LLVM source-based 覆盖率插桩（仅 Clang）
- `ENABLE_CLANG_TIDY` - 静态分析
- `CLANG_TIDY_CHECKS` - clang-tidy 检查项白名单
- `ONEQ_ENABLE_HDF5_OUTPUT` - SAR HDF5 图像输出（需 HighFive）
- `ONEQ_ENABLE_FLIGHT_DYNAMIC` - 构建 flight_dynamic 机动模块
- `STACK_SIZE_OPTION` - 栈大小配置
- `PACKAGE_MANAGER` - 包管理器选择（none/conan）

#### ProjectDependencies.cmake

**用途**: 第三方依赖加载与链接（JSBSim、Eigen、Boost、nanoflann、flatbuffers、spdlog、ZLIB、可选 HighFive）
**功能**:

- 三种 JSBSim 获取路径：Conan 预编译 / `third_party` 源码构建 / prebuilt 目录
- 按 sensor 子模块分发链接依赖
- 生成供 `ProjectTemplateConfig.cmake.in` 注入的 `find_dependency` 块

#### ProjectInstall.cmake

**用途**: 安装规则（库目标、头文件白名单、CMake 包配置）
**说明**: 仅在 `ENABLE_INSTALL=ON` 时生效；消费同目录的 `ProjectTemplateConfig.cmake.in`

#### FeatureFlatBuffers.cmake

**用途**: FlatBuffers 构建期代码生成（`schemas/replay/*.fbs` → `*_generated.h`）
**说明**:

- 定义 `oneq_setup_flatc()` 和 `oneq_flatbuffers_generate()` 两个宏/函数
- flatc 来自 Conan flatbuffers 包暴露的 `flatbuffers::flatc` imported target
- 实际调用发生在 `src/CMakeLists.txt` 的 `find_package(flatbuffers)` 之后
- 生成头输出到 `${CMAKE_BINARY_DIR}/generated`，不进 git

#### TargetProperties.cmake

**用途**: 目标属性公共配置
**功能**:

- MSVC 运行时库（`MSVC_RUNTIME_LIBRARY`）显式设置
- 静态库 `ONEQ_STATIC_DEFINE` 宏定义
- 输出名称、`DEBUG_POSTFIX d`、`VERSION`/`SOVERSION`
- 非 Windows 平台默认隐藏符号（`CXX_VISIBILITY_PRESET hidden`）

#### EnsureCRLF.cmake

**用途**: VS2015 源码 CRLF + UTF-8 BOM 归一化
**说明**:

- 仅对 Visual Studio 14 2015 生成器生效，其他环境直接 `return()`
- 默认关闭（`ONEQ_VS2015_NORMALIZE_SOURCE=OFF`），避免脏化 checkout
- 修复 VS2015 在含多字节 UTF-8 字符（如中文注释）源码上的解析 bug

#### ProjectTemplateConfig.cmake.in

**用途**: `ProjectInstall.cmake` 通过 `configure_package_config_file()` 消费的模板
**说明**: 生成供下游 `find_package()` 使用的 `1qConfig.cmake`

## 模块依赖关系

```
CMakeLists.txt (主文件)
  ├── cmake/project/ProjectLanguageDefaults.cmake   # cppstd 单一事实源（最早）
  ├── cmake/project/ProjectOptions.cmake            # 必须在所有 Feature 之前
  ├── cmake/features/FeatureUnityBuild.cmake        # ENABLE_UNITY_BUILD
  ├── cmake/features/FeatureCCache.cmake            # USE_CCACHE
  ├── cmake/features/FeaturePrecompiledHeaders.cmake# ENABLE_PCH
  ├── cmake/features/FeatureClangTidy.cmake         # ENABLE_CLANG_TIDY
  ├── cmake/features/FeatureClangFormat.cmake       # 提供 format/format-check 目标
  ├── cmake/features/FeatureCoverage.cmake          # ENABLE_COVERAGE（仅 Clang）
  ├── cmake/project/FeatureFlatBuffers.cmake        # 只定义函数，flatc 在 src/ 中调用
  ├── cmake/compilers/Compiler{MSVC|ClangGCC}.cmake # 根据 if(MSVC) 二选一
  ├── cmake/project/EnsureCRLF.cmake                # 仅 VS2015 生效
  └── add_subdirectory(src)                         # 创建 ${PROJECT_CORE_TARGET} 及各 sensor 目标
        ├── cmake/project/ProjectDependencies.cmake # find_package + 链接第三方依赖
        │     └── 注入 ONEQ_CONFIG_FIND_DEPENDENCIES（供 .cmake.in 模板）
        ├── cmake/project/FeatureFlatBuffers.cmake 函数体在此调用
        └── cmake/project/TargetProperties.cmake    # 设置输出名/可见性/CRT 等目标属性

  # target 创建之后
  └── cmake/packaging/PackageManagerConan.cmake     # PACKAGE_MANAGER=conan 时包含

  # ENABLE_INSTALL=ON 时（在 src/CMakeLists.txt 末尾）
  └── cmake/project/ProjectInstall.cmake            # install(EXPORT)，消费 ProjectTemplateConfig.cmake.in

# 旁路（非主 CMakeLists.txt 包含）：
cmake/packaging/ConanBootstrapToolchain.cmake       # 作为 CMAKE_TOOLCHAIN_FILE，引导 conan install
  └── include ../project/ProjectLanguageDefaults.cmake  # 跨目录依赖，保证 cppstd 一致
```

## 使用方法

### 在主 CMakeLists.txt 中包含

```cmake
include(cmake/project/ProjectLanguageDefaults.cmake)   # cppstd 默认值（最早）
# ... project() / build type 配置 ...

include(cmake/project/ProjectOptions.cmake)             # 必须在所有 Feature 之前

include(cmake/features/FeatureUnityBuild.cmake)
include(cmake/features/FeatureCCache.cmake)
include(cmake/features/FeaturePrecompiledHeaders.cmake)
include(cmake/features/FeatureClangTidy.cmake)
include(cmake/features/FeatureClangFormat.cmake)
include(cmake/features/FeatureCoverage.cmake)
include(cmake/project/FeatureFlatBuffers.cmake)

# 编译器特定配置（二选一，Clang 走 ClangGCC 分支）
if(MSVC)
    include(cmake/compilers/CompilerMSVC.cmake)
else()
    include(cmake/compilers/CompilerClangGCC.cmake)
endif()

include(cmake/project/EnsureCRLF.cmake)                 # 仅 VS2015 生效

add_subdirectory(src)                           # 在此创建各 target
  # src/CMakeLists.txt 内部依次包含：
  #   cmake/project/ProjectDependencies.cmake → FeatureFlatBuffers 函数调用
  #   → cmake/project/TargetProperties.cmake
  #   → cmake/project/ProjectInstall.cmake

# target 创建之后
if(PACKAGE_MANAGER STREQUAL "conan")
    include(cmake/packaging/PackageManagerConan.cmake)
endif()
```

### 配置选项示例

```bash
# 启用 Unity Build 和 ccache
cmake -B build -DENABLE_UNITY_BUILD=ON -DUSE_CCACHE=ON

# 启用静态分析（慎用，会显著增加编译时间）
cmake -B build -DENABLE_CLANG_TIDY=ON

# 运行格式化检查（不改文件）
cmake --build build --target format-check

# 执行格式化（会直接改文件）
cmake --build build --target format

# 配置大栈空间
cmake -B build -DSTACK_SIZE_OPTION=LARGE_PROJECT

# 使用 Conan 包管理器
cd build
conan install .. --output-folder=. --build=missing
cmake .. -DPACKAGE_MANAGER=conan
```

## 复用指南

将本项目 cmake 配置复用到新项目时：

1. **直接复制**（零修改）：`compilers/` 和 `features/` 全部文件
2. **复制后微调**：`packaging/ConanBootstrapToolchain.cmake`（改 cppstd、删 VS2015 特化）
3. **重写**：`project/` 全部文件（这些绑定到本项目的依赖、目标名、sensor 布局）

接缝点是两个项目变量：`PROJECT_CORE_TARGET` 和 `ONEQ_OBJECT_TARGETS`。
项目层所有文件都消费它们，可复用层都不碰。

## 扩展指南

### 添加新的特性模块

1. 创建 `cmake/features/FeatureNewFeature.cmake`
2. 添加选项到 `cmake/project/ProjectOptions.cmake`
3. 在主 CMakeLists.txt 中 include
4. 更新本 README

### 添加新的编译器支持

1. 创建 `cmake/compilers/CompilerNewCompiler.cmake`
2. 在主 CMakeLists.txt 中添加编译器检测逻辑
3. 更新本 README

## 最佳实践

1. **包含顺序**: 始终先包含 `ProjectLanguageDefaults.cmake`，再 `ProjectOptions.cmake`
2. **错误处理**: 在模块中使用 `FATAL_ERROR` 提示必需工具缺失
3. **状态消息**: 使用树形结构（`└─`）提升可读性
4. **注释完整**: 每个模块开头应有用途说明
5. **选项验证**: 在模块中验证选项有效性
6. **路径规范**: cmake 模块间的相互引用优先使用 `${CMAKE_CURRENT_LIST_DIR}`
   （跨目录时用 `../`），避免硬编码 `${CMAKE_SOURCE_DIR}/cmake/...` 绝对路径
