# CMake 配置模块

本目录采用分层模块化设计：通用构建能力以函数或工具入口暴露，1q 项目特定
胶水集中在 `project/`。当前边界以“显式目标应用”为主，避免 include 后把
编译选项污染到 vendor、tests、examples 或 tools。

## 目录结构

```
cmake/
├── README.md
├── compilers/                         编译器族 target option helpers
│   ├── CompilerClangGCC.cmake         apply_clang_gcc_options()
│   └── CompilerMSVC.cmake             apply_msvc_options()
├── features/                          通用工具/feature helpers
│   ├── FeatureCCache.cmake            ccache compiler launcher
│   ├── FeatureClangFormat.cmake       format / format-check targets
│   ├── FeatureClangTidy.cmake         CMAKE_CXX_CLANG_TIDY 配置
│   ├── FeatureCoverage.cmake          apply_coverage_options()
│   └── FeatureUnityBuild.cmake        apply_unity_build()
└── project/                           1q 项目特定配置与胶水
    ├── BuildOptions.cmake             通用构建选项
    ├── FeatureFlatBuffers.cmake       FlatBuffers 代码生成函数
    ├── OneqModuleOptions.cmake        1q 模块选项
    ├── PackageConfigDependencies.cmake find_dependency 注入块
    ├── ProjectDependencies.cmake      第三方依赖入口
    ├── ProjectInstall.cmake           安装规则
    ├── ProjectLanguageDefaults.cmake  C++ 标准单一事实源
    ├── ProjectOptions.cmake           option 稳定入口
    ├── ProjectTemplateConfig.cmake.in PackageConfig 模板
    ├── ReplaySchemas.cmake            replay schema manifest
    ├── TargetBuildOptions.cmake       自有 target 构建选项应用
    ├── TargetPrecompiledHeaders.cmake PCH target policy
    ├── TargetProperties.cmake         输出名/可见性/CRT 等目标属性
    ├── dependencies/
    │   ├── ConanPackages.cmake        Conan package discovery
    │   ├── Jsbsim.cmake               JSBSim 来源解析
    │   └── ModuleLinkMatrix.cmake     模块依赖链接矩阵
    └── legacy/
        └── Vs2015SourceNormalization.cmake
```

## 分层模型

| 层 | 目录 | 边界契约 |
|----|------|---------|
| 编译器层 | `compilers/` | 只定义 target-scoped helper；不直接调用 `add_compile_options()` / `add_link_options()`。 |
| 特性层 | `features/` | 提供工具查找、custom target 或 target-scoped helper；clang-format/tidy 仍带 1q 默认源码树约定。 |
| 项目层 | `project/` | 消费 `PROJECT_CORE_TARGET`、`ONEQ_OBJECT_TARGETS`、sensor/replay 布局和安装导出契约。 |

> Conan 依赖引导已从 `cmake/` 移出，改由仓库根的 `scripts/bootstrap_conan.sh`
> 负责：它按 preset 名派生参数并执行 `conan install`，生成的 `conan_toolchain.cmake`
> 由各 preset 的 `CMAKE_TOOLCHAIN_FILE` 直接引用。真实依赖发现和 target 链接仍在 `project/`。

`compilers/` 和 `features/` 是通用构建层，不再声称可直接零修改复制到任意项目：
`FeatureClangFormat.cmake` / `FeatureClangTidy.cmake` 仍默认使用本仓库源码布局和
`.clang-tidy`；真正复用时应参数化这些路径。

## 主调用链

```
CMakeLists.txt
  ├── cmake/project/ProjectLanguageDefaults.cmake
  ├── cmake/project/ProjectOptions.cmake
  │     ├── BuildOptions.cmake
  │     └── OneqModuleOptions.cmake
  ├── cmake/features/*.cmake           # 定义工具目标或 target helper
  ├── cmake/project/FeatureFlatBuffers.cmake
  ├── cmake/project/TargetPrecompiledHeaders.cmake
  ├── cmake/compilers/Compiler{MSVC|ClangGCC}.cmake
  ├── cmake/project/legacy/Vs2015SourceNormalization.cmake
  └── add_subdirectory(src)
        ├── cmake/project/ProjectDependencies.cmake
        │     ├── dependencies/Jsbsim.cmake
        │     ├── dependencies/ConanPackages.cmake
        │     ├── dependencies/ModuleLinkMatrix.cmake
        │     └── PackageConfigDependencies.cmake
        ├── cmake/project/ReplaySchemas.cmake
        ├── cmake/project/TargetProperties.cmake
        ├── cmake/project/TargetBuildOptions.cmake
        └── cmake/project/ProjectInstall.cmake

scripts/bootstrap_conan.sh <preset>     # 构建前先跑：conan install 生成 toolchain
```

## 模块说明

### compilers/

`CompilerClangGCC.cmake` 和 `CompilerMSVC.cmake` 只定义函数。`src/CMakeLists.txt`
通过 `TargetBuildOptions.cmake` 对 `${PROJECT_CORE_TARGET}` 与
`${ONEQ_OBJECT_TARGETS}` 显式应用 warning、优化、LTO、stack-size、visibility 等
策略。vendor 目标不再被这些设置隐式影响。

### features/

- `FeatureCCache.cmake`：配置 C/C++ compiler launcher。
- `FeatureClangFormat.cmake`：创建 `format` / `format-check` 目标；默认扫描
  `include/`、`src/`、`tests/`、`examples/`、`tools/`。
- `FeatureClangTidy.cmake`：在 `ENABLE_CLANG_TIDY=ON` 时配置
  `CMAKE_CXX_CLANG_TIDY`，默认读取仓库根 `.clang-tidy`。
- `FeatureCoverage.cmake`：定义 `apply_coverage_options()`，只对显式传入的
  target 注入 LLVM source-based coverage flags。
- `FeatureUnityBuild.cmake`：定义 `apply_unity_build()`，只对显式传入的 target
  设置 `UNITY_BUILD`。

PCH 不在 `features/` 中实现，因为实际头集合是 1q target policy；见
`project/TargetPrecompiledHeaders.cmake`。

### project/

- `ProjectOptions.cmake` 是稳定入口，实际拆到 `BuildOptions.cmake` 与
  `OneqModuleOptions.cmake`。
- `ProjectDependencies.cmake` 是第三方依赖入口，内部按来源解析、package discovery、
  module link matrix、PackageConfig dependency block 分文件维护。
- `FeatureFlatBuffers.cmake` 定义 `setup_flatc()` 和
  `flatbuffers_generate()`；schema manifest 在 `ReplaySchemas.cmake`。
- `TargetBuildOptions.cmake` 对项目自有 target 应用 compiler、unity、coverage、PCH
  策略。
- `legacy/Vs2015SourceNormalization.cmake` 是默认关闭的 VS2015 源码归一化兼容入口；
  它可能重写源码树，因此保持在 legacy 下并由显式选项控制。

## 扩展规则

1. 新 compiler 文件应只定义函数，不应 include 即修改目录级 compile/link options。
2. 新 feature 若依赖 1q 源码树布局，应在 README 中明确标注，不要标成纯通用。
3. 新第三方依赖先进入 `dependencies/ConanPackages.cmake`，再在
   `dependencies/ModuleLinkMatrix.cmake` 中声明消费 target。
4. 新 replay schema 同时更新 `ReplaySchemas.cmake`、codec/trace/replay 代码和
   contract 测试。
5. 安装公共头继续通过 `ProjectInstall.cmake` 与 `src/*/CMakeLists.txt` 显式白名单
   维护，并由 install/public boundary guards 校验。
