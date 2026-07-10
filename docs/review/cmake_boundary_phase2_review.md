# CMake 边界第二阶段完成记录

Status: draft
Authority: CMake directory boundary review and implementation record
Date: 2026-07-09
Implementation-State: completed

## 结论

审查报告中建议迭代顺序的 Phase 2A 到 Phase 2D 已完成。当前 CMake 边界从
“目录先行”推进为“目标显式应用”：编译器/coverage/unity/PCH 不再通过 include
向目录级全局注入编译/链接选项，1q 自有 target 通过 `TargetBuildOptions.cmake`
显式接收这些策略，vendor target 不再被隐式污染。

## 完成项

### Phase 2A: 文档契约修正

- `cmake/README.md` 已重写为当前真实边界。
- 不再声称 `compilers/` / `features/` 可零修改完全复用。
- README 明确说明 clang-format/tidy 仍带 1q 默认源码树约定。
- README 明确 `packaging/` 只负责 Conan toolchain/bootstrap，真实依赖发现和链接在 `project/`。

### Phase 2B: 低风险命名和空壳清理

- `CompilerMSVC.cmake` 内部变量不再使用 `ONEQ_MSVC_COMMON_COMPILE_OPTIONS`。
- 删除空壳式 `cmake/packaging/PackageManagerConan.cmake`。
- PCH 不再放在 `features/` 中作为提示文件；真实策略迁移到 `cmake/project/TargetPrecompiledHeaders.cmake`。
- VS2015 源码归一化入口迁移到 `cmake/project/legacy/Vs2015SourceNormalization.cmake`。

### Phase 2C: `ProjectDependencies.cmake` 拆分

`cmake/project/ProjectDependencies.cmake` 现在只作为稳定入口，内部职责拆成：

- `cmake/project/dependencies/Jsbsim.cmake`：JSBSim 来源解析，导出 `JSBSim::JSBSim` 和 `ONEQ_JSBSIM_*` 状态。
- `cmake/project/dependencies/ConanPackages.cmake`：Conan package discovery 和基础依赖集合。
- `cmake/project/dependencies/ModuleLinkMatrix.cmake`：1q 模块 target 链接矩阵、optional SAR HDF5、spdlog/zlib compile definitions。
- `cmake/project/PackageConfigDependencies.cmake`：`ProjectTemplateConfig.cmake.in` 所需 `find_dependency` 注入块。

### Phase 2D: 通用层 target 化

- `cmake/compilers/CompilerClangGCC.cmake` 只定义 `apply_clang_gcc_options()`。
- `cmake/compilers/CompilerMSVC.cmake` 只定义 `apply_msvc_options()`。
- `cmake/features/FeatureCoverage.cmake` 只定义 `apply_coverage_options()`。
- `cmake/features/FeatureUnityBuild.cmake` 只定义 `apply_unity_build()`。
- `cmake/project/TargetBuildOptions.cmake` 对 `${PROJECT_CORE_TARGET}` 和 `${ONEQ_OBJECT_TARGETS}` 统一应用 compiler、unity、coverage、PCH 策略。
- `ProjectDependencies.cmake` 中原先为 vendor JSBSim 临时移除/恢复 `-fvisibility=hidden` 的目录级 workaround 已删除。

## 额外整理

- `cmake/project/BuildOptions.cmake`：通用构建选项。
- `cmake/project/OneqModuleOptions.cmake`：1q 模块选项。
- `cmake/project/ReplaySchemas.cmake`：replay FlatBuffers schema manifest。

## 验证

通过：

- `cmake --preset llvm-ninja-debug`
- `cmake --build --preset llvm-ninja-debug --target 1q_contract_tests`
- `ctest --test-dir build/llvm-ninja-debug-local -R 'public_api_boundary_guard|install_manifest_guard|public_header_cxx11_guard|docs_structure_guard|contract::1q_contract_tests' --output-on-failure`
- `cmake --fresh --preset llvm-ninja-coverage`
- PCH 独立验证：
  - `cmake -S /Users/aurora/Code/1q -B /private/tmp/1q-cmake-pch-options -G Ninja -DPACKAGE_MANAGER=conan -DCMAKE_TOOLCHAIN_FILE=/Users/aurora/Code/1q/build/llvm-ninja-debug-local/build/Debug/generators/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Debug -DENABLE_TESTING=OFF -DBUILD_SHARED_LIBS=OFF -DENABLE_PCH=ON -DENABLE_UNITY_BUILD=OFF`
  - `cmake --build /private/tmp/1q-cmake-pch-options --target 1q_lib`
- JSBSim source-mode boundary 验证：
  - `cmake -S /Users/aurora/Code/1q -B /private/tmp/1q-cmake-jsbsim-source -G Ninja -DPACKAGE_MANAGER=conan -DCMAKE_TOOLCHAIN_FILE=/Users/aurora/Code/1q/build/llvm-ninja-debug-local/build/Debug/generators/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Debug -DENABLE_TESTING=OFF -DBUILD_SHARED_LIBS=OFF -DONEQ_JSBSIM_FROM_SOURCE=ON`
  - `rg -n "third_party/jsbsim.*fvisibility=hidden" /private/tmp/1q-cmake-jsbsim-source/compile_commands.json` 无命中。
  - `rg -n "src/CMakeFiles/(airborne|1q_lib|sar|sbirs|eos|esr).*fvisibility=hidden" /private/tmp/1q-cmake-jsbsim-source/compile_commands.json` 有命中，证明 1q 自有 target 仍显式接收 visibility 策略。

## 已知非 CMake 边界限制

`ENABLE_UNITY_BUILD=ON` 的临时构建验证暴露源代码层面的 unity-build 冲突：部分 `.cpp`
在匿名命名空间中定义同名 helper，合并到同一 translation unit 后出现重定义。例如 EOS
debug/lifecycle 文件中的 `FindRecord` / `FindAttribution`，ESR debug/lifecycle 文件中的
`FindAssociation`。这不是本次 CMake 边界 target 化引入的新耦合，而是当前源码尚未满足
unity-build 兼容性的既有限制。

后续若要让 `ENABLE_UNITY_BUILD=ON` 成为可构建配置，需要单独做源码层处理：重命名文件局部
helper、抽共享 helper，或对冲突源设置 `SKIP_UNITY_BUILD_INCLUSION`。
