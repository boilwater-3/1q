# SAR FFT 后端阶段 2 研究记录

日期：2026-06-04

## 当前仓库事实

- `conanfile.py` 目前固定依赖 Eigen、Boost、nanoflann、FlatBuffers、Zlib；非 Windows 额外依赖 spdlog/fmt/JSBSim。
- `cmake/ProjectDependencies.cmake` 当前没有 `find_package(FFTW)`、KissFFT、PocketFFT、MKL、cuFFT 等 FFT 后端。
- `src/common/numerics::ZFFT1D` 是 O(n^2) 朴素 DFT，源码注释明确当前默认长度很小，`n > 512` 应替换为 Cooley-Tukey FFT 或 Eigen FFT。

## 工程判断

SAR Phase 1 的 RDA 需要 range 维与 azimuth 维频域处理。若直接复用 `ZFFT1D`，小型单元测试可以通过，但 1024x1024 级别图像会把性能风险隐藏到算法验收阶段，审批上不合理。

因此阶段 2 必须先冻结 FFT 后端与统一 API，再进入正式 RDA 实现。阶段 1 的会话/API 骨架不得承诺大尺寸 RDA 性能。

## 后端候选

| 候选 | 优点 | 风险 |
|---|---|---|
| Eigen unsupported FFT | 已有 Eigen 依赖，新增包管理成本低 | `unsupported/Eigen/FFT` 可用性、VS2015 兼容性和精度/性能需实测 |
| KissFFT/PocketFFT vendored | 小型、可控、适合封装 | 需要新增第三方源码、许可证审查、CMake/install 集成 |
| FFTW | 成熟高性能 | 依赖和许可证/平台包管理复杂度较高，不适合先手默认 |
| 继续使用 `ZFFT1D` | 无新增依赖 | 只能用于很小规模测试，不能作为 RDA 正式后端 |

## 推荐门禁

阶段 2 先实现 `sar::signal` 内部 FFT facade，公开头不暴露后端：

- `Fft1D(input, inverse)`
- `FftRows(matrix, inverse)`
- `FftCols(matrix, inverse)`

归一化约定：

- forward 不归一化。
- inverse 除以 N。

验收测试：

- complex round-trip。
- delta pulse。
- known sinusoid。
- rows/cols 轴向测试。
- 非 2 次幂长度策略测试。

## 当前验证

已在 `llvm-ninja-debug` 预设下验证 Eigen unsupported FFT 可编译，并通过 SAR 内部 FFT facade 单元测试：

- complex round-trip。
- delta pulse。
- known sinusoid。
- rows/cols 轴向测试。
- 非 2 次幂长度 round-trip。
- invalid input rejection。

验证命令：

```sh
cmake --preset llvm-ninja-debug
cmake --build --preset llvm-ninja-debug --target 1q_unit_tests
build/llvm-ninja-debug-local/bin/1q_unit_tests '--gtest_filter=SarFftBackendTest.*'
```

当前限制：

- 已验证 macOS AppleClang + Conan Eigen 3.4.0 的正确性与当前平台性能。
- Conan 新增项目级 `eigen_version=auto|3.3.9|3.4.0` 选项和 `llvm-ninja-debug-eigen339` preset。
- macOS AppleClang + Conan Eigen 3.3.9 已完成构建、SAR 正确性和性能验证。
- `sar_cxx11_compat` 已使用 C++11 + Eigen 3.3.9 编译全部 SAR engine 源文件。
- 按 Phase 1 决策，Windows/VS2015 不作为强制审批门。
- 1024x1024 二维 FFT facade Debug 核心变换当前观测约 344 ms，门限为 10 s。
- 1024x1024 内部 RDA 合成场景 Debug 核心处理当前观测约 934-996 ms，门限为 30 s。
- 1024x1024 真实点目标内部管线 Debug 当前观测约 1493 ms。
- 1024x1024 public Session Debug 当前观测约 1421-1496 ms，隔离运行峰值常驻内存约 137035776 bytes。

## 当前建议

阶段 2 冻结 Conan Eigen unsupported FFT，并以 C++11 + Eigen 3.3.9 当前平台编译门作为 Phase 1 兼容证据。Windows/VS2015 后续可作为非阻断验证项。当前 `sar_performance` 已覆盖 1024x1024 二维 FFT、内部 RDA、真实点目标内部管线和 public Session。
