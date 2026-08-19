# 1q 库消费指南

Status: active
Last-reviewed: 2026-08-20
Authority: build/install consumer guide

本文只描述已经由仓库 install/consumer 路径验证的消费方式。构建系统契约见
`docs/common/contract.md`；尚未验收的平台能力不得从 preset 或示例命令推断。

## 当前支持的消费方式

当前已验证的方式是：在能够解析 1q 第三方依赖的同一构建环境中安装 1q，然后由下游项目通过
`find_package(1q CONFIG REQUIRED)` 消费导出的 `1q::1q` target。CI 使用 Conan toolchain 和对应
依赖配置完成这条 install/consumer 验证。

安装树包含：

- 1q public headers、库文件、`1qConfig.cmake`、版本文件和 exported targets；
- 根据当前构建启用的依赖生成的 `find_dependency(...)` 声明。

这不等于“零依赖安装包”。安装树不复制第三方头文件、配置或二进制；下游须通过当前 toolchain、
`CMAKE_PREFIX_PATH` 或等价机制提供可解析的依赖 target。最稳妥的做法是让库构建、安装和 consumer
使用同一依赖环境。

## 构建并安装

以下示例使用仓库当前验证的 Release preset：

```bash
bash scripts/bootstrap_conan.sh llvm-ninja-release-local
cmake --preset llvm-ninja-release-local -D ENABLE_INSTALL=ON
cmake --build --preset llvm-ninja-release-local -j 4
cmake --install build/llvm-ninja-release-local
```

实际安装前缀由 preset 的 `CMAKE_INSTALL_PREFIX` 决定。CI 的精确 install/consumer 命令以
`.github/workflows/ci.yml` 为准。

## 下游 CMake

```cmake
cmake_minimum_required(VERSION 3.16)
project(my_app LANGUAGES CXX)

find_package(1q REQUIRED CONFIG)

add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE 1q::1q)
```

配置下游时必须同时提供 1q 安装前缀和可解析第三方依赖的环境。例如在同一 Conan 构建环境中：

```bash
cmake -S . -B build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE=/path/to/conan_toolchain.cmake \
  -DCMAKE_PREFIX_PATH=/path/to/1q/install
cmake --build build -j 4
```

调用方只链接 `1q::1q`，无需手工枚举 1q 自身的 include 目录或猜测库文件名；第三方 target 是否可解析
仍是 consumer 配置环境的责任。

## Conan 边界

仓库根 `conanfile.py` 当前是本项目的依赖 bootstrap recipe：它声明第三方 requirements，并生成
CMake toolchain/dependency 文件。它没有 `build()`、`package()` 或 `package_info()`，因此当前不提供
可由下游直接写成 `requires = "1q/0.1"` 的 1q Conan package。

在正式增加 Conan package/export 和独立 consumer 验证之前，不得把 `1q/0.1` 依赖写法作为受支持
消费方式。Conan 目前只负责构建环境中的第三方依赖解析。

## 导出 target 的实际属性

| 属性 | 当前规则 |
|---|---|
| Target | `1q::1q` |
| 库类型 | 跟随 `BUILD_SHARED_LIBS`；项目默认 ON，本地/CI presets 当前覆盖为 OFF |
| Public include | `${INSTALL_PREFIX}/include` |
| C++ 标准 | 项目默认 C++17；允许调用方显式设置不低于 C++11 的值 |
| 静态定义 | 仅静态构建公开 `ONEQ_STATIC_DEFINE` |
| 依赖 | 由 exported target 与 `1qConfig.cmake` 的 `find_dependency` 共同解析 |

不得把某个 preset 的静态构建选择写成项目全局默认，也不得把最低兼容探针误写成默认语言标准。

## 主要构建选项

| 选项 | 项目默认值 | 含义 |
|---|---:|---|
| `BUILD_SHARED_LIBS` | ON | 选择 shared/static 主库；常用 presets 当前设为 OFF |
| `ENABLE_TESTING` | OFF | 注册测试目标与 CTest |
| `ENABLE_EXAMPLES` | OFF | 构建第一方示例和 batch validation |
| `ENABLE_INSTALL` | OFF | 启用安装与 package-config 规则 |
| `ONEQ_ENABLE_FLIGHT_DYNAMIC` | OFF | 构建 flight_dynamic 模块及其专属测试/示例 |
| `ONEQ_ENABLE_FILE_LOG` | ON | 内置文件日志后端（ProjectFileLog）：Windows 上 spdlog 关闭时承载 `PROJECT_LOG_*` 落盘 `1q_library.log`；非 Windows 默认休眠（spdlog 分支优先），总开关关闭时宏回到空操作 |
| `ONEQ_ENABLE_SBIRS_ACCEPTANCE_LOG` | OFF | 开启 SBIRS 验收信息日志（`[SbirsAccept]` 事件流：WFOV 地面覆盖区/驻留时间、疑似目标与信号能量、宽窄切换连续命中、NFOV 捕获/跟踪、焦平面脱靶量、通道协同）；关闭时宏与派生计算一并剪除，零开销 |
| `ONEQ_ENABLE_RIR_ACCEPTANCE_LOG` | OFF | 开启远程识别雷达验收信息日志（`[RirAccept]` 事件流：检测链 SNR/SINR/Pd 与回波/噪声/干扰/杂波功率及处理增益、波位扫描序列、角距量测、跟踪滤波/关联状态、识别特征与结论、驻留调度统计）；宏基础设施已就绪，调用点按验收输出统计清单逐项接线；关闭时零开销 |
| `ONEQ_ENABLE_PRECISION_EVALUATION_LOG` | OFF | 开启精度评估日志（`[PrecisionEval]` 事件流：红外角度误差、双星交会位置误差、速度误差、落点/发射点预测误差样本与 AHP 综合评分，评估层 `precision_evaluation` 模块）；关闭时零开销 |

选项的最终值以所选 preset 与 configure 命令覆盖后的 CMake cache 为准。

## 支持边界

- macOS Conan 路径由当前 CI 覆盖 configure、build、install 和 consumer。
- Windows v141 preset（`VisualStudio.15.0-amd64`，Conan）已在 README 记载为 Windows 构建
  主线（库与 examples 可构建；示例集成日志在 Windows 走 `std::ofstream` 文件后端，
  `CA_LOG_BACKEND_SPDLOG=0`）。VS2015 与 no-Conan preset（`scripts/fetch_third_party.bat`）
  仍为未验收脚手架。CI 的 configure/build/install/consumer 验收仍只在 macOS 运行，
  因此按构建治理规则（`docs/practice/build_and_test_governance.md` CMake 工程边界 4）
  仍不构成"已支持 Windows"的正式声明。
- install tree 不是通用、完全自包含的二进制 SDK；跨机器分发前必须另行验证依赖闭包和运行时库。
