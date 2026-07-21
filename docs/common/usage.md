# 1q 库消费指南

Status: active
Last-reviewed: 2026-07-21
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

选项的最终值以所选 preset 与 configure 命令覆盖后的 CMake cache 为准。

## 支持边界

- macOS Conan 路径由当前 CI 覆盖 configure、build、install 和 consumer。
- Windows presets 与 `scripts/fetch_third_party.bat` 已存在，但尚未完成 contract 要求的真实 Windows
  全链验收，因此不构成正式支持声明。
- install tree 不是通用、完全自包含的二进制 SDK；跨机器分发前必须另行验证依赖闭包和运行时库。
