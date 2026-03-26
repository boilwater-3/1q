# 1q

`1q` 是一个面向外部服务模块的雷达仿真模型库，当前包含机载雷达和电子侦察雷达两套主模块。项目重点在于稳定的公共 API、可替换的内部组件，以及可测试的仿真链路编排。

## 模块概览

- `include/1q/airborne_radar/`: 机载雷达公共 API。
- `include/1q/electronic_surveillance_radar/`: 电子侦察雷达公共 API。
- `src/airborne_radar/`: 机载雷达实现，覆盖环境、决策、信号处理、跟踪与会话编排。
- `src/electronic_surveillance_radar/`: ESR 实现，覆盖环境、截获与流水线编排。
- `tests/`: 单元测试、集成测试、安装消费测试。
- `examples/`: 从快速上手到高级注入、可视化的示例程序。

## 依赖

- C++11
- CMake
- Conan
- GTest / GMock
- Eigen
- Sophus
- nanoflann
- eventpp
- spdlog

## 构建

仓库约定优先使用 CMake preset。常用 preset：

- `llvm-ninja-debug-local`
- `llvm-ninja-release-local`
- `llvm-ninja-release-local-stress`

示例命令：

```bash
cmake --preset llvm-ninja-debug-local
cmake --build --preset llvm-ninja-debug-local
ctest --preset llvm-ninja-debug-local --output-on-failure
```

同一个 preset 下应串行执行 `configure -> build -> test`。

## 示例

示例代码位于 `examples/`：

- `example_quick_start.cpp`: 最小接入示例。
- `example_radar_session.cpp`: 会话接口与一帧驱动示例。
- `example_advanced_injection.cpp`: 高级注入与定制装配示例。
- `example_*_visualizer.cpp`: 检测距离、战术模式、多目标、航迹和 ECM 场景可视化示例。

## 文档

- `doc/architecture/`: 架构与模块设计文档。
- `doc/electronic_surveillance_radar/`: ESR 设计与说明文档。
- `doc/reference/`: 参考文档。

## 测试

测试覆盖关联、跟踪、环境、决策、会话编排和安装消费路径。修改公共 API 或关键逻辑时，应同步补充或更新 `tests/` 下的测试。

## 约束

- 不引入 C++ exceptions。
- 高速仿真/数学热点路径不打日志。
- 优先在 `src/` 内部收敛改动，再考虑扩大公共头暴露面。
