# 1q

`1q` 是一个面向外部服务模块的仿真模型库，当前覆盖机载雷达（AR）、电子侦察雷达（ESR）、合成孔径雷达（SAR）、光电传感器（EOS）、飞行动力学（Flight Dynamic）以及天基红外传感器（SBIRS）六套主模块。项目重点在于稳定的公共 API、可替换的内部组件，以及可测试的仿真链路编排。

## 模块概览

- `include/1q/airborne_radar/`、`src/airborne_radar/`: 机载雷达公共 API 与实现，覆盖环境、决策、信号处理、跟踪与会话编排。
- `include/1q/electronic_surveillance_radar/`、`src/electronic_surveillance_radar/`: ESR 公共 API 与实现，覆盖环境、截获与流水线编排。
- `include/1q/sar/`、`src/sar/`: 合成孔径雷达公共 API 与实现。
- `include/1q/electro_optical_sensor/`、`src/electro_optical_sensor/`: 光电传感器公共 API 与实现。
- `include/1q/flight_dynamic/`、`src/flight_dynamic/`: 飞行动力学、制导与机动模型。
- `include/1q/sbirs_sensor/`、`src/sbirs_sensor/`: 天基红外传感器（SBIRS）公共 API 与实现，覆盖环境、错误模型、NFOV 调度与处理流水线。
- `include/1q/{coordinate,environment,foundation,replay,trace}/`: 跨模块共享的坐标、环境、基础类型、回放与追踪接口。
- `tests/`: 单元测试、集成测试、契约测试、性能测试与安装消费测试。
- `examples/`: 各模块的快速上手、会话用法与集成示例。

## 依赖

- C++17（CMake 默认 `PROJECT_DEFAULT_CXX_STANDARD = 17`，最低要求 C++11）
- CMake / Conan
- GTest / GMock（测试）
- Eigen、nanoflann、Boost
- FlatBuffers、zlib
- spdlog / fmt（日志依赖）
- JSBSim、HighFive（可选，由 Conan 选项控制）

## 构建

仓库约定优先使用 CMake preset。本地开发常用 preset（定义于 `CMakeUserPresets.json`）：

- `llvm-ninja-debug-local`
- `llvm-ninja-release-local`

CI preset（定义于 `CMakePresets.json`，当前支持 macOS / LLVM + Ninja）：

- `llvm-ninja-debug`、`llvm-ninja-release`（macOS / LLVM + Ninja）

示例命令：

```bash
bash scripts/bootstrap_conan.sh llvm-ninja-debug-local
cmake --preset llvm-ninja-debug-local
cmake --build --preset llvm-ninja-debug-local
ctest --preset llvm-ninja-debug-local --output-on-failure
```

同一个 preset 下应串行执行 `bootstrap -> configure -> build -> test`。
`bootstrap_conan.sh` 负责按 preset 派生参数并执行 `conan install`，生成
`conan_toolchain.cmake` 供后续配置使用。

## 示例

`examples/` 是**消费方集成参考**（怎么写一个接库的程序，只使用 `include/` 公开接口）：

- `examples/component_attachment/`: 消费方集成参考示例——五传感器（AR/ESR/EOS/
  SBIRS/SAR）+ 融合 + 威胁评估 + 多机编队，场景 JSON 数据驱动（含多机区域巡逻
  fleet_patrol_multi_zone 场景）。
- `examples/common/`: example 层共享便利层（`json_reader` + 五域 `config_loaders/` +
  `viz/` 共享可视化查看器，不属于库 public surface）。
- `examples/configs/`: 跨模块共享的配置样例。

验证/开发期工具已迁出 examples（角色分离）：多场景批量验证框架在
`tests/consumer/batch_validation/`，飞行力学开发期轨迹工具在
`tests/unit/flight_dynamic/fd_tools/`。

## 文档

- `CLAUDE.md`: 工程约束、构建测试规则与重构策略。
- `docs/<module>/design.md`: 各模块当前设计（AR / ESR / SAR / EOS / Flight Dynamic / SBIRS）。
- `docs/common/`: 跨模块契约与开放问题（`contract.md`、`open_questions.md`）。
- `docs/practice/`: 工程实践与基础设施类设计文档（覆盖率、批量场景验证框架等）。
- `docs/review/`: 模块评审与迁移计划。
- `include/1q/README.md`: 公共头文件导航与对外接入建议。
- `tests/README.md`: 测试分层约定与运行建议。

## 测试

测试覆盖关联、跟踪、环境、决策、会话编排、契约、性能和安装消费路径（见 `tests/{unit,integration,contract,performance,consumer}/`）。修改公共 API 或关键逻辑时，应同步补充或更新 `tests/` 下的测试。

## 约束

- 不引入 C++ exceptions。
- 高速仿真/数学热点路径不打日志。
- 优先在 `src/` 内部收敛改动，再考虑扩大公共头暴露面。
