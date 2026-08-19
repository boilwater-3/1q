# 示例程序（examples）

本目录是 **examples 层**：消费方集成参考——示范"怎么写一个接库的程序"。
只使用 `include/` 对外公开的 Session / Adapter / Replay 接口，不修改任何库源码。

## 快速开始

```bash
# 1. 依赖引导 + configure（示例默认不构建，需显式开启；FD 真实飞行仿真可选）
bash scripts/bootstrap_conan.sh llvm-ninja-release-local
cmake --preset llvm-ninja-release-local -DENABLE_EXAMPLES=ON [-DONEQ_ENABLE_FLIGHT_DYNAMIC=ON]

# 2. 构建 demo
cmake --build --preset llvm-ninja-release-local --target component_attachment_demo

# 3. 运行基线场景（400 周期，产物落 log/）
./build/llvm-ninja-release-local/bin/component_attachment_demo
```

想跑别的场景：`--scene examples/component_attachment/scenes/<name>/<name>.json`。
改场景不用重编译（场景是 JSON 数据驱动）。详见
[`component_attachment/README.md`](component_attachment/README.md)。

## 定位：单一角色

examples 层只承担一个角色：**消费方集成参考**。验证框架与模块开发期工具
不属于"示例"，已归位 `tests/`：

| 角色 | 位置 |
| --- | --- |
| 消费方集成参考 | `examples/`（本目录） |
| 多场景批量验证框架 | `tests/consumer/batch_validation/`（只走公开 Session/Adapter/Replay 接口） |
| FD 开发期验证工具（逐帧 CSV + 分析脚本） | `tests/unit/flight_dynamic/fd_tools/`（机动模块验证形态 = 帧导出 + 脚本分析，见该目录 README） |

## 目录结构

```
examples/
├── CMakeLists.txt                  编排层：定义共享变量 + add_subdirectory()
├── README.md                       本文件
├── common/                         共享便利层：JSON 解析 + 五域 config_loaders + viz/ 共享查看器（不属于库 public surface）
├── configs/                        五域会话配置 JSON + remote_identification_radar/ 识别数据库（详见 configs/README.md）
├── component_attachment/           消费方集成参考示例：五传感器 + 融合 + 威胁评估 + 多机编队
└── precision_evaluation/           评估层集成参考示例：双星定位精度评估（五项误差 + AHP 综合评分）
```

## 示例内容

`component_attachment/` 为消费方业务层参考实现：自定义实体-组件模式
（组件基类 + 挂载 + Boost.Signals2 事件），每周期按挂载序推进，覆盖
飞行（FD 六自由度机动 + 运动学回退）→ 五传感器（AR/ESR/EOS/SBIRS/SAR）
→ 融合 → 威胁评估 → 决策指令的事件链，并提供：

- **场景数据驱动**：`scenes/<name>/<name>.json` 声明飞行脚本、目标脚本、传感器业务覆写
  与冒烟下限（含多机区域巡逻 `fleet_patrol_multi_zone`），新场景只加子目录
  不改代码；场景预期表归档为同名 `.md`；
- **集成端日志示范**：库日志（`1q_library.log`）+ 集成端事件/视图日志
  （`integration_events.log` / `integration_views.log`，中文人读行），落盘
  密度由 `logger/logger_modes.h` 三模式宏门控；
- **外置查询数据源**：各组件暴露 `powered_on()` / 扫描方位 getter 与
  `LastDebugView()`，供选定实体后拉取设备状态；
- **运行时修改接口**：各组件把库的 RuntimeConfigPatch 薄包装为公开方法
  （提交语义随模块而异，权威定义见 `docs/common/contract.md`）。

详见 [`component_attachment/README.md`](component_attachment/README.md)。

`precision_evaluation/` 为评估层集成参考（需求 3.2.1.6.3 定位精度分析）：
硬编码双星几何（两颗卫星各测一条视线）+ 双目标弹道真值，每周期推进真值并驱动
`PrecisionEvaluationSession`（内部自持双星 SBIRS + 融合逐航迹滤波 + 弹道推演），
结束打印五项误差统计（红外测角 / 双星交会位置 / 速度 / 落点 / 发射点）与
AHP 加权综合得分，并做"五指标均有样本 + AHP 合法 + 综合分 ∈ (0,1]"自检：

```bash
cmake --build --preset llvm-ninja-release-local --target precision_evaluation_demo
./build/llvm-ninja-release-local/bin/precision_evaluation_demo [--cycles 60]
# 逐周期 [PrecisionEval] 事件流：configure 时加 -DONEQ_ENABLE_PRECISION_EVALUATION_LOG=ON
```

## 共享便利层

`common/` 提供 example 层共享便利工具（**不属于 oneq 库的 public surface**
——库内部不消费 JSON）：

- `json_reader`（`oneq::JsonReader`）：轻量 JSON 解析；
- `csv_writer`：流式 CSV 写入（批量验证框架与 component_attachment 输出共用）；
- `config_loaders/<域>/`：各传感器域 JSON → `*SessionConfig` 的映射器
  （`config_loader.h` 三件套），component_attachment 与批量验证框架消费；
- `viz/`：共享可视化查看器 `build_viewer.py`（统一契约 v2：多机
  platform_track/route_plan + zones），从 CSV 构建单文件交互 HTML；
- 由本层 `CMakeLists.txt` 定义 `ONEQ_EXAMPLE_COMMON_DIR` /
  `ONEQ_EXAMPLE_COMMON_SOURCES`，子目录通过目录作用域继承并内联到 target。

## 配置注入约定

| 宏 | 注入者 | 用途 |
| --- | --- | --- |
| `SCENE_CONFIG_DIR` | component_attachment demo 与单元测试 | 指向 `examples/configs/`，供 config_loader 加载 JSON |
| `CA_SCENE_DIR` | component_attachment demo | 指向 `examples/component_attachment/scenes/`，`--scene` 默认值 |
| `CA_DEFAULT_OUTPUT_DIR` | component_attachment demo | 指向 `examples/component_attachment/log/`，`--output-dir` 默认值（运行时产物不入版本控制） |

## 相关文档

- [`configs/README.md`](configs/README.md) — 五域会话配置 JSON 字段说明 + remote_identification_radar/ 识别数据库
- [`component_attachment/README.md`](component_attachment/README.md) — 示例详细设计
- [`component_attachment/scenes/README.md`](component_attachment/scenes/README.md) — 场景系统（JSON schema + 场景集 + 六自由度/巡逻/多机/天基设计）
- [`component_attachment/logger/README.md`](component_attachment/logger/README.md) — 集成端日志设施（三模式宏门控）
- [`cmake/README.md`](../cmake/README.md) — 构建架构与目录约定
- [`docs/common/usage.md`](../docs/common/usage.md) — 1q 库消费指南
- [`docs/common/session_contract.md`](../docs/common/session_contract.md) — 会话相关模块契约
- `docs/practice/batch_validation.md` — 批量验证框架（tests/consumer 分区）
- `tests/unit/flight_dynamic/fd_tools/README.md` — FD 开发期验证工具
