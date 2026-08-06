# 示例程序（examples）

本目录是 **examples 层**：只使用 `include/` 对外公开的 Session / Adapter / Replay 接口，
不修改任何库源码。每个子目录按传感器域或工具职能切分，各自拥有独立的 `CMakeLists.txt`，
由顶层 `examples/CMakeLists.txt` 以 `add_subdirectory()` 编排——与 `src/` 的模块拆分惯例一致。

## 目录结构

```
examples/
├── CMakeLists.txt                  编排层：定义共享变量 + add_subdirectory()
├── README.md                       本文件
├── common/                         共享便利层：JSON 解析 + 三域 config_loaders（不属于库 public surface）
├── configs/                        四域会话配置 JSON（详见 configs/README.md）
├── sar/                            SAR 域示例（session / integration）
├── behavior_layer/                 行为层参考实现（EnTT ECS 业务层，AR/ESR/EOS 三传感器全链）
├── component_attachment/           自定义实体-组件模式示例（组件基类 + 挂载 + Boost.Signals2 事件）
├── flight_dynamic/                 机动模块 CSV 工具与轨迹生成（依赖 JSBSim）
└── batch_validation/               多场景批量验证（详见 batch_validation/README.md）
```

## 示例分类

三域（AR/ESR/EOS）per-domain 示例已于 2026-08-05 删除——其 session_usage（API 教程）
与 scene（端到端场景）类目功能并入 `behavior_layer/` 三传感器全链（见下文）。
当前仅 SAR 保留 per-domain 示例形态：

| 类别 | 程序命名 | 定位 | 配置来源 |
| --- | --- | --- | --- |
| **session_usage** | `sar_session_usage` | API 教程：Builder 构建配置 → 创建 Session → 多周期 StepWithResult | 代码内联 / config_loader |
| **integration_demo** | `sar_integration_demo` | 集成示范：展示 `SarModule` 包装类在外部引擎中的接入方式 | config_loader |

SAR 无 scene 示例；`integration_demo` 展示的是 `SarModule` 包装类（内部用普通 Session）。

## 两种 ECS 开发模式

业务层示例提供两种 ECS 开发模式对照（均覆盖 AR/ESR/EOS 三传感器全链 +
融合 + 飞行动力学，共用同一份 `configs/` 共享配置）：

| 模式 | 目录 | ECS 形态 | 事件机制 |
| --- | --- | --- | --- |
| **ECS 开源库** | `behavior_layer/` | EnTT 3.14（纯数据组件 + 自由函数系统） | EnTT 自带 observer/sigh |
| **自定义实体-组件** | `component_attachment/` | 组件基类 + 子类（携带逻辑）+ 挂载到实体 | **Boost.Signals2**（常见开源事件库） |

`component_attachment/` 的自定义 ECS 核心（`core/`）约 300 行纯头文件：组件基类
（Name/OnAttach/OnDetach/Step 虚接口）、实体（挂载容器，挂载序 = 步进序）、
世界（实体注册表 + 共享场景状态 + 信号集合）。FD 场景按六自由度机动设计
（从起飞开始，`kTakeoff → 航点 → kLand`，不做空中配平），详见
`component_attachment/README.md`。

## 共享便利层

`common/` 提供 example 层共享便利工具（**不属于 oneq 库的 public surface**——库内部不消费 JSON）：

- `json_reader`（`oneq::JsonReader`）：轻量 JSON 解析；
- `config_loaders/<域>/`：各传感器域 JSON → `*SessionConfig` 的映射器（`config_loader.h` 三件套），
  供 `behavior_layer` 与 `batch_validation` 消费；
- 由顶层 `CMakeLists.txt` 定义 `ONEQ_EXAMPLE_COMMON_DIR` / `ONEQ_EXAMPLE_COMMON_SOURCES`，
  各子目录通过目录作用域继承并内联到 target，无需函数传递。

## 构建与运行

示例默认不构建，需显式开启：

```bash
# 1. 标准依赖引导（详见 cmake/README.md）
bash scripts/bootstrap_conan.sh llvm-ninja-debug
cmake --preset llvm-ninja-debug -DENABLE_EXAMPLES=ON
cmake --build --preset llvm-ninja-debug --target sar_session_usage
```

飞行力动示例额外需要机动模块：

```bash
cmake --preset llvm-ninja-debug -DENABLE_EXAMPLES=ON -DONEQ_ENABLE_FLIGHT_DYNAMIC=ON
```

构建产物统一输出到 `build/<preset>/bin/`。

## 配置注入约定

部分示例通过编译期宏注入运行时路径，避免硬编码：

| 宏 | 注入者 | 用途 |
| --- | --- | --- |
| `SCENE_CONFIG_DIR` | `behavior_layer_demo` | 指向 `examples/configs/`，供 config_loader 加载 JSON |
| `BATCH_CONFIG_DIR` | `*_batch_validation` | 同上，与 scene 同源 |
| `FD_JSBSIM_ROOT_DIR` | flight_dynamic 全部 | JSBSim 飞机数据根目录，优先取 `ONEQ_JSBSIM_DATA_ROOT_DIR` |

## flight_dynamic 示例

`flight_dynamic/` 提供 11 个独立 CSV 工具，覆盖机动模块的轨迹生成与质量分析：

- **起飞/降落**：`takeoff_land_csv`
- **机动扫描**：`maneuver_sweep_csv`
- **盘旋（Orbit）**：`orbit_quality_csv` / `orbit_trace_csv` / `orbit_long_duration`
- **跑道形（Racetrack）**：`racetrack_quality_csv` / `racetrack_trace_csv` / `racetrack_approach_diag` / `racetrack_approach_trace`
- **8 字形**：`figure8_approach_trace`
- **S 形转弯**：`sturn_trace_csv`

每个工具链接 `JSBSim::JSBSim`，通过 `setup_fd_example` 宏统一注入 src 头路径、
JSBSim 头路径与数据根目录。`orbit_visualize.py` 可将 trace CSV 可视化为 PNG + KML。

## 行为层参考实现（EnTT ECS）

`behavior_layer/` 是消费方业务层的参考实现：实体/组件装配由 EnTT（`entt/3.14.0`，
header-only，**example 侧依赖，不进入库本体**）registry 承担，逻辑以纯数据组件 +
自由函数系统表达（详见 `behavior_layer/README.md` 与 `docs/review/Bahavior.md` §5）。

- **组件**：`TaskingComponent`（角色/上下级/区域任务）、`SensorObservationComponent`、
  `FleetStatusComponent`、`RoutePlanComponent`、`FusedSituationComponent`、
  `CommandFrameComponent`；
- **系统**（每周期按 `flight → recon → maneuver → jam → decision` 顺序执行，对齐
  session `Step` 语义）：飞行（平台动力学——`ONEQ_ENABLE_FLIGHT_DYNAMIC=ON` 时经
  `flight_dynamic::FlightManager` 真实飞行仿真，关闭/数据缺失回退运动学）、侦察
  （**三传感器 AR/ESR/EOS 会话同周期推进**，输出在边界适配为泛型探测记录并经
  融合引擎跨源合并）、机动规划（长机调 `navigation` 面规划全员航路）、干扰
  （经 ECM 既有公共面构造周期输入，观测帧由 ESR 输出填充）、决策（聚合产出
  命令帧，消费方读取后驱动 `SubmitExternalDecision` 等执行面）；
- **事件模型**：命令 = 写命令帧组件；事件报告 = `entt::observer` 响应组件变化，
  不建全局事件总线。

> **演进路线（2026-08-05 已兑现）**：ECS 组件/系统模式覆盖了 session_usage 与 scene
> 类目的全部职责；三域（AR/ESR/EOS）per-domain 旧示例已删除，功能并入本示例
> （决策记录见 `docs/review/Bahavior.md` 实施状态注记）。

## 相关文档

- [`configs/README.md`](configs/README.md) — 四域会话配置 JSON 字段说明
- [`batch_validation/README.md`](batch_validation/README.md) — 批量验证框架设计与 CSV schema
- [`cmake/README.md`](../cmake/README.md) — 构建架构与目录约定
- `docs/public_model_config_manual.md` — 配置字段详尽说明
