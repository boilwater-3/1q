# 示例程序（examples）

本目录是 **examples 层**：只使用 `include/` 对外公开的 Session / Adapter / Replay 接口，
不修改任何库源码。每个子目录按传感器域或工具职能切分，各自拥有独立的 `CMakeLists.txt`，
由顶层 `examples/CMakeLists.txt` 以 `add_subdirectory()` 编排——与 `src/` 的模块拆分惯例一致。

## 目录结构

```
examples/
├── CMakeLists.txt                  编排层：定义共享变量 + add_subdirectory()
├── README.md                       本文件
├── common/                         共享便利层：轻量 JSON 解析（不属于库 public surface）
├── configs/                        四域会话配置 JSON（详见 configs/README.md）
├── airborne_radar/                 AR 域示例（session / integration / scene / config_compare）
├── electro_optical/                EOS 域示例（session / integration / scene）
├── electronic_warfare/             ESR 域示例（session / integration / scene）
├── sar/                            SAR 域示例（session / integration）
├── behavior_layer/                 行为层参考实现（EnTT ECS 业务层，AR 单域全链）
├── flight_dynamic/                 机动模块 CSV 工具与轨迹生成（依赖 JSBSim）
└── batch_validation/               多场景批量验证（详见 batch_validation/README.md）
```

## 示例分类

每个传感器域（airborne_radar / electro_optical / electronic_warfare / sar）提供三类示例，
按由浅入深的顺序排列：

| 类别 | 程序命名 | 定位 | 配置来源 |
| --- | --- | --- | --- |
| **session_usage** | `<domain>_session_usage` | API 教程：Builder 构建配置 → 创建 Session → 多周期 StepWithResult | 代码内联 / config_loader |
| **integration_demo** | `<domain>_integration_demo` | 集成示范：展示 `XxxModule` 包装类在外部引擎中的接入方式 | config_loader |
| **scene** | `<domain>_scene` | 端到端场景：多目标对抗剧本 + 真实 session 跑多周期，模块级冒烟 | `examples/configs/*.json`（`SCENE_CONFIG_DIR`） |

其中 airborne_radar 额外提供 `ar_config_compare_test`（多份 JSON 配置加载一致性校验）。
SAR 当前无 scene 示例。

> `integration_demo` 展示的是 `XxxModule` 包装类（内部用普通 Session）；
> `scene` 与 `session_usage` 直接操作 Session / Adapter，二者定位不同。

## 共享便利层

`common/` 提供 `json_reader`（`oneq::JsonReader`），是 example 层的 JSON 解析便利工具：

- **不属于 oneq 库的 public surface**——库内部不消费 JSON。
- 各域示例通过 `config_loader.h` 调用它，将 JSON 树映射为 `*SessionConfig` 结构体。
- 由顶层 `CMakeLists.txt` 定义 `ONEQ_EXAMPLE_COMMON_DIR` / `ONEQ_EXAMPLE_COMMON_SOURCES`，
  各子目录通过目录作用域继承并内联到 target，无需函数传递。

## 构建与运行

示例默认不构建，需显式开启：

```bash
# 1. 标准依赖引导（详见 cmake/README.md）
bash scripts/bootstrap_conan.sh llvm-ninja-debug
cmake --preset llvm-ninja-debug -DENABLE_EXAMPLES=ON
cmake --build --preset llvm-ninja-debug --target airborne_radar_session_usage
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
| `SCENE_CONFIG_DIR` | `<domain>_scene` | 指向 `examples/configs/`，供 config_loader 加载 JSON |
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
- **系统**（每周期按 `recon → maneuver → jam → decision` 顺序执行，对齐 session
  `Step` 语义）：侦察（AR 会话输出适配为泛型探测记录并更新融合）、机动规划（长机
  调 `navigation` 面规划全员航路）、干扰（经 ECM 既有公共面构造周期输入）、决策
  （聚合产出命令帧，消费方读取后驱动 `SubmitExternalDecision` 等执行面）；
- **事件模型**：命令 = 写命令帧组件；事件报告 = `entt::observer` 响应组件变化，
  不建全局事件总线。

> **演进路线**：ECS 组件/系统模式覆盖了 session_usage（API 教程）与 scene（端到端
> 场景）类目的全部职责，将逐步取代现有 per-domain 示例；旧示例在迁移完成前保留，
> 本轮不迁移（决策记录见 `docs/review/Bahavior.md` 实施状态注记）。

## 相关文档

- [`configs/README.md`](configs/README.md) — 四域会话配置 JSON 字段说明
- [`batch_validation/README.md`](batch_validation/README.md) — 批量验证框架设计与 CSV schema
- [`cmake/README.md`](../cmake/README.md) — 构建架构与目录约定
- `docs/public_model_config_manual.md` — 配置字段详尽说明
