# 行为层参考实现（`examples/behavior_layer/`）

消费方业务层的 EnTT ECS 参考实现（冻结契约：`docs/review/Bahavior.md` §5）。
实体/组件装配由 `entt::registry` 承担，逻辑以纯数据组件 + 自由函数系统表达；
**EnTT 仅为 example 侧依赖**（`conanfile.py` 基础清单，header-only），不进入库本体，
`include/1q/` 公共头与 C++11 下限均不受影响。

## 目录结构

| 文件 | 职责 |
| --- | --- |
| `components.h` | 六个数据组件（纯数据、无虚函数） |
| `systems.h` / `systems.cpp` | 四个系统（自由函数，`entt::registry&`） |
| `assembly.h` / `assembly.cpp` | 装配：registry ctx 上下文、实体创建、周期调用序、观察者工厂 |
| `behavior_layer_demo.cpp` | 主程序：脚本化场景 + 每周期系统调用序 + 事件报告 |

## 组件（`components.h`）

| 组件 | 内容 | 写入方 |
| --- | --- | --- |
| `TaskingComponent` | 角色（单机/长机/僚机）、上下级、区域任务（`navigation` 面类型） | 装配层（层级显式注入，无"发现"机制） |
| `SensorObservationComponent` | `source_id` + 泛型探测记录（`fusion::DetectionRecord`） | `recon_system` |
| `FleetStatusComponent` | 平台 LLA 位置/航向/速度 | 消费方聚合注入（demo 脚本） |
| `RoutePlanComponent` | 航路计划（`navigation::RoutePlan`）+ 版本号 | `maneuver_system` |
| `FusedSituationComponent` | 融合态势 + 新/消失事件计数 | `recon_system`（经 `fusion` 引擎） |
| `CommandFrameComponent` | AR 战术指令 / ECM 周期输入 / 外部决策覆盖 | `jam_system` + `decision_system` |

## 系统与周期调用序

每周期由 `StepBehaviorLayer()` 按以下顺序执行（对齐 session `Step` 语义）：

1. **`recon_system`** — 驱动 AR 会话（`StepWithResult`），用库内
   `ArCycleOutputAdapter` 把轨迹输出转为 ECEF，再适配为 `fusion::DetectionRecord`
   （key = `association_key`），调用 `FusionEngine::Update` 更新融合态势；
2. **`maneuver_system`** — 层级纯函数：有上级 → 零计算；长机/单机调
   `AreaCoveragePlanner::Plan` 规划区域覆盖航路；长机把计划下发到各僚机
   （编队偏移与航段驱动属消费方职责）；
3. **`jam_system`** — 从编队状态构造 `EcmCycleInput` 骨架（仅 ECM 既有公共面，
   无逐威胁 tasking SPI）；ESR 会话接入后由其输出填充 `sensor_observation_frame`；
4. **`decision_system`** — 聚合融合态势（高置信威胁判定）产出 `ArCommand`，
   写入命令帧组件；`external_decision` 为预留接线位。

事件模型：命令 = 写 `CommandFrameComponent`；事件报告 = `entt::observer`
（`MakeSituationObserver`，监听 `FusedSituationComponent` 变化），不建全局事件总线。

## 接线位（执行面驱动）

| 执行面 | 接线 | 状态 |
| --- | --- | --- |
| AR | `ArSession::SubmitExternalDecision` / `decision_observation` | 已演示（命令帧输出；外部覆盖为预留位） |
| ECM | `EcmCycleInput`（`sensor_observation_frame` + 默认技术） | 帧骨架已构造，ESR 观测待接入 |
| navigation | `AreaCoveragePlanner::Plan` → `RoutePlanComponent` | 已演示 |
| fusion | `FusionEngine::Update` → `FusedSituationComponent` | 已演示 |
| flight_dynamic | 消费方将 `RoutePlanComponent` 适配为 `FlightManager` 指令 | 未接入（模块默认 OFF，驱动属消费方职责） |
| ESR / EOS | 会话输出 → 新增 `SensorObservationComponent` 源通道 | 未接入（`source_id` 维度已就绪） |

## 构建与运行

```bash
bash scripts/bootstrap_conan.sh llvm-ninja-release-local   # 拉取 entt/3.14.0
cmake --preset llvm-ninja-release-local -DENABLE_EXAMPLES=ON
cmake --build --preset llvm-ninja-release-local --target behavior_layer_demo
./build/llvm-ninja-release-local/bin/behavior_layer_demo
```

启用 `BUILD_TESTING` 时注册冒烟测试 `examples::behavior_layer_demo`
（LABELS：`examples;behavior_layer`），验证 EnTT 依赖链与端到端全链。

## 演进路线

ECS 组件/系统模式覆盖了 session_usage（API 教程）与 scene（端到端场景）类目的
职责，将逐步取代现有 per-domain 示例；旧示例在迁移完成前保留，本轮不迁移。
