---
Status: draft
Date: 2026-08-05
Authority: 行为组件层（决策 / 机动 / 侦察 / 干扰）分解设计的决策记录。非规范性审查记录；
不得替代 `docs/common/contract.md`、`docs/common/session_contract.md` 及各模块
`docs/<module>/design.md`。若本文与库实现冲突，以库为准。
---

# 行为组件层设计：库内算法面 + 消费方业务层

> **实施状态（2026-08-05）**：§3/§4 两个算法面骨架已落地为新模块 `navigation`、`fusion`
> （含单元测试），以 `docs/navigation/`、`docs/fusion/` 模块文档为当前实现权威；
> 本文仍为决策记录，不替代模块文档。§5 example（EnTT 业务层）已落地为
> `examples/behavior_layer/`（`entt/3.14.0` 为消费方侧依赖，单域 AR 端到端参考实现，
> 含依赖链冒烟测试），详见 `examples/behavior_layer/README.md`。
> 演进路线：ECS 组件/系统模式将逐步取代现有 session_usage/scene 类示例，
> 旧示例在迁移完成前保留（见 examples/README.md）。

## 0. 定位与结论

行为组件层是**实体决策层**：装配在实体上，消费传感器 session 的去真值化输出，向执行
模块（flight_dynamic / ECM / AR seam）下发决策。它本质是业务编排，**整体不入库**，
以 example 参考实现承载；库内只吸收其中两个**跨业务可复用**的算法面：

| 算法面 | 归属 | 形态 |
|---|---|---|
| 路径规划面（区域覆盖路径生成） | 新模块 `navigation` | 不绑定 flight_dynamic，输出中性航点类型 |
| 多源融合面（关联 + 置信度融合） | 新模块 `fusion` | 泛型探测记录，不感知传感器类型 |

## 1. 背景（原始需求）

原始设计（本文件早期草稿）：行为组件为独立组件，在实体-组件模式下装配在实体上，核心为
实体的决策组件；决策组件持有抽象行为组件指针集合（多态），行为组件解析/处理信息后把结果
发送给决策组件进行二次处理（发送事件等）。三类行为组件：

- **机动行为组件**：路径规划层提供行为模式对应的飞行路径计算能力；区域数据（多边形/圆形，
  经纬高）在外部解析后做区域覆盖路径生成。面向单机与编队两模式：无上级且无下级 → 单机
  只算自己的巡逻航路；无上级但有下级 → 长机规划全体僚机与自己的航路；有上级 → 不计算。
- **侦察行为组件**：对电子侦察（ESR）、光学（EOS）、SAR 三类探测结果以平台标识为主关联键
  （无标识时以地理位置相邻门限为辅助）聚合为融合目标态势；以判决值（0/1）与探测质量归一化
  值为输入按权重计算综合置信度；输出融合目标态势列表作为综合侦察报告的核心数据来源。
  关键问题：大场景融合成本；报告产生周期的最优解。
- **干扰行为组件**：原文档为空。

## 2. 分解原则：算法与业务的边界

按**跨业务可复用性**划线——与具体任务/平台/编队/传感器耦合的属于业务，可复用算法入库：

| 能力 | 是否耦合业务 | 归属 |
|---|---|---|
| 区域覆盖路径生成（多边形/圆形 LLA → 航路点集） | 否：输入区域+覆盖参数，输出航路 | **库内 `navigation`** |
| 多源关联融合（异构探测 → 融合态势） | 否：泛型探测记录，不感知传感器类型 | **库内 `fusion`** |
| 行为组件/决策组件（EnTT 数据组件 + 系统） | 是：ECS 装配、角色判定、事件语义 | example（EnTT） |
| 编队编排（长机规划 → 分发 → N 个 FlightManager） | 是 | example |
| 报告节奏策略、干扰任务分配策略 | 是 | example |
| 与各 session 的接线（输出 → 输入帧 → 决策 → seam） | 是 | example |

先例：`examples/electronic_warfare/`（EsrModule、integration_demo、config_loader）已是
"跨模块业务级装配放 example、库内只留算法/模型"的现成形态。

## 3. 库内算法面一：路径规划面（新模块 `navigation`）

### 3.1 不绑定 flight_dynamic 的理由（冻结）

原方案曾提议挂载于 `flight_dynamic/guidance/AreaCoveragePlanner`，**否决**，证据：

1. `cmake/project/ProjectOptions.cmake:76`：`ONEQ_ENABLE_FLIGHT_DYNAMIC` 默认 **OFF**，
   绑定后路径规划面在默认构建中不可用；
2. 消费方很大概率使用路径规划面但不走本库机动（自有机动/航迹实现），绑定会强制依赖。

冻结决策：路径规划面是**独立中立算法面**，输出自有中性航点类型，不引用
`flight_dynamic` 的任何类型；由业务层适配到 `flight_dynamic::Waypoint` 或消费方自有类型。

### 3.2 公共类型与 API（首期最小面）

```
namespace navigation {
  // 覆盖区域：多边形（LLA 顶点序列，度制）或圆形（圆心 LLA + 半径）
  struct CoverageArea {  // 内部为 variant：多边形 | 圆形
    ...
  };
  // 覆盖参数：扫描航向、扫描间距、高度、速度、模式（扫描 | 盘旋）
  struct CoveragePlanConfig { ... };
  // 中性航点：LLA + 速度 + 到达半径；语义对齐 coordinate 域度制惯例
  struct RoutePoint {
    double latitude_deg; longitude_deg; altitude_m; speed_mps; radius_m; ...
  };
  using RoutePlan = std::vector<RoutePoint>;

  class AreaCoveragePlanner {
    RoutePlan Plan(const CoverageArea& area, const CoveragePlanConfig& cfg) const;
  };
}
```

- 首期能力：多边形扫描线（boustrophedon）覆盖、圆形单环/同心圆盘旋；8 字/S 型等规划
  模式不在首期（执行侧已有 `ManeuverExecutor`，规划侧 YAGNI）。
- 单位：中性类型跟随 `coordinate` 域惯例（度制，`LlaPositionDegM`）；flight_dynamic 的
  `Waypoint` 为弧度制，单位转换属业务层适配职责。
- 形态：新模块 `include/1q/navigation/` + `src/navigation/`，与 flight_dynamic 同形
  （无 Session 三元组）；不引入新依赖（Eigen/nanoflann 已在依赖清单），**无构建门**。

## 4. 库内算法面二：多源融合面（新模块 `fusion`）

### 4.1 关联键策略（冻结）

原文档"平台标识为主关联键"与本库**去真值化纪律**直接冲突：ESR hypothesis 刻意不含场景
真值标识（`EmitterHypothesis.h`），EOS 输出记录无 target_id/confidence（attribution 仅
存在于 debug 路径）。冻结决策：**纯库内身份 + 特征/空间关联，不引入外部身份通道**。

- 关联键 = 调用方提供的库内身份键（如 AR `association_key`、ESR `hypothesis_id`；
  跨源一致性由调用方保证）+ 特征相似度门限 + 空间门限；
- 空间门限分层：带位置记录 → nanoflann KD-tree 半径搜索；仅方位记录 →
  `src/common/geometry/BearingCluster.h` 方位相干门限（先例：ESR 内部
  `KdTreeClusterer` 的 nanoflann 用法）；
- 大场景成本：身份键哈希 + KD-tree 空间门控 + 特征门限，每周期 O(N log N + M)，
  增量融合 + 滑窗，避免全量两两比对。

### 4.2 置信度与输出

- 融合置信度 = Σ 判决值(0/1) × 质量归一化 × 权重；权重/门限/窗口配置化；
- 探测记录为**泛型**（位置/方位/特征向量/判决值/质量/库内 key），算法不感知
  ESR/EOS/AR 具体类型，由业务层适配；
- 输出：融合目标态势记录（库内 key、各源探测状态、融合置信度、各通道量测）；
- 报告节奏：**无普适最优周期**（原文档问题），冻结为配置化周期 + 事件触发
  （新目标出现/置信度跃迁/目标消失），与融合窗口解耦。

### 4.3 SAR 输入的现实约束

库内 SAR 无探测/spot-report 输出（`SarOutputFrame` 仅为图像质量元数据）。原文档
"三类传感器探测结果列表"中 SAR 项以库为准：首期 SAR 不作为融合输入，可作为使命状态
（图像质量/SNR）旁路信息；SAR 探测能力属未来扩展。

### 4.4 形态

新模块 `include/1q/fusion/` + `src/fusion/`，无 Session；可复用 `src/common/estimation/`
滤波器做轨迹滤波；不引入新依赖，无构建门。

## 5. 消费方业务层（`examples/behavior_layer/` 参考实现，EnTT 驱动）

与 `examples/electronic_warfare/` 平级的新示例目录，承载实体-组件模式的完整业务形态。
**ECS 选型（冻结）：引入 EnTT（header-only 轻量级 ECS，主流 C++17 实现，适配仿真项目），
实体/组件装配由 `entt::registry` 承担，不手工搭建 ECS。**

EnTT 为 data-oriented 模型：组件是**纯数据**（无虚函数），逻辑是**系统**（自由函数，
`entt::registry&`）。由此文档原"决策组件持有 `unique_ptr<IBehaviorComponent>` 多态
集合"的形态作废，演化为数据组件 + 系统——组件间无反向引用、每周期单向帧交换的纪律
反而更契合：

```
examples/behavior_layer/
  components.h              // EnTT 数据组件（纯数据）：
                            //   TaskingComponent            — own_role/superior/subordinates/区域任务
                            //   SensorObservationComponent  — 传感器输出适配后的泛型探测记录
                            //   FleetStatusComponent        — 编队状态（消费方聚合注入）
                            //   RoutePlanComponent          — 航路计划（长机产出）
                            //   FusedSituationComponent     — 融合态势报告
                            //   CommandFrameComponent       — DecisionFrame 命令输出
  systems.h/.cpp            // 系统（自由函数，entt::registry&）：
                            //   maneuver_system / recon_system / jam_system / decision_system
  assembly.h/.cpp           // 装配：registry 接线各 session 输出/输入
  behavior_layer_demo.cpp   // 主程序：registry + 每周期系统调用序
```

- 每周期推进 = 按序调用系统（对齐 session 的 Step 语义）；决策系统聚合各组件结果，写入
  `CommandFrameComponent`；消费方读取命令帧并驱动 `SubmitExternalDecision` /
  `EcmCycleInput` / `FlightManager`（先例：`ArCommand`——"行为决策层下发的战术指令"）。
- **层级来源（冻结）**：确定性仿真不存在"发现"机制，"有上级/有下级"必须来自
  `TaskingComponent` 显式输入；角色判定为纯函数：有上级 → passive 零计算；无上级
  有下级 → 长机规划全员航路；否则单机。
- **编队编排**：长机系统经 `registry.view<...>()` 查询僚机实体，调用 `navigation` 面后
  写入各机 `RoutePlanComponent`；分发与驱动 FlightManager 仍是消费方职责。
- **事件模型（冻结）**：不建全局消息总线；命令 = `CommandFrameComponent` 组件写入，
  事件驱动报告（新目标出现/置信度跃迁/目标消失）用 `entt::observer` 观察组件变化触发。
- **与 AR 交互**：走既有 public seam（`SubmitExternalDecision` / `decision_observation`，
  session_contract.md §Session composition ownership），不注入或替换 session 内部对象。
- **与 ECM 交互（冻结）**：干扰系统只在 ECM 现有公共面内工作——构造
  `EcmCycleInput::sensor_observation_frame` + runtime patch 默认技术；**不做逐威胁
  tasking SPI**（ECM design.md 明确不公开 planner SPI）。逐威胁任务分配列为已知限制。
- **报告节奏**：侦察报告按配置化周期 + `entt::observer` 事件触发产出，作为综合侦察
  报告数据来源。

**依赖边界（冻结）**：EnTT 仅作为 example/consumer 侧依赖引入（`conanfile.py`
`requirements()` 增加 `entt/3.14.0`，header-only，无链接形态问题）；**不进入库本体**
——`include/1q` 公共头与库目标保持 EnTT 无关，C++11 floor 与库依赖面不变。
`navigation`/`fusion` 算法面同样保持 EnTT 无关（纯算法，由系统调用）。

## 6. 冻结决策表（证据矩阵）

| 冻结项 | 决策 | 关键证据 |
|---|---|---|
| 行为层整体定位 | **narrow**：业务面入 example；两个算法面入库 | `examples/electronic_warfare/` 先例；AGENTS.md 库定位 |
| 路径规划面归属 | **pass**：新模块 `navigation`，不绑定 flight_dynamic | `ProjectOptions.cmake:76` 门控默认 OFF；消费方可能自有机动 |
| 侦察关联键 | **pass**：纯库内身份 + 特征/空间关联，无外部身份通道 | ESR/EOS 输出去真值化纪律 |
| 组件组合方式 | **修订**：EnTT 数据组件 + 系统（多态 `unique_ptr` 集合方案作废） | EnTT data-oriented 惯例；帧交换纪律更契合 |
| 层级来源 | **reject**"发现"机制 → tasking 显式输入 | 确定性仿真无发现机制；flight_dynamic 无编队概念 |
| ECS 选型 | **pass**：EnTT（header-only，C++17，主流轻量级 ECS），仅 example/consumer 侧依赖 | 仿真项目适配性；库本体依赖面与 C++11 floor 不变 |
| ECM tasking SPI | **消解**：业务层在 example，现有公共面即可闭环 | ECM design.md"不公开 planner SPI" |
| 空间索引 | **narrow**：融合面内建 nanoflann KD-tree | ESR `KdTreeClusterer` 内部先例 |
| 报告周期 | **reject**单一最优 → 配置化 + 事件触发 | 无证据支持任何单一值 |
| 事件模型 | **pass**：命令帧组件写入 + `entt::observer` 事件触发，不建全局总线 | `ArCommand` 先例；EnTT observer |

## 7. 非目标与已知限制

- 不做实体管理器/ECS 容器入库（实体装配由消费方 EnTT registry 承担，EnTT 不进库本体）；
- 不引入外部身份通道（守去真值化纪律）；
- 不做 ECM 逐威胁 tasking SPI（首期；干扰行为只能用观测帧内容与默认技术间接影响调度）；
- 不建事件总线；
- 首期路径规划仅区域覆盖（扫描/盘旋），8 字/S 型等规划模式不做；
- SAR 首期不作为融合输入（库内无探测输出）；
- 编队编排由业务层负责，flight_dynamic 保持单机定位不变。

## 8. 未来变更触发条件（evidence-first）

- **业务层提升**：若出现第二个消费方需要行为组件/决策组件，提升为正式模块前须重走
  证据矩阵，并先冻结 `docs/common/contract.md` 的 SPI 唯一性条款变更；
- **EnTT 回退**：若 EnTT 与项目构建（C++ 标准/Conan 版本）出现兼容问题，回退为手工
  装配方案，并同步修订本文件的 ECS 选型冻结项；
- **算法面撤回**：若 `navigation`/`fusion` 无第二个消费方复用，应从库内撤回 example
  （入库非单向门）；
- **ECM tasking**：若业务需求证明逐威胁任务分配必要，冻结新 SPI 须走 ECM design.md
  变更规则（§5：technique/资源/热语义或快照所有权变化须同步单元/trace/replay/跨域测试）；
- **SAR 探测**：若需 SAR 探测级输入，先在 SAR 侧新增探测能力（独立工程，另行冻结）。
