# 自定义实体-组件示例（component_attachment）

## 定位

第二种示例模式：**自定义实体-组件**开发（不依赖 EnTT 等 ECS 开源库），与
`behavior_layer`（EnTT ECS 开源库模式）形成两种开发模式对照。

- **组件基类**：`core/component.h` 定义虚接口（Name / OnAttach / OnDetach / Step），
  组件**携带逻辑**（区别于 EnTT 纯数据组件）；
- **模块 → 组件**：每个仿真模块对应一个 `Component` 子类（飞行 / AR / ESR / EOS /
  SBIRS / SAR / 融合 / 威胁评估），**挂载**到实体上参与仿真；
- **事件机制**：使用 C++ 常见开源事件库 **Boost.Signals2**（零自定义分发层），
  弥补"自研 ECS 无事件功能"（对应 EnTT 的 observer/signals）。

单平台精简场景：1 个 `platform` 实体挂载 8 个组件，400 周期 × 1 s。四传感器
（AR / ESR / EOS / SBIRS）端到端：探测 → 融合 → 威胁评估 → 高置信威胁 → 决策指令（事件链）；
SAR 为图像产品通道（无探测输出，不入融合，发布产品生命周期事件）。

## 目录结构

```
examples/component_attachment/
├── core/                            自定义 ECS 核心（纯头文件）
│   ├── component.h                  组件基类（虚接口 + 生命周期钩子）
│   ├── entity.h                     实体（组件挂载容器，挂载序 = 步进序）
│   ├── world.h                      世界（实体注册表 + Step + 共享场景状态）
│   ├── signals.h                    事件信号集合（Boost.Signals2 具名信号）
│   └── events.h                     组件间通信的事件类型（纯数据结构）
├── components/                      模块组件（每个模块对应一个组件）
│   ├── flight_component.h/.cpp      FlightComponent：六自由度机动（起飞→航点→降落）
│   ├── ar_sensor_component.h/.cpp   ArSensorComponent：机载雷达会话
│   ├── esr_sensor_component.h/.cpp  EsrSensorComponent：电子侦察会话
│   ├── eos_sensor_component.h/.cpp  EosSensorComponent：光电会话
│   ├── sbirs_sensor_component.h/.cpp SbirsSensorComponent：天基红外会话（第 4 融合通道）
│   ├── sar_sensor_component.h/.cpp  SarSensorComponent：合成孔径雷达产品（不入融合）
│   ├── fusion_component.h/.cpp      FusionComponent：多源融合引擎
│   ├── demo_log.h/.cpp              集成端日志设施（CA_LOG_EVENT / CA_LOG_EVENT_DUP / CA_LOG_VIEW 宏 → 事件/视图两个命名 logger → integration_events.log / integration_views.log；并装配库日志 1q_library.log）
│   ├── demo_log_modes.h             日志模式选择区（纯宏定义：视图/事件各三模式，宏门控不参与编译；默认跨周期增量+只记关键，可由 CMake 变量覆盖）
│   ├── demo_log_i18n.h              issue code → 中文名适配表（纯查表零依赖；不翻译/不解析 message，量值走 DebugView 结构化字段；未知 code 回退英文原文）
│   ├── sensor_utils.h               平台坐标转换（ECEF 解析）
│   └── scene_types.h                DemoSceneState：共享场景状态（真值注入）
├── component_attachment_demo.cpp    主程序（装配与编排：场景文件加载 + 实体/会话创建 + 周期循环 + 查询演示 + 冒烟断言）
├── demo_config.h/.cpp               演示常量 + 五会话配置加载（JSON 基线）
├── scene_data.h/.cpp                场景描述（scenes/*.json → SceneData + 业务覆写应用；
│                                    coverage 块经 AreaCoveragePlanner 规划巡逻航路）
├── scene_script.h/.cpp              世界模型目标真值脚本（场景目标脚本 → ECEF 状态 → 四通道周期真值 + 推进）
├── scenes/                          场景描述文件（JSON 数据驱动；baseline_takeoff_east.json 为基线）
├── demo_output.h/.cpp               输出落盘与事件消费（DemoOutputs 平台轨迹 CSV / DecisionListener 事件链）
├── CMakeLists.txt
└── README.md
```

> 传感器输出 → 融合探测记录的边界适配（`Adapt*` 系列）与源通道常量在
> `examples/common/sensor_adapt.h`（与 behavior_layer 共用，消除双份维护）。

## ECS 核心设计

### 组件基类（core/component.h）

```cpp
class Component {
 public:
  virtual ~Component() = default;
  virtual const char* Name() const = 0;          // 类型标识（日志/卸载）
  virtual void OnAttach(Entity& host) {}         // 挂载钩子：保存宿主引用
  virtual void OnDetach() {}                     // 卸载钩子
  virtual void Step(World& world, double dt_sec) = 0;  // 周期推进
};
```

组件携带逻辑：每个模块组件封装一个库会话/引擎，`Step` 内驱动会话并产出状态。

### 实体与挂载（core/entity.h）

- 实体是组件的**挂载容器**：`Attach(std::unique_ptr<Component>)` 按序追加并触发
  `OnAttach`；`DetachByName` 触发 `OnDetach`；实体销毁时逐个 `OnDetach`（钩子对称）。
- **挂载序 = 周期执行序**：本示例
  `Flight → AR → ESR → EOS → SBIRS → SAR → Fusion`——传感器读到推进后的
  平台位姿，融合读到本周期探测（SAR 无探测，位置在传感器之后、融合之前）。
- **同实体组件通信**（周期内同步数据聚合）：`host.Find<T>()`（dynamic_cast
  类型化访问）。例如 `FusionComponent::Step` 聚合四个传感器组件的探测记录。

### 世界与共享上下文（core/world.h）

- `World` 承担 EnTT registry 的职责：`CreateEntity(name)`（创建序 = 步进序）、
  `FindEntity(name)`、`Step(dt)` 顺序步进全部实体、`signals()` 事件通道。
- **共享场景状态**（对应 `registry.ctx()`）：`SceneState` 基类（cycle/t_sec），
  消费方继承扩展（`DemoSceneState` 增加四通道世界真值 + 天基平台位置 + SAR
  点目标），每周期更新、组件读取。

### 事件机制（core/signals.h + events.h）

事件机制直接使用 **Boost.Signals2**（C++ 常见开源事件库，boost 已在 conanfile
基础依赖，header-only）：

```cpp
// 发布（组件内）
world.signals().on_target_confirmed(event);
// 订阅（消费方）
boost::signals2::scoped_connection conn =
    world.signals().on_target_confirmed.connect(handler);
```

10 个具名信号（信号 = 事件通道，仅组织库对象，零自定义分发逻辑）：

| 信号 | 事件 | 发布方 |
| --- | --- | --- |
| `on_platform_state` | `PlatformStateEvent` | FlightComponent（每周期） |
| `on_waypoint_reached` | `WaypointReachedEvent` | FlightComponent（航点完成） |
| `on_target_confirmed` | `TargetConfirmedEvent` | ArSensorComponent（首确认，源：库内 ArTrackLifecycleRecorder） |
| `on_target_lost` | `TargetLostEvent` | ArSensorComponent（失跟，源：库内 ArTrackLifecycleRecorder） |
| `on_emitter_hypothesis` | `EmitterHypothesisEvent` | EsrSensorComponent（假设） |
| `on_eos_detection` | `EosDetectionEvent` | EosSensorComponent（首发现/更新/丢失，源：库内 EosDetectionLifecycleRecorder） |
| `on_sbirs_detection` | `SbirsDetectionEvent` | SbirsSensorComponent（首发现/更新/coasting/丢失，源：库内 SbirsDetectionLifecycleRecorder） |
| `on_sar_product` | `SarProductEvent` | SarSensorComponent（产出/持续/丢失/失败，源：库内 SarProductLifecycleRecorder） |
| `on_fusion_updated` | `FusionUpdatedEvent` | FusionComponent（态势更新） |
| `on_command_issued` | `CommandIssuedEvent` | DecisionListener（决策，事件链） |

两种通信形态同时演示：**周期内同步数据聚合**用组件类型化访问；**跨周期通知 /
记录**走信号（事件）。

### 事件日志与调试视图落盘（components/demo_log.h，外部集成惯用法）

**两个日志模块**，输出到两个文件（与库内部 `src/common/logging/ProjectLog.h`
区分——库日志走 spdlog 默认 logger，宿主拥有 logger 生命周期）：

| 模块 | 后端 | 输出文件 |
| --- | --- | --- |
| **库内部日志** | 库内 `PROJECT_LOG_*` 宏 → spdlog 默认 logger（`InitIntegrationLog` 装配为文件 sink） | `1q_library.log`（时间戳 + 级别 + 消息） |
| **集成端日志** | spdlog 命名 logger `"integration_events"` / `"integration_views"`（均带 stdout，pattern 仅为消息体） | `integration_events.log`（事件行）+ `integration_views.log`（各组件每周期调试视图行） |

集成端日志的字符串**归属组件源文件**（事件产生处），通过日志宏就地填充——
外部集成的典型形态（宏背后接消费方自己的日志/落盘设施；本示例直接使用
conanfile 的 spdlog 依赖，fmt 风格 `{}` 格式化，编译期格式检查）。**日志内容为
中文人读文本**（给人类看，不做结构化落盘；规则 12 的结构化持久化由外部集成方
接入自己的日志/事件系统）：

```cpp
// 组件源文件内（发布信号前）
CA_LOG_EVENT(world, "target_confirmed", "目标={} 位置=({:.5f},{:.5f})",
             static_cast<unsigned long long>(confirmed.target_id),
             confirmed.position.latitude_deg, confirmed.position.longitude_deg);
world.signals().on_target_confirmed(confirmed);
```

`CA_LOG_EVENT(world, type, ...)` 的 cycle/t_sec 取自共享场景状态（与事件字段
同源），背后设施把事件行（`[事件:type] 周期=... 时间=...s 中文详情`）写入
`integration_events.log` 并打印控制台，另维护事件计数（摘要/冒烟断言用）。事件宏分两类：
`CA_LOG_EVENT`（关键事件：确认/丢失/首发现/产出/失败/航点/指令等）与
`CA_LOG_EVENT_DUP`（周期性重复事件：每周期平台状态、`kUpdated`/`kProductSustained`
更新类、辐射源假设、融合更新——仅在事件模式一（KEY）下不落盘，信号照常发布）。
集成方替换该设施即接入自己的日志系统；单元测试不初始化日志设施，宏调用静默跳过
（no-op）。

**调试视图落盘在组件内直写**（规则 12）：各传感器组件（AR/EOS/SBIRS/SAR）的
`Step` 在构建 `LastDebugView()` 后直写中文人读行到集成端视图日志
（`integration_views.log`）——日志给人读，示例不做结构化落盘：`session_contract.md` 规则 12 的"调用方结构化持久化
DebugView"由外部集成方接入自己的日志/事件系统实现，结构化格式与字段布局由
调用方自定（参考 `*OutputDebugView` 字段集合直接转写）。

**日志三模式（宏门控，编译期）**：DebugView 每周期都会产生，落盘多少、怎么落
由集成方决定——`components/demo_log_modes.h` 顶部"模式选择区"示范三种常见写入方式，
未选中的模式**不参与编译**。模式选择有两条途径（互斥）：
1. **CMake 构建时控制**（推荐，无需改源码）：`-DCA_VIEW_LOG_MODE=summary|nonnominal|delta`
   `-DCA_EVENT_LOG_MODE=all|key|aggregate`（不传则用源码默认；非法值 FATAL_ERROR）；
2. **源码调试时**：改 `demo_log_modes.h` 里的注释（每次只启用一个视图模式 +
   一个事件模式）重新编译。

默认模式：**视图模式二（跨周期增量）+ 事件模式一（只记关键事件）**：

| 模式 | 宏 | 行为 |
| --- | --- | --- |
| 视图模式一（只落非标称行） | `CA_VIEW_LOG_MODE_NONNOMINAL` | 每周期只把非标称目标（AR 非 `kConfirmed`；EOS/SBIRS 非 `kDetected`）逐行写日志，全标称时写一行"全部正常"；日志量 ∝ 异常数 |
| 视图模式二（跨周期状态增量，**默认**） | `CA_VIEW_LOG_MODE_DELTA` | 只写状态与上一周期不同的目标行（上一周期状态表由组件持有）；无变化时写一行"无状态变化"；日志量 ∝ 变化数 |
| 视图模式三（每周期摘要行） | `CA_VIEW_LOG_MODE_SUMMARY` | 每周期一行中文摘要（周期/完成与否/目标状态明细带**结构化量值**——方位/俯仰/距离/RCS，库 DebugView 输入实体回填，未检测也可见；问题列表为 **code + 中文名**（`demo_log_i18n.h` 查表，未知 code 回退英文 message 原文，不翻译/不解析 message），日志量恒定） |
| 事件模式一（只记关键事件，**默认**） | `CA_EVENT_LOG_MODE_KEY` | `CA_LOG_EVENT` 逐条落盘，`CA_LOG_EVENT_DUP`（周期性重复事件）不落盘 |
| 事件模式二（周期聚合） | `CA_EVENT_LOG_MODE_AGGREGATE` | 每周期把全部事件聚合为一行（`[事件聚合] 周期=N 事件数=M [中文名×次数, ...]`） |
| 事件模式三（逐条全量） | `CA_EVENT_LOG_MODE_ALL` | 事件逐条落盘 |

SAR 为**阶段型视图**（无逐目标状态），不适用目标级三模式落盘，只实现每周期
摘要行（执行状态/完成阶段/L1/L3 成像标志/SNR/点目标数/问题列表）。

**Lifecycle 事件字符串化**：库内 `*LifecycleRecorder` 产出的生命周期事件
（`GetLastEvents()`，如 AR 首确认/失跟、EOS/SBIRS 首发现/丢失、SAR 产品事件）
为纯 struct、库内无字符串化工具——其"转字符串写日志"由各组件源文件内宏的
手写中文格式串承担（`"类型=首发现 探测ID={} 目标={} 信噪比={:.1f}dB 方位={:.1f}°"`
等，kind/status 枚举在组件内做中文名映射），字符串归属组件（组件自描述）；
DebugView 同理以组件内摘要行直写，示例层不内置 JSON 序列化器。

## 模块 → 组件映射

| 组件 | 封装库模块 | 周期行为 | 发布信号 |
| --- | --- | --- | --- |
| `FlightComponent` | flight_dynamic（FD 门控 + 运动学回退） | 六自由度机动推进：起飞→航点巡航→降落；航点完成判定；**循环巡逻**（coverage 场景：FD 模式航点簿记消费库完成事件、kCompleted 后以当前状态 Reset 重建续飞；运动学回退路径带航点寻的 + 索引回绕，段间瞬时转向） | on_platform_state、on_waypoint_reached |
| `ArSensorComponent` | airborne_radar（ArSession + ArCycleOutputAdapter + ArTrackLifecycleRecorder） | 探测 → `DetectionRecord`（key=关联键，含位置）；首确认/失跟事件由库内 recorder 差分产生 | on_target_confirmed / on_target_lost |
| `EsrSensorComponent` | electronic_surveillance_radar（EsrSession） | 假设 → `DetectionRecord`（key=假设键，方位+射频特征） | on_emitter_hypothesis |
| `EosSensorComponent` | electro_optical_sensor（EosSession + EosCycleInputAdapter + EosDetectionLifecycleRecorder） | 探测 → `DetectionRecord`（key=0，仅方位）；首发现/更新/丢失事件由库内 recorder 差分产生 | on_eos_detection |
| `SbirsSensorComponent` | sbirs_sensor（SbirsSession + SbirsDetectionLifecycleRecorder） | 探测 → `DetectionRecord`（key=0，仅方位，与 EOS 同构）；首发现/更新/coasting/丢失事件由库内 recorder 差分产生 | on_sbirs_detection |
| `SarSensorComponent` | sar（SarSession + SarProductLifecycleRecorder） | 孔径积累成像；产品生命周期事件由库内 recorder 差分产生（**无探测输出，不入融合**，契约见 docs/review/Bahavior.md）；阶段型调试视图每周期直写摘要行 | on_sar_product |
| `FusionComponent` | fusion（FusionEngine） | 聚合四传感器探测一次 `Update`；新/消失差分 | on_fusion_updated |
| `ThreatComponent` | threat_assessment（ThreatEvaluator） | 融合态势 + AR 调试视图按键组装输入 → 威胁分/等级；等级升级（含首见）→ 升级事件；每周期视图摘要行 | on_threat_updated |

## 运行时修改接口

每个组件把库模块的「运行时修改」设计暴露为组件公开方法（未来外部调用入口，
薄包装库 API，不改库行为）。提交语义随模块而异（权威定义：
`docs/common/contract.md`「运行期配置提交策略」）：

| 组件 | 接口 | 提交语义 |
| --- | --- | --- |
| `ArSensorComponent` | `bool TryApplyRuntimeConfig(const ArRuntimeConfigPatch&)` | **事务性提交**：补丁先暂存，下次成功周期边界统一生效（失败由库内快照完整回滚）；与现有配置冲突的非法补丁入口即原子拒绝 |
| `EsrSensorComponent` | `bool TryApplyRuntimeConfig(const EsrRuntimeConfigPatch&)`；`ApplyRuntimeConfigWithResult(...)` → 结构化结果（拒绝原因枚举） | **立即提交**：调用即生效、单向落定（session 层无回滚）；结构化结果供外部决策/诊断 |
| `EosSensorComponent` | `bool TryApplyRuntimeConfig(const EosRuntimeConfigPatch&)` | **立即提交**：补丁经 resolver 原子校验后一次生效；frame_rate_hz 热更新经 resolver 校验（非法值整补丁拒绝） |
| `SbirsSensorComponent` | `bool TryApplyRuntimeConfig(const SbirsRuntimeConfigPatch&)` | **立即提交**：补丁经配置校验后一次生效（如 scan_rate < 0 整补丁拒绝） |
| `SarSensorComponent` | `bool TryApplyRuntimeConfig(const SarRuntimeConfigPatch&)` | **立即提交**：补丁经配置校验后一次生效（如 retain_raw_phase_history 依赖 raw echo，依赖不满足时整补丁拒绝） |
| `FlightComponent` | `bool PushManeuver(const ManeuverCommand&)`；`bool ClearManeuvers()`；`bool Abort()` | **命令式**（FD 无 patch 范式）：追加机动队列/清空/中止；FD 未启用或初始化失败（运动学回退）时返回 false（指令被丢弃） |
| `FusionComponent` | 无 | fusion 模块参数为**会话级不可变**（库内无 RuntimeConfigPatch 设计，FusionEngine 仅构造时接受 FusionConfig）；需要运行时修改须先补库 API，不在示例层包装 |

补丁字段统一用 `has_*` 位标志选择（未设置的字段不覆盖现值）；传感器组件的
生命周期 recorder 事件源与运行时修改入口互不影响（recorder 只管差分事件，
patch 只改会话配置）。

## 实体状态查询接口（外置查询数据源）

选定实体后按实体名/ID 拉取设备状态的查询数据源：各传感器组件把**最近周期
执行真相**暴露为 const getter（每周期 `Step` 随会话结果刷新，外部系统读取
即"当前状态"；未步进前默认 `powered_on=true`，与 `sensor_enabled=true` 配置
默认一致）。有对应库语义的组件才提供函数：AR/SAR 无扫描方位概念（跟踪/
成像雷达），不提供角度函数。

| 组件 | 开关机 | 扫描方位 |
| --- | --- | --- |
| `ArSensorComponent` | `bool powered_on()` | —（无扫描方位概念） |
| `EsrSensorComponent` | `bool powered_on()` | `float scan_azimuth_deg()`（deg，平台系） |
| `EosSensorComponent` | `bool powered_on()` | `float scan_azimuth_deg()`（deg，平台系） |
| `SbirsSensorComponent` | `bool powered_on()` | `float scan_azimuth_deg()`（deg，ECEF 极坐标参考） |
| `SarSensorComponent` | `bool powered_on()` | —（无扫描方位概念） |

有 DebugView 的四个组件（AR/EOS/SBIRS 目标列表型 + SAR 阶段型）另暴露
`const *OutputDebugView& LastDebugView()`——最近周期调试视图快照（规则 12 落盘示范：
per-target 状态 + 规则 13b kInfo 排除诊断；关机周期清零，拒绝周期为
`kCycleNotCompleted`/`kCycleNotExecuted` 快照）。组件在 `Step` 内取该视图
直写中文人读行到集成端视图日志 `integration_views.log`（每周期一行，如
`[视图:ar] 周期=5 完成=是 目标=[1001 已确认(RCS 2.20m²)] 问题=[ar.target_snr_below_threshold 目标信噪比低于门限]`；
SAR 为阶段型摘要行）——日志给人读，结构化持久化由外部集成方接入自己的
日志/事件系统实现（示例不内置 JSON 序列化器）。落盘密度（三模式）由
`components/demo_log_modes.h` 模式选择区宏控制。ESR 库内无 DebugView，不适用
视图落盘。

**组件层电源门控**：电源状态由 `sensor_enabled` 补丁唯一维护（`TryApplyRuntimeConfig`
成功且带 `has_sensor_enabled` 时更新，拒绝的补丁不改状态）。关机时组件**不驱动
会话**（设备不工作，会话扫描相位冻结），角度主动清零表示无有效扫描方位；重新
上电后组件恢复驱动，会话从冻结状态继续推进。角度字段在开机周期随会话输出帧
刷新（被拒绝周期为空帧 → 0）。消费方一律以 `powered_on()` 守卫：仅开机且最近
周期 `kCompleted` 时角度才代表有效扫描方位。demo 末尾
"Platform Sensor States (entity query)" 块演示按实体查询（`Entity::Find<T>()`）。

## 周期调用序

```
每周期（World::Step(1.0)）：
  platform 实体按挂载序步进：
    1. Flight    → 推进平台位姿（FD 子步进 10 ms × 100；或运动学回退）
                  → 发布 PlatformStateEvent / WaypointReachedEvent
    2. ArSensor  → 读 Flight 状态构建周期输入 → 驱动 ArSession
                  → 探测存自身 detections_，发布轨迹事件
    3. EsrSensor → 同上（EsrSession）
    4. EosSensor → 同上（EosSession）
    5. SbirsSensor → 读共享场景天基平台位置与红外目标真值 → 驱动 SbirsSession
                  → 探测存自身 detections_，发布探测生命周期事件
    6. SarSensor → 读 Flight 状态（LLA/NED）与 SAR 点目标真值 → 驱动 SarSession
                  → 发布产品生命周期事件（无 detections_，不入融合）
    7. Fusion    → 类型化聚合四传感器 detections_ → FusionEngine::Update
                  → 发布 FusionUpdatedEvent
    8. Threat    → 融合输出（置信度）+ AR 调试视图（运动学/RCS）按键组装
                  → ThreatEvaluator::Evaluate → 威胁分/等级
                  → 等级升级 → 升级事件，逐目标发布 ThreatUpdatedEvent
  订阅者（demo 侧）：
    DecisionListener → 置信度 ≥ 3.0 或威胁等级 HIGH → 发布 CommandIssuedEvent（事件链）
    组件宏（CA_LOG_EVENT / CA_LOG_VIEW）→ 事件与视图摘要就地记录 → integration_events.log / integration_views.log（人读行）
```

## 场景设计：六自由度机动从起飞开始

**场景由场景描述文件驱动**（`scenes/*.json`，见下节）：平台飞行脚本、目标脚本、
ESR 波形、天基平台、EOS 扫描、SAR 任务几何/链路、融合配置与冒烟下限全部数据化——
新场景 = 新 JSON 文件 + 重跑，无需改代码；`scene_script` 只保留"目标脚本 → ECEF 状态
→ 各通道真值"的纯转换。下述基线行为对应 `scenes/baseline_takeoff_east.json`。

`FlightComponent` 的 FD 路径遵循 `examples/flight_dynamic/takeoff_land_csv.cpp`
的权威用法（**不做空中配平**——空中配平虽允许但存在不稳定问题）：

- **初始条件**：机场地面（alt 0）、零速度、姿态水平、`do_trim = false`；
- **机动队列**：`kTakeoff`（滑跑→抬轮→爬升到巡航高度）→ 航路点巡航
  （kFlyToWaypoint）→ `kLand`（降落目标 = 航路终点，场景简化）；
- **子步进 10 ms**：地面滑跑/起落架为快动态，100 ms 步长会失控发散
  （实测起飞段 roll 达 180° 量级后数值崩溃；10 ms 与权威示例一致）；
- **运动学回退**（FD 关闭/初始化失败）：模拟起飞爬升（5 m/s 到巡航高度）+
  巡航直线，行为语义一致。

演示场景（400 周期）：机场 (30°N, 120°E) → 起飞东飞 → 巡航 400 m / ~50 m/s →
3 个航点（正东，间距约 5 km）。目标真值 2 个空中目标固定 400 m 高度、**正北**
12/14 km（东速 47 m/s = FD 巡航实际地速，保持目标恒在平台正北侧方；北速
±5 m/s 提供两目标分离与 AR 径向速度）。目标方位（正北）落在 EOS 扫描覆盖内
（平台局部系 az 0 = 东，扫描 50°~130°，覆盖正北 az 90°）。EOS 探测距离窗
≈ [11.5, 22.9] km（400 m 高度 × 俯仰角 2°/1°），目标斜距全程稳定在窗内
（起飞爬升期平台高度不足、窗口窄，探测从爬升后期开始）。FD 模式实测（目标
位置修复后）：起飞 ~157 s 完成、航点 0 在演示窗口内到达（`waypoint_reached`
事件）、EOS 生命周期事件 ~300 条（首发现/丢失各 ~150 次，cycle 101 起——
起飞段平台向西北爬升（FD 实际航迹 hdg 293°→358°），目标相对平台方位从 90°
快速扫到 ~59°，cycle 1-100 恰好不在扫描波束窗内）；运动学模式下 3 个航点
全部到达、EOS 探测记录 ~177 次。事件目标 ID 为外部原始目标标识（1001/1002），
无外部标识时回退 AR 内部关联键。

### 区域巡逻（coverage）

场景 `platform` 块可选 `coverage` 字段声明巡逻区域与规划参数，加载时经
`navigation::AreaCoveragePlanner`（多边形牛耕式扫描 / 圆形盘旋，独立中立算法面）
生成巡逻航路，平台沿航路**循环巡逻**。`coverage` 与显式 `platform.waypoints`
互斥（航路来源歧义报错）；规划失败（几何非法/模式-区域不匹配）报错退出，
不允许静默退化为直飞。

### 多机编队（platforms[]）

场景可选顶层 `platforms[]` 数组声明**从机**（多机编队）：每条目同 `platform`
块字段（原点/航向/巡航 + `waypoints` 或 `coverage` 区域任务），另加可选
`name`（实体名，缺省 `wingman_<N>`）。主平台（`platform` 块）挂载五传感器
与融合（感知资产）；从机**纯飞行**（只挂 FlightComponent），各自航路/区域
任务 = "不同指令"。飞行器编号（aircraft_id） = 1（主）+ 2..N（按数组序），
落盘到共享可视化契约（platform_track/route_plan 的 aircraft_id 列）。

**循环巡逻的执行语义（按 flight_dynamic 权威语义实现）**：

- **运动学回退路径**：航点寻的（每周期航向指向下一航点，段间瞬时转向），航路
  耗尽后索引回绕首个航点——几何干净，直线段序列可手算先验；
- **FD 模式**：航点簿记**消费库完成事件**（`FlightManager::GetWaypointEvents()`，
  每航点 1 条，含完成门/距离快照）——库的航点完成是双层语义（中间航点法平面
  穿越或到达圈、最终航点转弯量级捕获圈），组件自研几何判定与库语义必然错位，
  不参与 FD 模式；航路完成后 kCompleted → **以当前载机状态 Reset 重建续飞**
  （库状态机契约：kCompleted 必须 Reset 恢复 kReady；初始运动学取自
  VehicleState，do_trim=false 不做空中配平）——DIAG 仿真时间重置即循环证据。

`patrol_area_scan.json`：巡逻区域 lat 29.9905~30.0045（约 1.56 km 南北）×
lon 120.0~120.02（约 1.93 km 东西），扫描航向 0（线沿东西）、间距 500 m →
3 条扫描线 = 6 航点（线纬度 29.99275/29.99725/30.00175），牛耕式交替方向；
到达半径 200 m（小于扫描间距，运动学路径防过渡航点连续到达合并）。
**cycles=600**（FD 模式起飞段 ~157 s 为硬开销，400 周期窗口装不下首轮+循环；
600 内 FD 首轮 ~423 s 完成 + 第二轮 4 航点可见，运动学模式 4 轮完整循环）。
目标相对基线**仅把 `v_east_mps` 47 归零**（保持正北 12/14 km 静止：扫描线
东西向飞行时目标恒在侧方，SAR squint ≈ 0 成像成立、EOS 方位恒在扫描扇区内；
`v_north_mps` ±5 保留，提供 AR 径向速度）——基线目标随平台同速东飞的设计
与"小区域往复巡逻"不兼容，属单变量控制调整（与 sbirs 场景归零 v_north 同理）。
南北过渡段（约 6 s/段）SAR 侧视几何不成立，被 squint 门控拒绝（**按设计
拒绝**，预期出现）。注意：SAR 首产出在 cycle 1（跑道低速假成像，SNR −2 dB
级，几何成立下的物理真实——基线同窗口是 squint 拒绝，因场景中心方位不同）；
产品事件 KEY 模式只落首条（持续类门控），sar_products=1 与基线一致。

### 天基通道（SBIRS / SAR）场景设计

- **SBIRS 天基平台（卫星）**：位置由消费方每周期注入共享场景状态（世界模型
  驱动），凝视模式固定于目标群中心正上方 +500 km（ECEF z 轴）。SBIRS 的
  az/el 为 **ECEF 极坐标**（`az=atan2(y,x)`、`el=asin(z/r)`，见库内
  SbirsGeometry），因此扫描配置为全向（span 360°）+ 下视（scan_center_el
  −90°），目标始终位于星下点附近（FOV 20° 覆盖）。**示例简化声明**：星载
  方位参考系（ECEF 极坐标）与机载通道方位（平台局部系）不同，融合方位相干
  门限（8°）无法跨参考系关联，SBIRS 探测在融合中独立成目标（通道 4）——
  演示"异构通道聚合"而非跨源关联。
- **SAR 侧视几何**：SAR 的 squint 门控（库内 `max_allowed_squint_angle_deg`
  = 10°，覆写自 sar.json 的 5°）要求平台视线垂直航迹。目标与平台同速东飞
  使目标恒在平台正北侧方，场景中心配置在 FD 巡航段（~cycle 331 起正东直线）
  的侧方（30.117°N, 120.06°E），巡航段飞越时 squint ≈ 0°、成像窗口覆盖
  ~cycle 19-397（产品事件 ~200 条）；起飞/转弯段（cycle 1-330 的绕飞与转向）
  侧视几何不成立，被库内 squint 门控拒绝（`PROJECT_LOG_ERROR` 每失败周期
  一条，**预期行为**：真实系统在几何不满足时同样不成像）。
- **SAR 链路预算**：目标 RCS 仅 2.2/1.4 m²，在 sar.json 的 10 kW 峰值功率 +
  30 dBi 天线下 13 km 斜距链路 SNR ≈ −29 dB（低于 minimum_snr_db）→ demo
  覆写峰值功率 1 MW、天线增益 40 dBi（SAR 常用量级）；目标位置修复后点目标
  回到场景中心附近真实位置（正北 12/14 km、400 m 高，斜距 ≈ 13 km），实测
  成像期 SNR ≈ 1.5~5.3 dB（高于 minimum_snr_db，L1 成像成立）；
  孔径 1024 脉冲 @ PRF 100 Hz ≈ 10.24 s 积累，逐周期滚动成像。
- **SAR 平台状态简化**：组件从 FlightComponent 读 LLA 位置与航向/速度，航向
  分解为 NED 速度（北/东分量），**姿态零假设**（roll/pitch 0、yaw = 航向）
  ——FD 模式未对外暴露姿态，示例简化。

> 生命周期事件语义说明：AR/EOS/SBIRS 传感器组件的"首确认/失跟"、"首发现/更新/丢失"
> 事件与 SAR 的"产出/持续/丢失/失败"事件由**库内 LifecycleRecorder**
> （`ArTrackLifecycleRecorder` / `EosDetectionLifecycleRecorder` /
> `SbirsDetectionLifecycleRecorder` / `SarProductLifecycleRecorder`）承担——
> Attach 到会话后 `StepWithResult` 内部自动驱动跨周期状态差分，集成侧不再
> 自研状态集合判定（消除掉轨漏报/集合无界等自研 bug）。`EosDetectionEvent.kind`
> / `SbirsDetectionEvent.kind` / `SarProductEvent.kind` 标注生命周期类型；
> 诊断类事件（`kNotDetected` / `kNoProduct`）未开启不转发。本场景 EOS 扫描
> 步进（20°/周期）大于波束宽度，目标约每 4 周期被探测 1 周期，故事件流以
> "首发现→丢失"交替为主、`kUpdated` 稀少——这是场景物理（扫描断续）在
> 生命周期语义下的自然表现。SAR 的 squint 门控失败周期（起飞/转弯段）不产生
> 生命周期事件（recorder 对非执行周期静默），属库内契约。

### 场景描述文件（数据驱动）

场景 = 消费方世界模型 + 业务调参的数据化载体，加载见 `scene_data.h/.cpp`
（`LoadSceneData` + `ApplySceneOverrides`，解析复用 `examples/common/json_reader.h`，
遵循 config_loader 惯例：缺省字段静默默认、语法错误与必填几何字段缺失报错）。
**场景文件即配置记录**（git 跟踪），新场景只加 JSON，不改代码。

顶层结构（`scenes/baseline_takeoff_east.json` 为基线样例）：

| 块 | 必填 | 字段（缺省值） |
| --- | --- | --- |
| `name` / `cycles` / `dt_sec` | 否 | 场景名 / 周期数（400）/ 步长 s（1.0） |
| `platform` | **是** | `origin_lat_deg`/`origin_lon_deg`（**必填**）、`origin_alt_m`（0）、`initial_heading_deg`（90）、`cruise_altitude_m`（400）、`cruise_speed_mps`（50）、`waypoints[]`（lat/lon 必填，alt/speed 缺省回退巡航参数、radius 500；**与块内 `coverage` 互斥**）、`coverage`（可选区域巡逻任务，字段见下） |
| `platforms[]` | 否 | 从机数组（多机编队，纯飞行不挂传感器）：每条目同 `platform` 块字段 + `name`（缺省 `wingman_<N>`）；巡航参数缺省回退主平台值 |
| `coverage`（platform/platforms[] 条目内） | 否 | 区域巡逻任务：`kind`（polygon/circle）、`mode`（scan/orbit，须与 kind 匹配）、polygon `vertices[]`（lat/lon 必填）或 circle `center` + `radius_m`、`scan_heading_deg`（0 = 扫描线沿正东）、`scan_spacing_m`（须 > 0）、`altitude_m`/`speed_mps`（缺省回退巡航参数）、`arrival_radius_m`（500）、`orbit_segments`（8）/`orbit_rings`（1）。加载时经 `navigation::AreaCoveragePlanner` 生成巡逻航路（填入该平台的 `waypoints`），**循环巡逻**（航路飞完回绕首航点）；规划失败（顶点 < 3/间距非正/模式-区域不匹配等）报错退出 |
| `targets[]` | **是**（可为空 = 无目标场景） | `id`/`azimuth_deg`/`range_m`/`altitude_m`/`rcs_m2`（**必填**）、`type`（`air`/`ground`，缺省 air；ground = 地面目标，静止近地运动学点，可视化以不同线型标注）、`v_east_mps`/`v_north_mps`（0）、`temperature_k`（0）、`projected_area_m2`（0）、`emitter_center_frequency_hz`（0 = 不配辐射源）、`maneuvers[]`（可选变速机动表：`start_cycle` 必填且严格递增，`v_east_mps`/`v_north_mps` 缺省 0——**绝对速度分段匀速**，未指定分量 = 0，须写全） |
| `esr` | 否 | 辐射源波形：`peak_gain_dbi`（30）、`bandwidth_hz`（2e6）、`peak_power_w`（5e7）、`pulse_width_s`（1e-6）、`pri_s`（1e-3）、`pulse_count`（200）、`timing_seed`（42） |
| `sbirs_satellite` | 否 | `altitude_m`（500000，凝视目标群质心正上方） |
| `eos_scan` | 否 | EOS 业务覆写：`frame_rate_hz`（10）、`scan_rate_deg_per_sec`（20）、`scan_start_az_deg`（50）、`scan_end_az_deg`（130）、`scan_center_el_deg`（0）、`boresight_depression_deg`（0） |
| `sar` | 否 | SAR 任务几何/链路覆写：`peak_power_w`（1e6）、`antenna_gain_db`（40）、`max_squint_deg`（10）、`scene_center_latitude_deg`（30.117…）、`scene_center_longitude_deg`（120.06）、`scene_center_altitude_m`（400）、`slant_range_m`（13000）、`platform_speed_mps`（50） |
| `fusion` | 否 | `position_radius_m`（1000）、`bearing_beamwidth_deg`（5）、`feature_threshold`（0）、`window_size`（10）、`max_missed_cycles`（5）、`source_weights[]`（空 = 全 1.0） |
| `high_threat_confidence` | 否 | 决策门限（3.0） |
| `smoke` | 否 | 冒烟下限：`min_key_events`/`min_sbirs_events`/`min_sar_products`/`min_fused_targets`（全 1；零产出场景显式置 0） |

原 demo_config 内硬编码的 EOS/SAR 业务覆写已迁入场景数据（`eos_scan`/`sar` 块，
经 `ApplySceneOverrides` 应用）；`kCruiseAltitudeM`/`kCruiseSpeedMps`/
`kHighThreatConfidence`/`kDtSec` 常量随迁出移除，语义见 SceneData 默认值。
场景验证工作流（预期事件表/三分类判定/原型库）见仓库 skill `scenario-verify`。

现有场景集（`scenes/`，每场景一份预期表归档 `*.md`）：

| 场景文件 | 被测行为 | 预期表结论 |
| --- | --- | --- |
| `baseline_takeoff_east.json` | 基线：全通道端到端（探测→融合→决策） | 通过（含 SBIRS 穿越质心注） |
| `no_targets_clean_airspace.json` | 空域清净：零假警（SAR 产品与点目标解耦） | 通过（1 项预期修正） |
| `target_maneuver_evasion.json` | 目标大机动：跟踪保持（AR 失跟需探测断链） | 通过（1 项预期修正） |
| `sbirs_altitude_snr_1000km.json` | SBIRS 高度专项：链路 1/R² 标度 + 门限边界 | 通过（1 项预期修正） |
| `patrol_area_scan.json` | 区域巡逻专项：coverage 块规划航路 + 循环巡逻（巡逻中四通道探测保持） | 通过 |
| `fleet_patrol_multi_zone.json` | 多机区域巡逻专项：3 机各自区域任务（platforms[]）+ 区域内空中/地面目标 + 契约 v2 多机可视化 | 通过（运动学冒烟 + **FD 600 周期复核**：三机 jsbsim、循环重启 5 次、SAR 起飞段 1 产品） |
| `threat_multi_target.json` | 威胁评估专项：三目标威胁分排序 + 等级映射 + 威胁→决策指令链 | 通过 |

## 构建与运行

```bash
cmake --preset llvm-ninja-release-local -DENABLE_EXAMPLES=ON [-DONEQ_ENABLE_FLIGHT_DYNAMIC=ON]
cmake --build --preset llvm-ninja-release-local --target component_attachment_demo
./build/llvm-ninja-release-local/bin/component_attachment_demo [--scene <path>] [--cycles <n>] [--output-dir <dir>]
```

- `--scene <path>`：场景描述文件（默认 `scenes/baseline_takeoff_east.json`，路径由
  CMake 注入 `CA_SCENE_DIR`）；场景文件即配置记录，见下节；
- `--cycles <n>`：仿真周期数（覆盖场景文件，默认场景文件值）；
- `--output-dir <dir>`：输出目录（日志 + CSV，默认 `examples/component_attachment/log/`
  ——CMake 注入的仓库内绝对路径，运行时产物不入版本控制，见 .gitignore）；
- 日志模式可在 configure 时用 CMake 变量控制（不传则用默认：视图跨周期增量 +
  事件只记关键）：
  ```bash
  cmake --preset llvm-ninja-release-local -DENABLE_EXAMPLES=ON \
      -DCA_VIEW_LOG_MODE=summary -DCA_EVENT_LOG_MODE=aggregate
  ```
- FD 开启时输出 `FlightComponent` 的六自由度机动日志（JSBSim），关闭/失败时
  打印回退告警并走运动学路径。
- 本示例依赖 spdlog（conanfile 非 Windows 依赖），Windows 构建不纳入
  （examples/component_attachment/CMakeLists.txt 门控）。

## 输出

| 文件 | 内容 | 说明 |
| --- | --- | --- |
| `integration_events.log` | 每行一条人读记录 | **集成端事件日志**（spdlog 命名 logger `"integration_events"`）：事件行 `[事件:type] 周期=... 时间=...s 中文详情`（10 类信号事件 + 纯日志事件 `patrol_loop_restart`（巡逻循环重启，无信号，KEY 模式落盘），字符串归属组件源文件，`CA_LOG_EVENT` / `CA_LOG_EVENT_DUP` 宏；事件模式二为 `[事件聚合]` 行） |
| `integration_views.log` | 每行一条人读记录 | **集成端视图日志**（spdlog 命名 logger `"integration_views"`）：AR/EOS/SBIRS/SAR 四组件每周期调试视图行 `[视图:module] 中文摘要`（`CA_LOG_VIEW` 宏；日志给人读，落盘密度三模式由宏门控，见"事件日志与调试视图落盘"节） |
| `1q_library.log` | 人读日志行 | **库内部日志**：库内 `PROJECT_LOG_*` 宏 → spdlog 默认 logger（时间戳 + 级别 + 消息），`InitIntegrationLog` 装配 |
| `platform_track.csv` | cycle,t_sec,lat_deg,lon_deg,alt_m,heading_deg,speed_mps,wp_index | 平台轨迹（每周期一行；FD 模式含起飞爬升段） |

控制台输出每周期一行摘要（平台高度/航向/速度/航点进度 + 融合目标数），事件
逐条打印；结束打印汇总（实体数/组件数/事件数/指令是否下发）。

## 与 behavior_layer（EnTT 模式）对照

| 方面 | behavior_layer（EnTT） | component_attachment（自定义） |
| --- | --- | --- |
| ECS 依赖 | EnTT 3.14（header-only 开源库） | 无 ECS 依赖（core/ 自研约 300 行） |
| 组件形态 | 纯数据 struct（无逻辑） | `Component` 基类 + 子类（携带逻辑） |
| 逻辑归属 | 自由函数系统（5 个 system） | 组件虚接口 `Step`（挂载序执行） |
| 注册表 | `entt::registry` + view/ctx | `World`（实体列表 + scene_state） |
| 实体句柄 | `entt::entity` | `EntityId`（uint64）+ `Entity&` |
| 事件机制 | `entt::observer` / sigh（库自带） | **Boost.Signals2**（常见开源事件库） |
| 组件间数据 | 组件字段 + ctx 读写 | 同实体 `Find<T>()` + 信号发布 |
| 生命周期钩子 | 无内建 | `OnAttach` / `OnDetach` |

两种模式共用同一批库 API（ArSession/EsrSession/EosSession/SbirsSession/
SarSession/FusionEngine/FlightManager）、同一份共享配置（`examples/configs/`
JSON）与同一套探测适配逻辑（`sensor_adapt.h` 与行为层 `systems.cpp` 同构）。

## 测试

- `tests/unit/examples/ecs_core_test.cpp`：ECS 核心单测（实体挂载/卸载生命周期
  钩子与调用序、按名精确卸载、类型化访问与宿主兄弟组件通路、挂载序与创建序
  步进、共享场景状态、信号发布→订阅接线、事件类型隔离）；
- `tests/unit/examples/ecs_component_runtime_test.cpp`：组件运行时修改接口单测
  （AR 合法/非法 patch 的接受与原子拒绝、ESR 立即提交 + 结构化拒绝状态码、
  EOS 立即提交 + 整补丁拒绝、SBIRS/SAR 立即提交 + 整补丁拒绝、FlightComponent
  机动入口在 FD 可用/不可用时的返回语义）+ 状态查询与调试视图单测（开关机/
  扫描方位、AR/EOS/SBIRS/SAR 四通道 `LastDebugView()` 逐目标/阶段型状态与
  13b kInfo 排除诊断、关机清零、SBIRS 关机冻结相位）；
- ctest `examples::component_attachment_demo`：demo 冒烟（400 周期 + 日志/CSV
  落盘 + 最小产出断言：关键事件 ≥ 1、SBIRS 关键探测事件 ≥ 1、SAR 关键产品事件
  ≥ 1、融合目标 ≥ 1、平台轨迹行数 = 周期数、视图行数每周期 ≥ 1 行（AR/EOS/
  SBIRS 为 ≥ 周期数——默认跨周期增量模式下状态变化周期会写多行；SAR 阶段型
  摘要恒每周期一行 == 周期数））。**断言与日志模式无关**：任意视图/事件模式
  组合（含 CMake `-DCA_*_LOG_MODE=...` 切换）下冒烟均成立。
