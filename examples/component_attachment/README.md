# 自定义实体-组件示例（component_attachment）

## 定位

第二种示例模式：**自定义实体-组件**开发（不依赖 EnTT 等 ECS 开源库），与
`behavior_layer`（EnTT ECS 开源库模式）形成两种开发模式对照。

- **组件基类**：`core/component.h` 定义虚接口（Name / OnAttach / OnDetach / Step），
  组件**携带逻辑**（区别于 EnTT 纯数据组件）；
- **模块 → 组件**：每个仿真模块对应一个 `Component` 子类（飞行 / AR / ESR / EOS /
  SBIRS / SAR / 融合），**挂载**到实体上参与仿真；
- **事件机制**：使用 C++ 常见开源事件库 **Boost.Signals2**（零自定义分发层），
  弥补"自研 ECS 无事件功能"（对应 EnTT 的 observer/signals）。

单平台精简场景：1 个 `platform` 实体挂载 7 个组件，400 周期 × 1 s。四传感器
（AR / ESR / EOS / SBIRS）端到端：探测 → 融合 → 高置信威胁 → 决策指令（事件链）；
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
│   ├── sensor_utils.h               平台坐标转换（ECEF 解析）
│   └── scene_types.h                DemoSceneState：共享场景状态（真值注入）
├── component_attachment_demo.cpp    主程序（装配 + 事件日志 + CSV 导出）
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

## 模块 → 组件映射

| 组件 | 封装库模块 | 周期行为 | 发布信号 |
| --- | --- | --- | --- |
| `FlightComponent` | flight_dynamic（FD 门控 + 运动学回退） | 六自由度机动推进：起飞→航点巡航→降落；航点完成判定 | on_platform_state、on_waypoint_reached |
| `ArSensorComponent` | airborne_radar（ArSession + ArCycleOutputAdapter + ArTrackLifecycleRecorder） | 探测 → `DetectionRecord`（key=关联键，含位置）；首确认/失跟事件由库内 recorder 差分产生 | on_target_confirmed / on_target_lost |
| `EsrSensorComponent` | electronic_surveillance_radar（EsrSession） | 假设 → `DetectionRecord`（key=假设键，方位+射频特征） | on_emitter_hypothesis |
| `EosSensorComponent` | electro_optical_sensor（EosSession + EosCycleInputAdapter + EosDetectionLifecycleRecorder） | 探测 → `DetectionRecord`（key=0，仅方位）；首发现/更新/丢失事件由库内 recorder 差分产生 | on_eos_detection |
| `SbirsSensorComponent` | sbirs_sensor（SbirsSession + SbirsDetectionLifecycleRecorder） | 探测 → `DetectionRecord`（key=0，仅方位，与 EOS 同构）；首发现/更新/coasting/丢失事件由库内 recorder 差分产生 | on_sbirs_detection |
| `SarSensorComponent` | sar（SarSession + SarProductLifecycleRecorder） | 孔径积累成像；产品生命周期事件由库内 recorder 差分产生（**无探测输出，不入融合**，契约见 docs/review/Bahavior.md） | on_sar_product |
| `FusionComponent` | fusion（FusionEngine） | 聚合四传感器探测一次 `Update`；新/消失差分 | on_fusion_updated |

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
  订阅者（demo 侧）：
    DecisionListener → 置信度 ≥ 3.0 → 发布 CommandIssuedEvent（事件链）
    EventLogger      → 全部信号 → 控制台 + events.csv
```

## 场景设计：六自由度机动从起飞开始

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
（起飞爬升期平台高度不足、窗口窄，探测从爬升后期开始）。FD 模式实测：起飞
~157 s 完成、航点 0 在演示窗口内到达（`waypoint_reached` 事件）、EOS 生命
周期事件 ~240 条（首发现/丢失各 ~120 次）；运动学模式下 3 个航点全部到达、
EOS 探测记录 ~177 次。事件目标 ID 为外部原始目标标识（1001/1002），无外部
标识时回退 AR 内部关联键。

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
  覆写峰值功率 1 MW、天线增益 40 dBi（SAR 常用量级），SNR ≈ +10 dB 过门限；
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

## 构建与运行

```bash
cmake --preset llvm-ninja-release-local -DENABLE_EXAMPLES=ON [-DONEQ_ENABLE_FLIGHT_DYNAMIC=ON]
cmake --build --preset llvm-ninja-release-local --target component_attachment_demo
./build/llvm-ninja-release-local/bin/component_attachment_demo [--cycles <n>] [--output-dir <dir>]
```

- `--cycles <n>`：仿真周期数（默认 400）；
- `--output-dir <dir>`：CSV 输出目录（默认 `/tmp/component_attachment_viz`）；
- FD 开启时输出 `FlightComponent` 的六自由度机动日志（JSBSim），关闭/失败时
  打印回退告警并走运动学路径。

## 输出

| 文件 | 列 | 说明 |
| --- | --- | --- |
| `platform_track.csv` | cycle,t_sec,lat_deg,lon_deg,alt_m,heading_deg,speed_mps,wp_index | 平台轨迹（每周期一行；FD 模式含起飞爬升段） |
| `events.csv` | cycle,t_sec,event_type,detail | 事件流（10 类事件；detail 为可读摘要） |

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
  机动入口在 FD 可用/不可用时的返回语义）；
- ctest `examples::component_attachment_demo`：demo 冒烟（400 周期 + CSV 落盘 +
  最小产出断言：事件数 ≥ 周期数、SBIRS 探测事件 ≥ 1、SAR 产品事件 ≥ 1、
  融合目标 ≥ 1、平台轨迹行数 = 周期数）。
