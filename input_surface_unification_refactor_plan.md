# 三模块 Input Surface 统一标准与破坏性重构计划

## 0. 执行状态（截至 2026-04-23）

- 第一阶段（统一公开类型骨架）：已完成
- 第二阶段（删除旧公开输入语言）：部分完成
- 第三阶段（适配器统一两步化）：基本完成
- 第四阶段（校验与 replay 收敛）：部分完成
- 第五阶段（公开 input 与内部 model 彻底解耦）：进行中

未收尾重点：

- AR 公开输入已切换为 `RadarSceneTarget`，但 AR 测试与文档仍有少量旧命名语义待继续清理
- 校验结构仅统一到 `entity_index`，尚未落地 `ValidationLocation`
- replay 生成 schema 层仍保留 `target_index` / `emitter_index` 字段名
- EOS 仍保留 `EosTargetState` 兼容别名，未完全收敛到单一命名语言

## 1. 背景与目标

当前 `airborne_radar`、`electro_optical_sensor`、`electronic_surveillance_radar` 三个模块在“外部 input”这一层存在明显不统一：

- 对外入口类型命名不统一：`RadarCycleInput` / `EosCycleInput` / `EsrCycleInput`
- 周期公共字段不统一：AR 缺少 `cycle_index`，EOS/ESR 具备
- 场景实体命名不统一：`target_features` / `scene_targets` / `scene_emitters`
- 输入语义层级不统一：AR 暴露的是“中间特征”`TargetFeature`，EOS 暴露“传感器目标态”`EosTargetState`，ESR 暴露“真值态”`EmitterTruthState`
- 外部适配能力不统一：AR 有两步/旧单步兼容口，EOS 有平台+目标转换，ESR 只有平台位姿转换
- 环境输入挂载位置不统一：EOS 将大量环境事实直接挂在 `CycleInput` 根级字段，ESR 则收敛为 `environment_observation`
- 校验结果结构也不统一：AR/EOS 用 `target_index`，ESR 用 `emitter_index`

本计划的目标不是“调一下字段名”，而是建立一套统一的 input 设计语言，并据此对三模块公开 input surface 做**不保兼容**的破坏性重构。

## 2. 本次统一只讨论什么

本计划只约束下列“外部输入面”：

- 会话运行期单周期输入：`*CycleInput`
- 对外场景实体输入：目标/辐射源/环境观测
- 外部世界到模块局部输入的适配器：`*ExternalInputAdapter`
- 输入校验契约：`*InputValidation`
- replay 中记录的 input payload

本计划**不直接**约束：

- `SessionConfig` / `RuntimePatch`
- 模块内部 pipeline / resolver / mapper
- 输出帧结构

这些层会被 input 重构波及，但它们不是本计划的标准制定对象。

## 3. 现状证据

> 注：本节描述的是改造前基线，当前仓库已有部分条目完成整改，具体进度以“0. 执行状态”为准。

### 3.1 周期输入公共骨架不一致

AR:

- [include/1q/airborne_radar/session/RadarCycleInput.h](/Users/aurora/Code/1q/include/1q/airborne_radar/session/RadarCycleInput.h) 仅含 `target_features`、`platform_pose`、`dt_sec`
- 没有 `cycle_index`

EOS:

- [include/1q/electro_optical_sensor/session/EosCycleInput.h](/Users/aurora/Code/1q/include/1q/electro_optical_sensor/session/EosCycleInput.h)
- 根级同时承载 `cycle_index`、平台位姿、太阳角、太阳辐照度、云量、风速、昼夜类型、背景温度、`scene_targets`

ESR:

- [include/1q/electronic_surveillance_radar/session/EsrCycleInput.h](/Users/aurora/Code/1q/include/1q/electronic_surveillance_radar/session/EsrCycleInput.h)
- 根级含 `cycle_index`、`platform_pose`、`scene_emitters`、`environment_observation`

结论：

- 三者没有形成同一个“周期输入骨架”
- EOS 将环境事实散落在根级，ESR 已经开始收敛为聚合子域，AR 则没有相应统一结构

### 3.2 场景实体语义层级不一致

AR:

- [include/1q/airborne_radar/model/TargetFeature.h](/Users/aurora/Code/1q/include/1q/airborne_radar/model/TargetFeature.h)
- `TargetFeature` 暴露了 `current_track_speed`、`current_track_velocity_*`、`has_cartesian_position` 等明显偏“处理中间态/跟踪态”的字段

EOS:

- [include/1q/electro_optical_sensor/session/EosCycleInput.h](/Users/aurora/Code/1q/include/1q/electro_optical_sensor/session/EosCycleInput.h)
- `EosTargetState` 更接近传感器感知所需的语义目标观测

ESR:

- [include/1q/electronic_surveillance_radar/model/EmitterTruthState.h](/Users/aurora/Code/1q/include/1q/electronic_surveillance_radar/model/EmitterTruthState.h)
- 明确命名为 `TruthState`
- 其注释也说明“仅用于生成接收机观测”

结论：

- AR 的公开输入过于靠近内部特征层
- ESR 的公开输入明确带有“真值”语义
- EOS 位于两者之间
- 三模块对“用户应该提供什么”的抽象层级并不一致

### 3.3 外部输入适配器能力不一致

AR:

- [include/1q/airborne_radar/session/RadarExternalInputAdapter.h](/Users/aurora/Code/1q/include/1q/airborne_radar/session/RadarExternalInputAdapter.h)
- 既有两步模式：
  - `TryMakeRadarPoseFromExternalKinematics(...)`
  - `TryMakeTargetFromExternalKinematics(...)`
- 也保留旧单步兼容口 `TargetExternalKinematicsInput`

EOS:

- [include/1q/electro_optical_sensor/session/EosExternalInputAdapter.h](/Users/aurora/Code/1q/include/1q/electro_optical_sensor/session/EosExternalInputAdapter.h)
- 有平台转换和目标从 `ECEF/LLA -> EosTargetState` 转换
- 无统一的“目标外部运动学输入对象”

ESR:

- [include/1q/electronic_surveillance_radar/session/EsrExternalInputAdapter.h](/Users/aurora/Code/1q/include/1q/electronic_surveillance_radar/session/EsrExternalInputAdapter.h)
- 仅有 `TryMakeEsrPoseFromExternalKinematics(...)`
- 缺少对辐射源输入实体的对应适配入口

结论：

- 三模块没有统一的“先统一平台姿态，再统一场景实体映射”的两步适配范式
- ESR 在外部实体输入适配上明显不完整
- AR 仍残留兼容性遗产

### 3.4 校验结构不一致

AR:

- [include/1q/airborne_radar/session/RadarInputValidation.h](/Users/aurora/Code/1q/include/1q/airborne_radar/session/RadarInputValidation.h)
- `ValidationIssue.target_index`

EOS:

- [include/1q/electro_optical_sensor/session/EosInputValidation.h](/Users/aurora/Code/1q/include/1q/electro_optical_sensor/session/EosInputValidation.h)
- `ValidationIssue.target_index`

ESR:

- [include/1q/electronic_surveillance_radar/session/EsrInputValidation.h](/Users/aurora/Code/1q/include/1q/electronic_surveillance_radar/session/EsrInputValidation.h)
- `ValidationIssue.emitter_index`

结论：

- 问题定位结构没有统一抽象
- 调用方跨模块消费 input 校验结果时无法形成一致处理流程

## 4. 统一标准

### 4.1 总体原则

统一后的 input 设计必须满足：

1. 外部输入只表达“外部世界可直接提供的事实或高层语义观测”
2. 不暴露内部处理中间态、缓存态、跟踪态、派生态
3. 周期输入骨架统一，模块差异只体现在子域内容，不体现在总组织方式
4. 外部适配器统一采用“两步模式”：先平台，再场景实体
5. 校验结构统一，跨模块可按相同模式消费
6. 不保留旧接口、旧字段、旧兼容路径、旧 replay 负担

### 4.2 标准一：统一周期输入骨架

三个模块都收敛为同样的外层组织：

```cpp
struct XxxCycleInput {
  std::uint32_t cycle_index{0U};
  float dt_sec{1.0f};
  oneq::foundation::PoseState platform_pose{};
  XxxSceneInput scene{};
  XxxEnvironmentInput environment{};
};
```

约束：

- `cycle_index` 成为三模块必选公共字段
- `dt_sec` 成为三模块必选公共字段
- 平台位姿统一表达为模块可消费的 pose state
- 目标/辐射源列表不再直接挂在根级，而是统一进入 `scene`
- 环境相关事实不再散落在根级，而是统一进入 `environment`

说明：

- 并不要求三个模块的 `XxxEnvironmentInput` 内容完全一样
- 但要求“结构位置一致，语义角色一致”

### 4.3 标准二：统一场景实体层级

三个模块公开给外部的场景实体必须都是“场景事实输入”，而不是内部处理中间态。

因此：

- AR 不再以 `model::TargetFeature` 作为公开输入实体
- ESR 不再以 `EmitterTruthState` 作为公开周期输入实体名
- 三模块统一采用 `Scene*` / `Observed*` / `Source*` 这类中性输入命名

推荐命名方向：

- AR: `RadarSceneTarget`
- EOS: `EosSceneTarget`
- ESR: `EsrSceneEmitter`

这些实体应满足：

- 字段只保留外部事实输入
- 禁止包含内部派生字段
- 禁止直接暴露“track”“truth”“feature”“resolved”这类内部阶段语言

明确裁剪：

- AR `TargetFeature.current_track_speed` 应从公开输入层移除
- AR `TargetFeature` 中带 `current_track_*` 语义的字段应改名/下沉到内部 model
- ESR `EmitterTruthState` 作为公开 input 名称应删除，改为中性场景辐射源输入类型

### 4.4 标准三：统一环境输入角色

环境输入在三个模块中都应表示“本周期外部提供的环境事实/观测”，而不是默认配置、内部环境模型、或散落的环境数值。

因此：

- EOS 当前根级环境字段整体收敛为 `EosEnvironmentInput`
- ESR 保留 `environment_observation` 的方向，但命名应与整体骨架统一
- AR 需要补齐与周期输入对应的环境输入聚合位

说明：

- 环境输入不要求和 `SessionConfig.environment` 一一同型
- `SessionConfig.environment` 是初始化基线
- `CycleInput.environment` 是单周期外部事实输入
- 两者职责必须严格分离

### 4.5 标准四：统一外部输入适配器范式

三个模块统一采用两步式适配：

1. `TryMake*PoseFromExternalKinematics(...)`
2. `TryMake*SceneEntityFromExternal...(...)`

统一要求：

- 不再保留单步大杂烩兼容口
- 适配器负责“坐标/参考系/姿态转换”
- 适配器不负责“内部特征派生”“内部策略补全”“内部默认值猜测”

明确裁剪：

- 删除 AR `TargetExternalKinematicsInput` 旧兼容接口
- EOS 引入明确的目标外部输入对象，而不是只暴露 `ECEF/LLA + appearance` 函数组合
- ESR 增补场景辐射源外部适配入口，不再只有平台姿态转换

### 4.6 标准五：统一校验结果模型

三个模块的输入校验统一为：

```cpp
struct ValidationIssue {
  ValidationSeverity severity;
  ValidationCode code;
  ValidationLocation location;
  std::string message;
};
```

其中：

- `ValidationLocation` 统一表达“第几个 scene entity / 哪个字段 / 是否是平台级问题”
- 不再出现 AR/EOS 用 `target_index`、ESR 用 `emitter_index` 的分裂模式

注意：

- `ValidationCode` 仍允许模块特化
- 但 `ValidationIssue` 的组织方式必须一致

### 4.7 标准六：统一命名语言

统一禁用以下作为公开 input 语言的核心名词：

- `Feature`
- `TruthState`
- `current_track_*`
- `resolved`
- `internal`

统一推荐：

- `CycleInput`
- `SceneInput`
- `EnvironmentInput`
- `SceneTarget`
- `SceneEmitter`
- `ExternalPoseInput`
- `ExternalTargetInput`
- `ExternalEmitterInput`

### 4.8 标准七：聚合层级最小化（针对单字段 struct）

`SceneInput` / `EnvironmentInput` 允许在阶段性状态下只包含一个字段，但必须满足以下条件：

1. 它是跨模块统一骨架中的固定语义槽位（`CycleInput.scene` / `CycleInput.environment`），不是临时包装。
2. 它承担明确边界职责：将“外部输入语义”与内部 model 隔离。
3. 代码中禁止再并行暴露等价根级字段，避免双路径。
4. 若后续确认该槽位长期不会扩展，且跨模块统一收益不成立，应回退为直接字段而非保留空心层。

反例（不允许）：

- 为了“看起来分层”而新增仅转发、无边界价值的壳结构。
- 公开头中保留旧别名/旧字段，同时再包一层单字段 struct。

## 5. 目标边界

### 5.1 模块公开 input 边界

公开 input 只允许包含以下四类内容：

- 周期元数据：`cycle_index`、`dt_sec`
- 平台状态：`platform_pose`
- 场景实体事实：targets / emitters
- 环境事实：太阳、天气、杂波、干扰、背景辐射等

### 5.2 严禁穿越 input 边界的内容

以下内容必须留在模块内部，不得继续作为公开 input：

- 跟踪器中间态
- feature engineering 产物
- truth/replay 专用命名
- 由库内可稳定推导出的派生量
- 针对内部算法调参的细粒度控制项

### 5.3 `config` 与 `input` 的边界

统一后必须明确：

- `SessionConfig` 负责初始化基线、默认策略、静态能力
- `CycleInput` 负责每周期外部事实
- `RuntimePatch` 负责运行期策略/模式调整

不得再出现：

- 用 `CycleInput` 偷带静态配置
- 用 `SessionConfig` 承担本周期环境事实
- 用外部适配器偷偷做策略推断

## 6. 破坏性重构方案

### 6.1 第一阶段：建立统一公开类型骨架

目标：

- 新增三模块统一骨架下的 scene/environment/input 类型
- 先让公开头文件层完成收敛

动作：

- AR:
  - 新增 `RadarSceneTarget`
  - 新增 `RadarSceneInput`
  - 新增 `RadarEnvironmentInput`
  - 重写 `RadarCycleInput`
- EOS:
  - 提取 `EosEnvironmentInput`
  - 新增 `EosSceneInput`
  - 重写 `EosCycleInput`
- ESR:
  - 新增中性 `EsrSceneEmitter`
  - 新增 `EsrSceneInput`
  - 规范 `EsrEnvironmentInput`
  - 重写 `EsrCycleInput`

这一阶段允许内部先做桥接，但桥接必须只存在于 `src/`，不得把双路径暴露回公开头文件。

### 6.2 第二阶段：删除旧公开输入语言

目标：

- 删除旧 public input 名称与兼容字段

动作：

- 删除 AR 公开 input 对 `TargetFeature` 的直接依赖
- 删除 ESR 周期输入中对 `EmitterTruthState` 的直接依赖
- 删除 AR `TargetExternalKinematicsInput`
- 删除 replay / consumer / contract test 中对旧 input 类型的依赖

说明：

- 这是破坏性重构的核心阶段
- 不接受“旧新并存”“临时别名”“兼容 typedef”

### 6.3 第三阶段：适配器统一两步化

目标：

- 三模块统一为“平台姿态 + 场景实体转换”两步模式

动作：

- AR 删除旧单步口，仅保留两步式接口
- EOS 增加 `EosExternalTargetInput`
- ESR 增加 `EsrExternalEmitterInput` 和对应转换接口
- 三模块统一状态返回语义和错误表达方式

### 6.4 第四阶段：校验与 replay 收敛

目标：

- 输入校验结构统一
- replay payload 名字和结构跟随新 input

动作：

- 统一 `ValidationIssue`
- 更新 `*ReplayFlatbufferCodec`
- 更新 `TraceSession`
- 更新 consumer / contract / unit tests

### 6.5 第五阶段：内部模型断开对旧 public input 的依赖

目标：

- 公开 input 与内部 model 彻底解耦

动作：

- 将 AR 的 `TargetFeature` 限制为内部处理类型
- 将 ESR 的 `EmitterTruthState` 限制为内部仿真/环境真值类型
- 在 `session` 或 `runtime` 层建立从新公开 input 到内部 model 的单向映射

## 7. 当前最适合的切分顺序

如果按“当前最适合”而不是“理论上最纯”推进，建议顺序如下：

1. 先做公开头文件层的统一标准落地
2. 再做 EOS 根级环境字段收拢
3. 再做 ESR `EmitterTruthState -> EsrSceneEmitter`
4. 最后做 AR `TargetFeature -> RadarSceneTarget`

原因：

- EOS 的根级环境字段收拢改动机械但收益高，适合作为统一骨架模板
- ESR 的公开命名问题最明显，尽早改名能稳定边界语言
- AR 牵涉最深，因为 `TargetFeature` 已深入验证、消费、回放和管线，适合放在标准已经清楚后做彻底切换

## 8. 理论最纯的最终状态

理论上最纯的形态是：

- 三模块都只有中性、场景事实导向的公开 input 类型
- 三模块都具有同构的 `CycleInput { cycle_index, dt_sec, platform_pose, scene, environment }`
- 三模块都具有两步式外部适配器
- 三模块都具有统一结构的校验问题模型
- `Feature` / `TruthState` 全部下沉为内部实现语言

## 9. 需要同步修改的文件范围

### 9.1 Public headers

- `include/1q/airborne_radar/session/*Input*.h`
- `include/1q/electro_optical_sensor/session/*Input*.h`
- `include/1q/electronic_surveillance_radar/session/*Input*.h`
- `include/1q/*/*ExternalInputAdapter.h`
- `include/1q/*/*InputValidation.h`

### 9.2 Internal implementation

- `src/airborne_radar/session/*`
- `src/electro_optical_sensor/session/*`
- `src/electro_optical_sensor/runtime/*`
- `src/electronic_surveillance_radar/session/*`
- `src/electronic_surveillance_radar/environment/*`
- `src/common/replay/*`

### 9.3 Tests and examples

- `tests/unit/*input*`
- `tests/consumer/*`
- `tests/contract/*public_api*`
- `examples/*`

## 10. 验收标准

完成后应满足：

1. 三模块 `CycleInput` 外层骨架一致
2. 根级不再直接暴露 `scene_targets / scene_emitters / target_features` 这类并列分裂命名
3. AR 不再以 `TargetFeature` 作为公开 input
4. ESR 不再以 `EmitterTruthState` 作为公开 input
5. EOS 根级环境数值被收拢为独立环境输入子域
6. 三模块都只保留两步式外部输入适配器
7. 校验结果结构统一
8. 旧公开类型、旧兼容接口、旧 replay 名称被删除
9. `llvm-ninja-debug-local` 下相关构建和测试通过

## 11. 建议的首个执行切口

建议从以下切口开始正式改造：

- 新增三模块 `SceneInput` / `EnvironmentInput` 聚合类型
- 重写三模块 `CycleInput` 为统一骨架
- 同步修改 unit/contract tests，先把公开契约层切到新语言

这是整个破坏性重构里最值得先做的一步，因为它直接把“input 标准”固化成可编译的公开 API。
