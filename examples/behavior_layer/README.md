# 行为层参考实现（`examples/behavior_layer/`）

消费方业务层的 EnTT ECS 参考实现（冻结契约：`docs/review/Bahavior.md` §5）。
实体/组件装配由 `entt::registry` 承担，逻辑以纯数据组件 + 自由函数系统表达；
**EnTT 仅为 example 侧依赖**（`conanfile.py` 基础清单，header-only），不进入库本体，
`include/1q/` 公共头与 C++11 下限均不受影响。

当前形态：**三传感器（AR / ESR / EOS）单平台端到端全链**——三会话同周期推进、
输出在边界适配为泛型探测记录、一次融合引擎更新产出跨源合并态势、决策下发命令帧、
ECM 输入帧由 ESR 观测填充。

## 目录结构

| 文件 | 职责 |
| --- | --- |
| `components.h` | 六个数据组件（纯数据、无虚函数）+ 源通道标识常量 |
| `systems.h` / `systems.cpp` | 四个系统（自由函数，`entt::registry&`）+ 三会话驱动与适配 |
| `assembly.h` / `assembly.cpp` | 装配：registry ctx 上下文、实体创建、周期调用序、观察者工厂 |
| `behavior_layer_demo.cpp` | 主程序：脚本化场景 + 每周期系统调用序 + 事件报告 |

## 组件（`components.h`）

| 组件 | 内容 | 写入方 |
| --- | --- | --- |
| `TaskingComponent` | 角色（单机/长机/僚机）、上下级、区域任务（`navigation` 面类型） | 装配层（层级显式注入，无"发现"机制） |
| `SensorObservationComponent` | `source_id` + 泛型探测记录（`fusion::DetectionRecord`） | `recon_system`（每传感器实体一份） |
| `FleetStatusComponent` | 平台 LLA 位置/航向/速度 | 消费方聚合注入（demo 每周期同步三传感器实体） |
| `RoutePlanComponent` | 航路计划（`navigation::RoutePlan`）+ 版本号 | `maneuver_system` |
| `FusedSituationComponent` | 融合态势 + 新/消失事件计数 | `recon_system`（经 `fusion` 引擎） |
| `CommandFrameComponent` | AR 战术指令 / ECM 周期输入 / 外部决策覆盖 | `jam_system` + `decision_system` |

实体挂载：长机（任务/航路/融合/命令帧/编队状态）+ 2 僚机（任务/航路/编队状态）+
**三传感器实体**（`SensorObservationComponent` + `FleetStatusComponent`，
`source_id` = 1/2/3 区分通道）。组件类型零新增——多传感器接入靠实体拆分与
`source_id` 分派，不靠新组件类型。

## 系统与周期调用序

每周期由 `StepBehaviorLayer()` 按以下顺序执行（对齐 session `Step` 语义）：

1. **`recon_system`** — 按实体 `source_id` 分派三会话（同周期同时间戳，多源时间
   对齐即业务层职责）：
   - **AR**：`StepWithResult` → `ArCycleOutputAdapter` → `DetectionRecord`
     （key = `association_key`，带位置）；
   - **ESR**：`StepWithResult` → `EmitterHypothesisList` 适配
     （key = `hypothesis_id`，方位 + 归一化射频特征 {GHz/MHz/ms/µs}，
     quality = 假设置信度）；
   - **EOS**：`EosCycleInputAdapter::Build` → `StepWithResult` → `EosDetectionRecord`
     适配（key = 0 无身份，**仅方位通道**，quality = SNR 归一化；range 通道首期
     不使用，保留方位相干关联路径）；
   
   三实体探测聚合后一次 `FusionEngine::Update` 更新融合态势；
2. **`maneuver_system`** — 层级纯函数：有上级 → 零计算；长机/单机调
   `AreaCoveragePlanner::Plan` 规划区域覆盖航路；长机把计划下发到各僚机；
3. **`jam_system`** — 构造 `EcmCycleInput`，ESR 成功周期发布去真值化观测帧时
   填充 `sensor_observation_frame`（`source_esr_batch_id` = ESR 批次号；观测字段
   业务层映射：RF 参数五件套直映、`EsrObservationQuality` → 威胁分数、
   SNR → 置信度）。无逐威胁 tasking SPI（冻结契约 §5/§7）；
4. **`decision_system`** — 聚合融合态势（高置信威胁判定）产出 `ArCommand`，
   写入命令帧组件；`external_decision` 为预留接线位。

事件模型：命令 = 写 `CommandFrameComponent`；事件报告 = `entt::observer`
（`MakeSituationObserver`，监听 `FusedSituationComponent` 变化），不建全局事件总线。

## 多源关联路径（demo 实际演示的融合语义）

| 关联层 | demo 中的表现 |
| --- | --- |
| ① 身份键直挂 | AR `association_key` 与 ESR `hypothesis_id` 均为从 1 递增的库内键，同物理目标恰好同键 → 同键探测归并（`key=1/2/3` 双通道航迹，置信度 = Σ 判决×质量×权重 跨源累加）。跨源一致性由调用方保证（冻结契约 §4.1）——demo 中两模块键空间巧合对齐 |
| ② 位置半径（KD-tree） | AR 轨迹带位置；同周期内多 AR 航迹互不合并（冻结：同周期新建航迹不参与关联） |
| ③ 方位相干 | EOS 无身份仅方位探测 → 与上一周期已存在的 ESR 方位航迹做 `AreBearingsCoherent`（方位最短弧 + 俯仰线性，均 < 门限 8°）→ 并入（`ch[2:x 3:y]` 双通道）；不相干时生成合成键（≥ 2^63）新航迹，失跟超限后删除 |
| ④ 特征门限 | 未启用（ESR 记录携带特征但异构维度下引擎本就不约束，不虚设门） |

**已知模块行为（非融合缺陷）**：ESR 假设关联器会随时间换发假设 id（3 个辐射源
产生 5-8 条假设航迹），融合航迹随之轮换——这是 ESR 模块自身的假设管理行为；
EOS 探测只在其扫描经过目标时产生（每 4 周期约 1-2 次），并入当周期对应的 ESR
航迹。航迹锚点 = 最近一次量测（任意源），方位比较针对该锚点。

**坐标参考**：三会话共享零姿态平台局部系（`atan2(north, east)`，az 0 = 东），
因此 ESR 假设方位与 EOS 探测方位可直接相干比较。

## 接线位（执行面驱动）

| 执行面 | 接线 | 状态 |
| --- | --- | --- |
| AR | `ArSession::SubmitExternalDecision` / `decision_observation` | 已演示（命令帧输出；外部覆盖为预留位） |
| ECM | `EcmCycleInput`（`sensor_observation_frame` 由 ESR 观测填充） | 已演示（帧构造 + 观测映射；ECM 会话驱动属消费方职责） |
| ESR | 会话输出 → `DetectionRecord`（source_id=2） | 已接入 |
| EOS | 会话输出 → `DetectionRecord`（source_id=3，仅方位） | 已接入 |
| navigation | `AreaCoveragePlanner::Plan` → `RoutePlanComponent` | 已演示 |
| fusion | `FusionEngine::Update` → `FusedSituationComponent` | 已演示（跨源合并） |
| flight_dynamic | 消费方将 `RoutePlanComponent` 适配为 `FlightManager` 指令 | 未接入（模块默认 OFF，驱动属消费方职责） |

## 场景调参说明（业务层决策，共享 JSON 未改动）

- **平台 700 m 高度**：使 EOS 探测距离窗（`alt/sin(视轴±半视场)`）落在
  [10, 40] km，与目标斜距（12-14 km）匹配；
- **EOS 会话配置覆写**（`configs.eos.mission`，仅内存）：`frame_rate_hz` 30→10
  （周期校验要求 dt ≤ 10/帧率，演示按 1 s/周期推进）；扫描几何从"下视地面监视
  （az ±10°、俯仰中心 -48°）"覆写为"水平扫描（az ±40°、20°/s）"以匹配空中目标；
- **ESR 辐射源用脉冲列波形**（200 脉冲 @ 9.5/10.0/10.5 GHz 互异频率）：噪声波形
  有效脉冲数为 1，在 `pfa=1e-6` 统计检测门限下无法过检；互异频率保证分选聚簇
  稳定分离 3 条辐射源假设。

## 构建与运行

```bash
bash scripts/bootstrap_conan.sh llvm-ninja-release-local   # 拉取 entt/3.14.0
cmake --preset llvm-ninja-release-local -DENABLE_EXAMPLES=ON
cmake --build --preset llvm-ninja-release-local --target behavior_layer_demo
./build/llvm-ninja-release-local/bin/behavior_layer_demo
```

启用 `BUILD_TESTING` 时注册冒烟测试 `examples::behavior_layer_demo`
（LABELS：`examples;behavior_layer`），验证 EnTT 依赖链与三传感器端到端全链。

## 演进路线

ECS 组件/系统模式覆盖了 session_usage（API 教程）与 scene（端到端场景）类目的
职责，将逐步取代现有 per-domain 示例；三传感器接入后，electronic_warfare 与
electro_optical 旧示例的侦察/干扰内容已被本示例覆盖。旧示例在迁移完成前保留，
本轮不迁移。
