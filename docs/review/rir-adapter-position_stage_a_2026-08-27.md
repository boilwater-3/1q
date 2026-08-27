---
Status: draft
Date: 2026-08-27
Review-Baseline: `evidence/rir-adapter-position` @ `0d03c957`
Authority: 非规范性记录；结论以 docs/common/contract.md、docs/common/session_contract.md
  及各模块 docs/<module>/design.md 为准；与库实现冲突时以库为准。
---

# rir-adapter-position：证据矩阵

<!-- 本文档写作规则：
1、证据一律写成一行：- **证据**：[evidence: 路径]，可加 ::符号名；禁止行号。
2、说明简要，一项一行；多个要点用 1、2、3 序号分点分行，禁止大段描述。
3、引用规则时直接写出规则内容，并用证据形式锁定来源文件；禁止写"见xx规则"。
4、面向非专业开发者，用平实中文；术语首次出现时给一句白话解释。
5、探针/测试必须是已实际执行的；无法直接验证的判断以"推理："开头标注。
-->

## §0 背景与待裁定的问题

1、触发：RIR 特征出口适配器（把雷达识别特征转成融合通用探测记录的那 30 行）只填了视线角和平台原点，没填位置量测；融合因此把 RIR 当成纯测向源，速度观测 σ 卡在关机点护栏之上。
- **证据**：[evidence: src/fusion/SensorAdapters.cpp]::AdaptRirFeatureMeasurementsToDetectionRecords
- **证据**：[evidence: docs/review/rir_dual_product_stage_a_2026-08-18.md]（§3.3 当时只冻结方位+原点+11 维特征）
- **证据**：[evidence: examples/scenes/rir_boost_burnout/rir_boost_burnout.md]（场景按「RIR 位置量测」写期望，与适配器现状矛盾）

术语：斜距 = 雷达到目标的直线距离；位置量测 = 融合记录里的三维坐标（经纬高）；仅方位 = 只有视线方向、没有距离，融合只能猜远近。

### 待裁定项（四问）

1、**F1 丢斜距是缺陷**：冻结「未填位置是漏填而非设计取舍」。
   能证明：出口①已有斜距且适配器单测断言 `has_position=false`。
   能否定：斜距语义不是平台到目标，或融合位置通道本就不接收雷达类源。
   最小范围：适配器填 `has_position`。

2、**F2 位置通道是运动学失效的原因**：冻结「RIR 场景 σ_v 卡在 20~40 m/s、关机点不可判，根因在融合吃不到位置，不在滤波算法或传感器」。
   能证明：引擎对 `has_position` 走 50 m 笛卡尔更新并提前返回；仅方位走弱可观测路径；关机护栏 5 m/s。
   能否定：带位置的融合航迹 σ_v 仍 ≥ 5 m/s（则根因在过程噪声/量测模型，不是适配器）。
   最小范围：适配器接通已有位置通道，不改滤波。

3、**F3 不必扩出口①、不必改融合引擎**：冻结「三字段（斜距+视线角+平台原点）在适配器内即可还原位置」。
   能证明：出口①三字段有单测；库已有 ENU→ECEF→LLA；`DetectionRecord.has_position` 已存在。
   能否定：斜距与视线角坐标系对不上，或还原必须新公开字段。
   最小范围：`SensorAdapters.cpp` + 适配器单测 + fusion/RIR 文档回写。

4、**F4 填位置后仍保留方位**：冻结「位置与方位并存；滤波已位置优先，去掉方位会伤跨源仅方位关联」。
   能证明：滤波 `has_position` 提前返回；无身份关联的方位路径要求航迹锚点带方位。
   能否定：双通道会双重计量（与引擎实现不符）。
   最小范围：只加位置，不删现有方位/原点映射。

5、**F5 噪声走融合默认 50 m，不开极坐标量测模型**：冻结「与 AR 同路径；记录级位置噪声仍是后续项」。
   能证明：AR 适配器 `has_position=true`；引擎 `R = default_position_noise_std_m`（默认 50 m）；算法文档已登记记录级噪声为后续冻结项。
   能否定：50 m 各向同性使 Stage B 场景仍过不了关机护栏。
   最小范围：不改 `FusionConfig`、不改更新器。

## §1 证据矩阵

| 待裁定项 | 假设（要证明什么） | 证据来源 | 探针/测试（已执行） | 通过条件 | 否定条件 | 建议判定 |
|---|---|---|---|---|---|---|
| F1 丢斜距是缺陷 | 出口①已带齐还原三维位置的三字段；适配器只取角、扔掉斜距；这不是 2026-08-18 冻结的有意终态，是漏填 | `RirFeatureMeasurementTypes.h` 的 `range_m` / `look_*` / `platform_position`；适配器只写 `has_bearing`+`sensor_origin`；`sensor_adapters_test` 断言 `has_position=false` | 1、读出口①组装：`record.range_m = context.range_m`，成功周期 `has_platform_position=true`。2、读适配器：无 `has_position=true` 赋值。3、读单测 `RirMeasurementsAdaptWithElevenDimensionLayout`：`EXPECT_FALSE(detection.has_position)`。4、读出口单测 `RirFeatureMeasurementTest`：`EXPECT_FLOAT_EQ(record.range_m, target.range_m)` 且平台 ECEF 恒透出 | 三字段在出口①真实存在且适配器未映射位置 | 斜距不是平台到目标的直线距离，或适配器本就不该出位置（与 AR 先例、场景期望矛盾） | pass |
| F2 位置通道是运动学失效的原因 | 融合引擎位置通道现成且 AR 在用；RIR 走仅方位 → 距离弱可观测 → σ_v 高于关机护栏 5 m/s | `FusionEngine` 滤波；`FusionConfig::default_position_noise_std_m`；关机护栏；`rir_boost_burnout.md` | 1、读 `ApplyFilterSample`：`has_position` 则 LLA→ECEF + `UnscentedUpdater<6,3>`，`R=default_position_noise_std_m`（50 m），随后 `return`（不走方位更新）。2、读 `InitFilter`：有位置用真位置零速起始；仅方位+原点沿视线用 `track_bearing_init_range_m`（默认 100 km）+ `track_bearing_init_range_std_m`（默认 300 km）。3、读 `UpdateBurnoutTracker`：`velocity_sigma_m ≥ 5` 不推进。4、读 `rir_boost_burnout.md`：明文「本场景用位置量测（RIR 雷达）」对照仅方位场景 σ_v≈20~24 m/s。5、用户 2026-08-26 无干扰演示实测 σ_v≈26 m/s（与仅方位地板同量级） | 引擎位置路径独立可走；当前 RIR 记录进不了该路径；关机护栏因此挡住 | 带位置的融合航迹协方差速度对角仍 ≥ 5 m/s（根因在 q/`R`，须另开冻结项） | pass |
| F3 不必扩出口①、不必改引擎 | 斜距+雷达局部 ENU 视线角+平台 ECEF，经已有坐标变换即可得到融合要的经纬高；无需新公开字段、无需极坐标量测模型 | `RirController::ComputeLookAngles`；`TryEnuToEcef` / `TryEcefToLla`；`DetectionRecord.has_position` | 1、读 `ComputeLookAngles`：`az=atan2(y,x)`（自东）、`el=atan2(z,hypot)`、斜距=目标 `range_m` 或位置模长——三量与局部 ENU 位置互逆。2、读 `position_transform.h`：已有 `TryEnuToEcef`（局部东-北-天 → 地心地固）。3、`TryBearingRangeToEnuOffset` 只做水平距离且方位自北，**不能**拿来当斜距还原。4、`DetectionRecord` 已有 `has_position`+`position`（经纬高） | 适配器内可写完还原；公开头与融合更新器零变更 | 视线角参考系与 ENU 轴对不上，或还原必须改出口①/引擎 | pass |
| F4 填位置后仍保留方位 | 双通道不双重计量：滤波位置优先；RIR 靠身份键直挂，方位留给跨源仅方位关联 | `FusionEngine::Update` 身份键直挂；`AssociateUnidentified`；`ApplyFilterSample` | 1、读 `Update`：`key ≠ 0` 的探测按键挂航迹（RIR `association_key` 走此路，不靠方位关联）。2、读 `AssociateUnidentified`：方位路径要求 `track.anchor.has_bearing`；若适配器去掉方位，SBIRS/EOS 等无身份仅方位源无法与 RIR 航迹做方位相干。3、读 `ApplyFilterSample`：有位置则更新后 `return`，方位噪声不会再喂一次 | 保留方位不造成双重更新；去掉方位会切断跨源仅方位挂靠 | 引擎会对同一记录先位置再方位各更新一次（与代码不符） | pass |
| F5 噪声走默认 50 m | 与 AR 同路径；不在本次改量测噪声模型 | AR 适配器；`FusionConfig`；`docs/fusion/algorithms.md` §4 | 1、读 `AdaptArTracksToDetectionRecords`：`has_position=true`，无记录级噪声字段。2、读 `FusionConfig::default_position_noise_std_m{50.0}`。3、读算法文档：「记录级位置噪声通道为后续冻结项，当前仅配置默认」。4、读出口①注释：仿真角度无噪声、斜距来自场景几何 | 复用配置默认即可接通；不改 `FusionConfig`/更新器 | Stage B 场景在 50 m 路径下 σ_v 仍 ≥ 5 m/s → 撤回本项，改开极坐标 `R` 或过程噪声 | pass |

## §2 判定汇总与待裁定问题

### 建议判定

1、**F1 pass**：出口①有斜距、适配器单测锁死 `has_position=false`，是漏填。
2、**F2 pass**：引擎位置通道现成；RIR 进不去；关机护栏因此失效。场景文档已按位置量测写期望。
3、**F3 pass**：还原落在适配器；不扩出口①、不改融合引擎。
4、**F4 pass**：位置与方位并存；只加不删。
5、**F5 pass**：走 AR 同款 50 m 默认噪声；极坐标量测模型仍是后续项。

### 推理（非探针直接值）

1、推理：50 m 位置、1 Hz、过程噪声 q=1 的 CV 滤波，稳态 σ_v 通常低于关机护栏 5 m/s；仅方位 300 km 距离先验做不到。精确 σ_v 以 Stage B 场景/单测协方差对角为准。
2、推理：用户 `rir_boost_burnout` 实测 σ_v≈26 m/s 与仅方位地板一致，作为 F2 旁证，不是本仓库内已提交的日志夹具。

### 建议 Stage B 范围（用户裁定前不写入 §3）

1、允许：`src/fusion/SensorAdapters.cpp`、`include/1q/fusion/SensorAdapters.h` 注释、`tests/unit/fusion/sensor_adapters_test.cpp`、`docs/fusion/algorithms.md`、RIR 设计文档中适配器映射句。
2、禁止：改 `DetectionRecord` / 出口①字段 / `FusionConfig` / `FusionEngine` 更新器 / replay schema / 关机护栏阈值。
3、行为：斜距有限且 >0、平台原点 ECEF→LLA 成功时，按 `ComputeLookAngles` 的逆运算还原局部 ENU，再 `TryEnuToEcef`→`TryEcefToLla` 填 `has_position`；失败则维持今日仅方位+原点。方位/特征/原点映射不变。
4、验收：`unit::fusion` 适配器新用例（还原精度、斜距非法/无原点/地心退化、`has_position` 与方位并存）；`unit::remote_identification_radar` 回归；Stage B 后用 `rir_boost_burnout` 看 σ_v 是否落到护栏下、关机确认分支是否出现。

### 请用户拍板

1、F1–F5 五条建议判定是否全部采纳？
2、Stage B 是否收在适配器（F3），还是改为出口①增发航迹 ECEF（更像 AR，但要改公开类型）？
3、F5：本次是否坚持 50 m 默认？若坚持，Stage B 场景若 σ_v 仍 ≥ 5 m/s 则停工回 Stage A，不开「顺便改 q/`R`」。
4、讨论结束后是否按上表冻结 §3，再从本分支 fork `feat/rir-adapter-position`？

## §3 冻结契约（用户讨论结束后填写）

<!-- 一行一项：
1、允许范围：模块/目录、类/函数、测试与文档。
2、明确禁止范围：公开头文件、跨模块类型、schema/回放、测试阈值、兼容层。
3、行为边界：输入、输出、错误回退、生命周期。
4、验收门：构建、聚焦测试、契约测试、特征化测试。
5、非目标。
-->

## 修订记录

（尚无。用户裁定后按编号追加，不静默改写矩阵建议判定。）

## §4 运行记录（Stage C 后填写）

<!-- 1、实现范围。
2、验证命令与结果。
3、权威回写去向：哪个结论写进了哪个文件。
4、残留风险。
5、后续冻结项。
-->
