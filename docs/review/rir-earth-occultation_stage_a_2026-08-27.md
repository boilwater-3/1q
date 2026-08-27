---
Status: final
Date: 2026-08-27
Review-Baseline: `evidence/rir-earth-occultation` @ `e1a0a859`
Authority: 非规范性记录；结论以 docs/common/contract.md、docs/common/session_contract.md
  及各模块 docs/<module>/design.md 为准；与库实现冲突时以库为准。
---

# rir-earth-occultation：证据矩阵

<!-- 本文档写作规则：
1、证据一律写成一行：- **证据**：[evidence: 路径]，可加 ::符号名；禁止行号。
2、说明简要，一项一行；多个要点用 1、2、3 序号分点分行，禁止大段描述。
3、引用规则时直接写出规则内容，并用证据形式锁定来源文件；禁止写"见xx规则"。
4、面向非专业开发者，用平实中文；术语首次出现时给一句白话解释。
5、探针/测试必须是已实际执行的；无法直接验证的判断以"推理："开头标注。
-->

## §0 背景与待裁定的问题

1、触发：用户确认 RIR 探测链没有地球通视/遮挡门，要求按 SBIRS 同口径补几何遮挡硬门。
- **证据**：[evidence: src/remote_identification_radar/runtime/RirController.cpp]
- **证据**：[evidence: src/sbirs_sensor/foundation/SbirsGeometry.h]::IsEarthOcculted

术语：地球通视/遮挡 = 雷达到目标的直线是否被地球挡住；挡住则不能探测。
术语：有限弦 = 只看「从平台到目标这一段」，不把射线往两端无限延长。
术语：k 因子 = 大气折射用的有效地球半径倍数，缺省 4/3，RIR 只拿它算大气损耗。

### 待裁定项（四问）

1、**F1 需求成立**：冻结「RIR 远距探测没有穿地排除，可对不可见目标出检测」。
   能证明：候选循环只有角域体积门与 SNR/检测门，没有球体求交。
   能否定：库内已有等价遮挡门（与探针不符）。
   最小范围：候选循环加一道硬排除，不改 SNR 公式。

2、**F2 口径=几何圆球、不含 k**：冻结「与 SBIRS 同：圆球半径 6371 km，相切算遮挡；k 不进本门」。
   能证明：用户裁定几何遮挡；k 只进 Blake 损耗；用 k·R 当遮挡球会把真实 ECEF 站点判进球内。
   能否定：合同要求电波视距/4/3 通视（本次未给出）。
   最小范围：复用 SBIRS 有限弦-圆球判定。

3、**F3 核进 common**：冻结「判定核不能放在 SBIRS 内部头，RIR 禁止依赖 SBIRS」。
   能证明：RIR 独立装备、不 include AR/SBIRS 头；两模块同相切约定。
   能否定：RIR 已可合法调用 SBIRS 内部几何（与独立装备边界不符）。
   最小范围：只迁 `IsEarthOcculted` + `ComputeEarthOccultationMarginM`。

4、**F4 插入点在体积门与 SNR 之前**：冻结「穿地目标不入检测候选、不算回波」。
   能证明：SBIRS 遮挡在视场/距离/SNR 之前；RIR 体积门是转台角域不是通视。
   能否定：通视应在 SNR 之后用损耗软杀（与硬门需求不符）。
   最小范围：`id==0` 之后、可扫描体积之前 `continue`。

5、**F5 公开 API 只加 issue code**：冻结「加性 `rir.target_earth_occulted`；无新配置/输入字段」。
   能证明：正常周期排除走 kInfo 诊断，code 是机器键；SBIRS 同形 `sbirs.target_occulted`。
   能否定：必须改周期输入或 replay schema 才能判定（坐标已具备：平台 ECEF + 目标 ENU）。
   最小范围：`RirIssueCodes.h` + `issue_codes.md`。

6、**F6 AR 不同步**：冻结「本次不改机载雷达」。
   能证明：用户范围是 RIR；AR 同样没有遮挡硬门，但是另一装备。
   能否定：合同要求 AR 同步（本次未给出）。
   最小范围：RIR 接线 + common 核 + SBIRS 薄封装。

## §1 证据矩阵

| 待裁定项 | 假设（要证明什么） | 证据来源 | 探针/测试（已执行） | 通过条件 | 否定条件 | 建议判定 |
|---|---|---|---|---|---|---|
| F1 需求成立 | RIR 候选循环没有地球遮挡硬门，穿地目标只要过角域和 SNR 就会进检测 | `RirController.cpp` 候选循环；`RirIssueCodes.h` 排除码全集；`algorithms.md` 登记表 | 1、`src/remote_identification_radar` 全文无 `IsEarthOcculted`/`EarthOccult`/`target_earth_occulted`。2、排除码只有体积外/检测门/超识别距/非识别模式/无特征库。3、候选循环：`id==0` → 视线角 → `TargetWithinSteerableVolume` → `TryBuildMeasurement`。4、算法登记无遮挡门，非目标清单也未把通视写成刻意不做 | 确认没有穿地硬排除 | 已有等价遮挡门 | pass |
| F2 口径=几何圆球、不含 k | 用户要 SBIRS 同款有限弦-圆球；k 只服务大气损耗 | 用户裁定；`SbirsGeometry`；`RirController` k 因子；`blake_atmos_loss` | 1、用户选择「几何遮挡（推荐）」而非电波视距。2、SBIRS：`ComputeEarthOccultationMarginM` 有限弦最近接近距离 − 半径，`<=0` 遮挡（相切算遮挡）。3、半径 `kEarthRadiusM=6371000`。4、RIR `effective_k_factor_` 只写入 `AtmosphericObservationRef.k_factor` 再进 Blake 损耗。5、Blake 用 k 除路径损耗，不拒绝超视距 | 本门不读 k、不放大地球半径 | 必须做 4/3 电波视距（本次未给出） | pass |
| F3 核进 common | RIR 不能依赖 SBIRS 内部头；两模块必须同判定否则相切约定会漂 | `design.md` 独立装备；`SbirsGeometry.h` 非 public | 1、RIR `design.md`：不 include 任何 AR 头，与 AR 无模块间接口。2、`IsEarthOcculted` 声明在 `src/sbirs_sensor/foundation/`，RIR 源码零引用。3、SBIRS 已有单测：对侧穿地 / 同侧外向 / 余量正负 | 核放到 `src/common/geometry`，SBIRS 改薄封装 | 允许 RIR include SBIRS 内部头（违反独立装备） | pass |
| F4 插入点在体积门与 SNR 之前 | 穿地是物理不可见，应先于转台角域和回波计算 | `SbirsPipeline` 门序；`RirController` 体积门 | 1、SBIRS：`IsEarthOcculted` 在 WFOV 视场和距离门之前。2、RIR 体积门注释：角域裁剪，不是通视。3、正常周期排除写 kInfo，不属于三写 | 遮挡 `continue` 发生在体积检查之前 | 遮挡只作 SNR 软损耗 | pass |
| F5 公开 API 只加 issue code | 平台 ECEF + 目标 ENU 已能还原目标 ECEF；排除走既有 issues 列表 | `RirCycleInput`；`TryEnuToEcef`；规则 13b | 1、平台 ECEF 必填 fail-closed。2、场景目标是雷达局部 ENU，适配层 ECEF→ENU。3、公共 `TryEnuToEcef` 已存在。4、`session_contract`：按目标门控排除写 kInfo，code 带模块前缀；具体门 `cause=kNone`。5、SBIRS 先例 `sbirs.target_occulted` + 余量进 message | 只加 `rir.target_earth_occulted` | 必须改输入 DTO 或 replay 表 | narrow |
| F6 AR 不同步 | 本次范围是 RIR；AR 没有遮挡硬门但不在本变更 | `src/airborne_radar`；用户范围 | 1、`include/1q/airborne_radar` 无 `IsEarthOcculted`/`EarthOccult`。2、用户指令与计划非目标写明 AR/ESR/EOS 不同步 | 不改 AR | 合同要求三模块一起改（本次未给出） | pass |

## §2 判定汇总与待裁定问题

### 建议判定

1、**F1 pass**：RIR 没有地球遮挡硬门。
2、**F2 pass**：几何圆球，不含 k。
3、**F3 pass**：判定核进 common，SBIRS 薄封装。
4、**F4 pass**：体积门和 SNR 之前排除。
5、**F5 narrow**：公开面只加 issue code。
6、**F6 pass**：不改 AR。

### 用户裁定（2026-08-27）

1、遮挡模型选几何圆球（与 SBIRS 同口径），不选电波视距。
2、用户要求按计划实施，上述建议判定全部采纳，写入 §3。

### 推理（非探针直接值）

1、推理：近距 ENU 测试（十公里、正高度）还原到 ECEF 后弦在球外，不应被误挡。
2、推理：`rir_long_range_scan` 俯仰约 20°、高度 1100 km / 2900 km，弦不穿地，场景应仍探测成立。
3、推理：ENU→ECEF 失败时不做本门（保持可检测），避免把坐标失败写成「被地球挡住」。

## §3 冻结契约

Proven requirement:
- RIR 对穿地视线做硬排除，判定与 SBIRS 相同：有限弦-地球圆球，半径 6371000 m，相切算遮挡。

Allowed scope:
- Modules/directories: `src/common/geometry/`（新核）、`src/sbirs_sensor/foundation/`（薄封装）、`src/sbirs_sensor/pipeline/SbirsPipeline.cpp`（半径常量引用）、`src/remote_identification_radar/runtime/RirController.cpp`、`include/1q/remote_identification_radar/session/RirIssueCodes.h`
- Tests: `tests/unit/common/common_earth_occultation_test.cpp`；`tests/unit/remote_identification_radar/rir_earth_occultation_test.cpp`；既有 `sbirs_foundation_test` 遮挡用例保持通过
- Docs: RIR `algorithms.md`/`boundaries.md`/`data-flow.md`；`docs/common/issue_codes.md`；SBIRS `algorithms.md` 一句核在 common；本文 §4

Explicitly out of scope:
- Public headers: 除加性 issue code 外不新增公开几何 API、不新配置项、不新周期输入字段
- Cross-module types: 不引入 SBIRS/AR 类型到 RIR
- Schema/trace/replay: 不改 flatbuffers
- Test thresholds/skips: 不放宽既有门限
- Compatibility layers: 无
- AR / ESR / EOS / 地形 / WGS84 椭球遮挡 / 4/3 电波视距 / 遮挡开关

Behavior boundary:
- Inputs: 平台 ECEF + 目标雷达局部 ENU（经 `TryEcefToLla` + `TryEnuToEcef` 还原目标 ECEF）
- Outputs: 遮挡目标不入检测候选；`rir.target_earth_occulted` kInfo，`cause=kNone`，message 带 `target_id` 与 `occultation_margin_m`
- Errors/fallback: 平台 LLA 或 ENU→ECEF 失败则本目标跳过遮挡门（不伪装成遮挡）；平台 ECEF 非法仍走既有整周期校验拒绝
- Lifecycle/debug/trace: 排除记录器按既有 `(code,cause)` + `entity_index` 差分，不新增记录器

Acceptance gates:
- Build: `1q_common_unit_tests` / `1q_remote_identification_radar_unit_tests` / `1q_sbirs_sensor_unit_tests` Release
- Focused tests: common 穿地/外向/余量/相切/对地目标；RIR 遮挡无量测、近距可见、遮挡优先于体积门
- Contract tests: 不加新契约测试（无 schema/公开 DTO 变更）
- Characterization tests: 可选 `rir_long_range_scan` 双目标仍探测

Non-goals:
- k 因子进入遮挡球半径
- 椭球、地形、大气波导
- 示例新场景（除非回归失败再改预期表）

## 修订记录

1、修订 1（2026-08-27，用户指令）：遮挡模型定为几何圆球，与 SBIRS 同口径，不是 4/3 电波视距。
2、修订 2（2026-08-27，用户指令）：按计划实施，F1–F6 建议判定全部采纳并冻结 §3。

## §4 运行记录

1、实现范围：`src/common/geometry/EarthOccultation.{h,cpp}` 判定核；SBIRS `SbirsGeometry` 薄封装 + 管道半径引用 common 常量；RIR 候选循环在体积门之前排除穿地目标，加性 code `rir.target_earth_occulted`；单测 common/RIR；权威文档回写。
2、验证命令与结果：
   1、`scripts/1q.sh build VisualStudio.15.0-amd64-release --target 1q_common_unit_tests`：pass
   2、`scripts/1q.sh build VisualStudio.15.0-amd64-release --target 1q_remote_identification_radar_unit_tests`：pass
   3、`scripts/1q.sh build VisualStudio.15.0-amd64-release --target 1q_sbirs_sensor_unit_tests`：pass
   4、`scripts/1q.sh test VisualStudio.15.0-amd64-release -R "unit::(common|remote_identification_radar|sbirs_sensor)" -j 4`：3/3 pass（common 0.11 s，sbirs 0.22 s，rir 6.05 s）
   5、`rir_long_range_scan` 场景编失败：`examples_core` PCH 找不到 `Eigen/Core`（既有示例 PCH 问题，非本变更）
3、权威回写去向：
   1、RIR 算法登记/非目标 → `docs/remote_identification_radar/algorithms.md`
   2、RIR 模块边界地球遮挡条款 → `docs/remote_identification_radar/boundaries.md`
   3、候选裁剪顺序 → `docs/remote_identification_radar/data-flow.md`
   4、issue code → `docs/common/issue_codes.md`（`rir.target_earth_occulted`；顺带补登记已落地的 `rir.target_outside_search_volume`）
   5、SBIRS 判定核在 common → `docs/sbirs_sensor/algorithms.md`
4、残留风险：圆球半径与 WGS84 椭球 ECEF 混用，掠地平线附近可能有公里级余量偏差；近距正高度 ENU 场景不受影响。
5、后续冻结项：AR 地球遮挡同步；电波视距/椭球/地形通视（本次非目标）。
