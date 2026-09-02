---
Status: draft
Date: 2026-09-02
Review-Baseline: `evidence/sbirs-geometry-transmittance` @ `2a4c4897`
Authority: 非规范性记录；结论以 docs/common/contract.md、docs/common/session_contract.md
  及各模块 docs/<module>/design.md 为准；与库实现冲突时以库为准。
---

# sbirs-geometry-transmittance：证据矩阵

<!-- 本文档写作规则：
1、证据一律写成一行：- **证据**：[evidence: 路径]，可加 ::符号名；禁止行号。
2、说明简要，一项一行；多个要点用 1、2、3 序号分点分行，禁止大段描述。
3、引用规则时直接写出规则内容，并用证据形式锁定来源文件；禁止写"见xx规则"。
4、面向非专业开发者，用平实中文；术语首次出现时给一句白话解释。
5、探针/测试必须是已实际执行的；无法直接验证的判断以"推理："开头标注。
-->

## §0 背景与待裁定的问题

用户观察到验收日志中"最大探测距离"逐周期不变，判断原因为目标辐射强度恒定且环境模型静态，选定迭代方向"透过率随视线几何变化"。本矩阵核实该判断并裁定改动口径。

术语（白话）：
1、透过率：光走完一段路后剩下的比例，1 = 完全没损耗。
2、d_max（最大探测距离）：拿当前检测门槛反推"最远还能看见多远"，是个诊断量，不等于目标实际距离。
3、气团因子 X：视线穿过大气的等效厚度，以"竖直向上穿过大气"为 1；斜着看时路径更长，X 大于 1。
4、大气壳顶：大气有效厚度上限，本矩阵按 100 km 取值（临近卡门线，常用口径）。

待裁定项（每项回答四问：冻结什么、什么证据证明、什么证据否定、通过后最小改动范围）：
1、透过率对视线几何无感的缺口是否成立（现状缺陷）。
2、不穿大气的"纯空间路径"是否不该扣大气衰减（物理口径）。
3、"几何修正"在旗舰场景是否只改数值、不产生时间动态（预期收窄）。
4、改动边界是否收敛在库内部管线与测试（不动公共契约）。

## §1 证据矩阵

| 待裁定项 | 假设（要证明什么） | 证据来源 | 探针/测试（已执行） | 通过条件 | 否定条件 | 建议判定 |
|---|---|---|---|---|---|---|
| 1、透过率对视线几何无感 | τ_eff 计算不吃任何几何量，同一周期所有目标共用一个值，d_max 因此对几何不响应 | [evidence: src/sbirs_sensor/environment/SbirsEnvironmentModel.cpp] ::ResolveEffectiveTransmittance；[evidence: src/sbirs_sensor/pipeline/SbirsPipeline.cpp]；[evidence: examples/log/sbirs_triple_sat_fix_messages/opir_acceptance.log] | 1、代码核对：ResolveEffectiveTransmittance 入参只有环境配置，不含位置/方向。2、管线核对：τ 每周期算一次、全部目标共用。3、当前 HEAD（2a4c4897）构建并运行旗舰场景（dual_sat_cycles=79/80），日志 320 行 d_max（4 星-目标对 × 80 周期）全部等于 114606933.3 m | τ 计算确无几何输入，且 d_max 全程恒定被实跑日志证实 | τ 已含几何修正，或 d_max 已随几何变化 | pass |
| 2、纯空间路径不该扣大气衰减 | 旗舰场景全部存活路径不穿大气壳（近地点就是目标自身高度 507~614 km），物理上透过率应为 1；当前统一扣 τ_eff=0.794 是对真空路径多扣 | [evidence: examples/log/sbirs_triple_sat_fix_messages/sbirs_sats.csv]；[evidence: examples/log/sbirs_triple_sat_fix_messages/sbirs_truth.csv]；[evidence: examples/scenes/sbirs_triple_sat_fix_messages/sbirs_triple_sat_fix_messages.json] | 1、CSV 复算：三颗 GEO 卫星（高约 35779 km）× 两目标逐周期位置，4 对存活路径近地点高度=目标高度 507~614 km，均高于 100 km 壳顶。2、复算另两对（sat104-目标2、sat204-目标1）视线穿地球，与日志中恰好无这两对的 d_max 行吻合（遮挡门先行排除）。3、场景两目标辐射强度均为 10000 W/sr，解释 d_max 两目标同值 | 存活路径全部不穿壳，"扣了不该扣的大气税"成立 | 存在存活路径近地点低于壳顶（则该路径确需大气项） | pass |
| 3、几何修正在本场景只改数值不产生时间动态 | 目标全程在壳外，几何修正后 d_max 仍为常数；要 d_max 随时间动，需要目标穿大气（弹道目标）或环境随时间变，均不在本次范围 | [evidence: examples/log/sbirs_triple_sat_fix_messages/sbirs_truth.csv] | 1、CSV 复算：目标高度全程 606~614 km 与 507~512 km，无穿壳时段。2、数值预期（数学推算，非实跑）：τ_eff=0.8×(1-0.0075)=0.794；壳段气团模型下 X=0、τ=1，d_max 变为 114606933.3/√0.794≈12861 万 m（约 +12.2%），仍为常数；首周期 SNR 85.3 升至约 107.4（÷0.794），门槛 0.001，无判不过风险 | 目标高度全程高于壳顶 → 几何修正只改数值口径 | 目标中途穿壳 → 几何修正本身即产生时间动态 | narrow（"d_max 随时间动"另立后续冻结项） |
| 4、改动边界收敛在库内部 | τ 从每周期一次（全目标共用）改为逐目标（几何因子），影响面=管线调用点+单测期望拼装+场景重跑；不动公共头与 ResolveEffectiveTransmittance 本体 | [evidence: src/sbirs_sensor/pipeline/SbirsPipeline.cpp]；[evidence: src/sbirs_sensor/foundation/SbirsRadiometry.cpp] ::ComputeMaxDetectionRangeM；[evidence: tests/unit/sbirs_sensor/sbirs_pipeline_test.cpp]；[evidence: include/1q/sbirs_sensor/config/SbirsEnvironmentConfig.h] | 1、全仓 grep：ResolveEffectiveTransmittance 生产消费仅管线一处，测试三处（环境模型/基础两处为相对比较，管线测试一处拼装 d_max 期望）。2、d_max 反解函数入参已含 τ，函数本身无需改签名 | 改动收敛在 src/sbirs_sensor 与 tests/，公共头零变更 | 需要改 include/1q 或跨模块接口 → 升级为跨模块契约流程 | pass |

补充证据（口径对比，防走错模型）：
1、若错用"目标处仰角气团"模型（X=1/sin(仰角)≈2.9），τ 将变为 0.794^2.9≈0.52、d_max 约打 8 折——但该口径对不穿大气的真空路径收大气税，物理上错。
- **证据**：[evidence: examples/log/sbirs_triple_sat_fix_messages/sbirs_los.csv]（首周期斜距 39240 km、SNR 85.3，可作修正前后对照基准）

## §2 判定汇总与待裁定问题

判定汇总：
1、冻结项 1 pass：几何无感缺陷成立，代码与实跑日志双证实。
2、冻结项 2 pass：旗舰场景全部存活路径不穿大气，统一扣 τ_eff 属于口径错误。
3、冻结项 3 narrow：本次修正数值口径；"d_max 随时间变化"在本场景无法由几何达成，需弹道目标或动态环境，另立冻结项。
4、冻结项 4 pass：改动可收敛在库内部管线与测试，公共契约零变更。

需要用户拍板的问题：
1、物理口径：是否采纳"壳段气团模型"——X=视线在大气壳内的穿壳弦长÷垂直壳厚，τ=τ_eff^X；纯空间路径 X=0 → τ=1；地面目标头顶方向 X=1（与现值完全向后兼容）。
2、数值影响接受度：本场景 d_max 114606933.3→约 12861 万 m（+12.2%）、SNR ×1.26，验收行数值变化需重跑核对（探测判定无风险，门槛 0.001 远低于 SNR）。
3、保守替代方案：空间路径维持 τ_eff、只对穿壳路径打折——在本场景等于零变化（白做），仅在"不想动现有数值"时选它。
4、大气壳顶取 100 km 是否可。
5、低仰角 X 封顶值（防地平线处发散，建议 X_max=10；本场景不触及）。
6、"d_max 随时间动"的后续路径（弹道目标场景 or 动态环境模型）本次不做、记入后续冻结项，是否可。

## §3 冻结契约（用户讨论结束后填写）

<!-- 一行一项：
1、允许范围：模块/目录、类/函数、测试与文档。
2、明确禁止范围：公开头文件、跨模块类型、schema/回放、测试阈值、兼容层。
3、行为边界：输入、输出、错误回退、生命周期。
4、验收门：构建、聚焦测试、契约测试、特征化测试。
5、非目标。
-->

## 修订记录

<!-- 编号条目（修订 1、修订 2……）：日期、裁定内容、来源（用户指令/新证据）；不静默改写既有条目。 -->

## §4 运行记录（Stage C 后填写）

<!-- 1、实现范围。
2、验证命令与结果。
3、权威回写去向：哪个结论写进了哪个文件。
4、残留风险。
5、后续冻结项。
-->
