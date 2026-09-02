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

已证明的需求：
1、透过率对视线几何无感是口径缺陷：不穿大气的路径被统一扣 τ_eff，旗舰场景 320 行 d_max 全等证实。
2、修正口径=壳段气团模型（修订 1）：只有真穿过大气的那段路才计衰减。

允许范围：
- 模块/目录：src/sbirs_sensor（environment 层几何因子 + pipeline 逐目标 τ 装配）、tests/unit/sbirs_sensor、docs/sbirs_sensor、examples 场景验收读数更新。
- 类/函数：environment 层新增气团因子函数（穿壳弦长÷垂直壳厚）；SbirsPipeline.cpp 逐目标 τ_geo 替换现每周期共用 τ；相关注释同步。
- 测试/文档：几何因子单测新增；管线测试期望拼装同步；场景重跑读数更新；模块权威文档回写。

明确禁止范围：
- 公开头文件：include/1q 零变更（SbirsEnvironmentConfig、输出类型、门限均不动）。
- 跨模块类型、schema/回放：不动。
- 测试阈值/跳过：不放宽。
- 兼容层：不加开关、不留旧口径分支。
- 弹道目标场景、动态环境模型：后续冻结项（修订 3）。

行为边界：
- 输入：卫星/目标 ECI 位置（管线已有）、SbirsEnvironmentConfig（不变）、地球半径取库内现有常数、壳顶 +100 km（修订 4）、X 封顶 10（修订 4）。
- 输出：逐目标 τ_geo = τ_eff^X；SNR/d_max/衰减归因改用 τ_geo；本场景四对 X=0 → τ=1，d_max ×√(1/0.794)≈1.1222（114606933.3→约 1.2861 亿米）。
- 错误回退：穿地路径由既有遮挡门先行排除（不进气团函数）；X 夹取 [0,10]。
- 生命周期/调试/回放：d_max 验收行数值更新、行格式不变。

验收门：
- 构建：Release 全绿。
- 聚焦测试：sbirs 单测全绿；新增几何因子单测覆盖纯空间 X=0、地面头顶 X=1、目标在壳内 X<1、低仰角封顶 10、双端在壳外但路径穿壳。
- 契约测试：公共 API 白名单测试不变（include/1q 零变更）。
- 特征化：旗舰场景重跑——d_max 四对全等且等于旧值×1.1222、dual_sat_cycles=79/80 不回归。

非目标：
- 不让 d_max 产生时间动态；不做天气时间演化；不改 RIR 模块；不动 examples 层 CSV/查看器命名。

## 修订记录

修订 1（2026-09-02，用户裁定）：采纳壳段气团模型——X=视线在大气壳内穿壳弦长÷垂直壳厚，τ=τ_eff^X；不穿大气路径 X=0→τ=1；地面目标正头顶 X=1 向后兼容。来源：用户选项"采纳壳段气团模型（推荐）"。
修订 2（2026-09-02，用户裁定）：接受数值影响——d_max/SNR 验收行数值变化，重跑核对并更新读数。来源：用户选项"接受，重跑核对（推荐）"。
修订 3（2026-09-02，用户裁定）："d_max 随时间变化"的弹道目标/动态环境路径本次不做，记入后续冻结项。来源：用户选项"可以，本次只修口径（推荐）"。
修订 4（2026-09-02，默认参数随修订 1 一并生效）：大气壳顶取 100 km；低仰角 X 封顶 10（本场景不触及）。来源：§2 建议项 4/5，用户未提异议随裁定 1 采纳。

## §4 运行记录（Stage C 后填写）

1、实现范围：
1、environment 层新增 `ComputeShellAirmassFactor`（穿壳弦长÷垂直壳厚，夹取 [0,10]）与 `ResolveGeometricTransmittance`（τ_geo=τ_eff^X）。
2、管线逐目标 τ_geo 替换每周期共用 τ：SNR/d_max/SNR 门归因与 issue 消息共 4 处用点；删除每周期 `transmittance` 变量。
3、单测：环境层新增 5 例（纯空间 X=0、地面天顶 X=1、壳内目标 X<1、掠壳封顶 10、退化输入）；管线测试 2 例改为新口径拼装（雾天测试目标移入壳内 79 km 高、扫描窗挪至方位 175°）。
4、公共头 include/1q 零变更（契约允许范围核实）。

2、验证命令与结果：
1、`cmake --build ... --target 1q_sbirs_sensor_unit_tests --config Release`：构建通过。
2、`1q_sbirs_sensor_unit_tests.exe`：271/271 全绿（含新增 5 例与改写 2 例）。
3、`1q_sbirs_sensor_contract_tests.exe`：14/14；`1q_sbirs_sensor_integration_tests.exe`：29/29。
4、旗舰场景 `sbirs_triple_sat_fix_messages.exe` 重跑：dual_sat_cycles=79/80 不回归；d_max 320 行全等 128617668.4 m，与闭式预测 114606933.3/√0.794=128617670.8 一致（浮点尾差 2.4 ppm）；四对存活路径 X=0（目标 507~614 km 高于壳顶），被排除两对恰为穿地遮挡对，与矩阵预判吻合。

3、权威回写去向：
1、docs/sbirs_sensor/algorithms.md："Foundation 物理链路"第 7 条 d_max 公式 τ_eff→τ_geo；"气象衰减模型"新增第 6 条壳段气团几何修正全口径。
2、docs/sbirs_sensor/boundaries.md：1b 条 d_max 依赖项改为逐目标 τ_geo（壳段气团）。

4、残留风险：
1、SNR/接收功率绝对数值仍挂在简化噪声链（80 K 热噪声标量 N_eff≈6.6e-11 W，背景亮度/读出默认 0）：对真实背景限制系统低约 4 个数量级；检测门 0.001 与 d_max 均按该链标定，场景判定自洽（用户 2026-09-02 确认该口径并知悉量级估算：真实同型目标 1 秒积分 SNR 约 130/21 dB、10 ms 驻留约 13/11 dB）。
2、验收日志 SNR 表示法此前已由线性改为 dB（用户确认），跨版本日志数值不可直接对比；旧日志 85.301 为旧链路线性值。

5、后续冻结项：
1、噪声口径真实化：背景辐射亮度 + 像元等效视场进入光子噪声（替代 1 sr 标量简化/热噪声兜底），使 SNR 落入真实量级——独立冻结项，未启动。
2、d_max 时间动态：弹道目标穿大气场景与/或动态环境模型（修订 3 裁定本次不做）。
