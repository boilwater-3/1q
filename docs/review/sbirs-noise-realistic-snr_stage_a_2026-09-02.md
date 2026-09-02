---
Status: draft
Date: 2026-09-02
Review-Baseline: `evidence/sbirs-noise-realistic-snr` @ `71655953`
Authority: 非规范性记录；结论以 docs/common/contract.md、docs/common/session_contract.md
  及各模块 docs/<module>/design.md 为准；与库实现冲突时以库为准。
---

# sbirs-noise-realistic-snr：证据矩阵

<!-- 本文档写作规则：
1、证据一律写成一行：- **证据**：[evidence: 路径]，可加 ::符号名；禁止行号。
2、说明简要，一项一行；多个要点用 1、2、3 序号分点分行，禁止大段描述。
3、引用规则时直接写出规则内容，并用证据形式锁定来源文件；禁止写"见xx规则"。
4、面向非专业开发者，用平实中文；术语首次出现时给一句白话定义。
5、探针/测试必须是已实际执行的；无法直接验证的判断以"推理："开头标注。
-->

## §0 背景与待裁定的问题

上一轮几何透过率迭代收口时确认：库 SNR 绝对值挂在简化噪声链上（实测 −19.7 dB），对真实天基红外系统低约 4 个数量级（同型目标真实约 130~380 线性）。用户裁定开启新议题"把 SNR 恢复到正常值"，抓手为噪声模型的"1 球面度等效视场"标量简化。

术语（白话）：
1、NEP（等效噪声功率）：探测器"底噪"折算成功率的大小，越小越灵。
2、像元视场 Ω：一个探测器像素看出去的立体角，越小背景光越少、光子噪声越低。
3、光子噪声：背景光子到达数量本身的随机涨落，真实系统的主要噪声来源。
4、E_ph：单个光子的能量，把"功率"换算成"光子数"用。

待裁定项（冻结什么、什么证据证明、什么证据否定、最小改动范围）：
1、SNR 绝对口径失真是否成立（热噪声标量主导、光子噪声从未生效）。
2、修复所需量是否全部已有公共字段（零新增契约）。
3、热噪声项默认是否应停用（对齐公共头注释的既载明意图）。
4、背景亮度是否默认开启（决定 SNR 默认落不落真实量级）。
5、数值影响与回归面是否可控。

## §1 证据矩阵

| 待裁定项 | 假设（要证明什么） | 证据来源 | 探针/测试（已执行） | 通过条件 | 否定条件 | 建议判定 |
|---|---|---|---|---|---|---|
| 1、SNR 绝对口径失真 | 库噪声底被热噪声标量 √(4k_B·T·Δf)≈6.65e-11 W（80 K、Δf=1/t）独占：背景亮度默认 0 → 光子项从未参与；场景配 NEP=1e-18（想要近无噪）被热项越过；光子项公式 √(P_bg·t) 缺光子能量换算且按"1 sr 等效视场"取背景功率，量纲为简化记账 | [evidence: src/sbirs_sensor/foundation/SbirsNoiseModel.cpp]；[evidence: include/1q/sbirs_sensor/config/SbirsHardwareConfig.h]；[evidence: examples/scenes/sbirs_triple_sat_fix_messages/sbirs_triple_sat_fix_messages.json] | 1、代码核对：光子项=√(P·t) 注释自认"假设 1 sr"；热项 T=80 默认必算；ResolveEffectiveNoiseW 仅在三项全 0 时回退 NEP。2、实测：旗舰场景 SNR=−19.689 dB（0.0107 线性）与热项独占完全一致（P=7.141e-13、N=6.65e-11）。3、场景硬件段只配 NEP=1e-18 与积分 1.0 | 热项主导且比真实噪声底高约 4 个数量级被代码与实测双证实 | 库已有路径能落真实口径 | pass |
| 2、零新增契约可达 | 修复所需量全部可由现有公共字段导出：像元视场 Ω=(pitch/f)²（focal_length_m=2.0、detector_pixel_pitch_m=30e-6 已在公共硬件配置）；光子能量 E_ph=hc/λ（波段上下限 wavelength_lower/upper_um=3/5 μm 已在公共配置，取中心 4 μm） | [evidence: include/1q/sbirs_sensor/config/SbirsHardwareConfig.h] | 读头文件核对字段与默认值（焦距/像元/波段上下限均存在且默认合理） | 四个量全部可导出，include/1q 零变更 | 任一量缺字段需新增公共配置 → 升级契约流程 | pass |
| 3、热项默认停用 | 公共头注释已载明"背景/热项参数（默认 0 表示只使用 NEP 标量）"，但 detector_temperature_k 默认 80 与注释相悖；改默认 0 使注释成立，热项公式保留为显式选配（T>0 才算），不加开关字段 | [evidence: include/1q/sbirs_sensor/config/SbirsHardwareConfig.h] | 头文件注释与默认值对照（注释说默认 0，实现默认 80） | 改默认后公共注释与实现一致，且旧用例显式配 T>0 的行为不变 | 存在依赖默认 80 热项的既定行为 | pass |
| 4、背景亮度默认开启 | L_bg 默认 0→2.0 W/(sr·m²)（地球中波红外典型值）使光子噪声默认生效，SNR 默认落真实量级；场景可覆盖 | [evidence: include/1q/sbirs_sensor/config/SbirsHardwareConfig.h]；[evidence: examples/scenes/sbirs_triple_sat_fix_messages/sbirs_triple_sat_fix_messages.json] | 场景未配背景亮度（硬件段仅 NEP/积分/帧率）→ 现默认 0 即光子项全局关闭 | 默认值变更后仅影响 SNR/探测概率/d_max 数值口径，检测判定不翻转（门限远低） | 默认变更翻转既有检测判定 → 需保守为场景显式配 | pass |
| 5、数值影响可控（数学推算） | 新口径 N_eff=√(NEP_det²+N_photon²)，N_photon=√(P_bg·E_ph·t)，P_bg=L·A·τ_opt·Ω；取默认值 L=2.0、Ω=(30e-6/2)²=2.25e-10 sr、E_ph=hc/4μm=4.97e-20 J、t=1 s：P_bg=7.06e-11 W、N_ph=1.87e-15 W；场景 NEP=1e-18 可忽略 → N_eff≈1.87e-15 W → SNR≈382（25.8 dB）、Pd≈1.0、d_max≈128.6Mm×√(6.65e-11/1.87e-15)≈2.4e10 m | [evidence: examples/log/sbirs_triple_sat_fix_messages/opir_acceptance.log] | 数值复算（本表）；门限 0.001 ≪ 382 无翻转风险；79/80 的缺周期为几何/时序非 SNR（推理：cross-cue 迭代已证） | 重跑后判定不回归、读数按新口径更新 | 检测/定位判定翻转 | narrow（重跑验证收口） |

补充证据（影响面）：
1、噪声单测以相对比较为主（热项 hotter>colder、合成被最大项主导、三项全 0 回退 NEP），绝对值钉得少——更新面小。
- **证据**：[evidence: tests/unit/sbirs_sensor/sbirs_noise_model_test.cpp]
2、精度评估层用 radiant 1e8 构造 SBIRS 场景与 SNR 门——需重跑核对（本场景上次 SNR×1.26 变化时综合分 0.37613 纹丝不动，推理：门远离当前值，风险低）。
- **证据**：[evidence: include/1q/precision_evaluation/PrecisionEvaluationTypes.h]
3、SBIRS foundation 复制自 EOS；本次只改 SBIRS 噪声口径，EOS 保持原口径（跨模块统一另立议题，非本次目标）。
- **证据**：[evidence: docs/sbirs_sensor/data-flow.md]

## §2 判定汇总与待裁定问题

判定汇总：
1、冻结项 1 pass：热噪声标量独占噪声底、光子项从未生效，代码与实测双证实。
2、冻结项 2 pass：像元视场/波段中心/焦距/像元间距全部已有公共字段，include/1q 零变更。
3、冻结项 3 pass：热项默认 80 与公共头注释"默认 0"相悖，改 0 即对齐（公式保留、显式选配）。
4、冻结项 4 pass：背景亮度默认 2.0 使 SNR 默认落真实量级，场景可覆盖。
5、冻结项 5 narrow：预期 SNR −19.7→+25.8 dB、Pd→1.0、d_max→约 2.4e10 m；重跑核对收口。

需要用户拍板的问题：
1、噪声合成口径：N_eff=√(NEP_det²+N_photon²)，N_photon=√(P_bg·E_ph·t_int)，P_bg=L_bg·A·τ_opt·Ω_pixel，Ω_pixel=(pitch/f)²，E_ph=hc/波段中心——是否采纳（热项退役为显式选配）。
2、背景亮度默认 0→2.0（默认开、SNR 开箱落真实量级）还是保持 0（场景显式配才生效）。
3、热项默认 80→0（对齐公共头注释）可否。
4、E_ph 取波段中心 4 μm 单色近似可否（不做波段积分）。
5、数值影响接受：SNR/Pd/d_max/验收读数全面更新，精度评估层重跑核对。

## §3 冻结契约（用户讨论结束后填写）

已证明的需求：
1、噪声底被简化热噪声标量独占、光子噪声从未生效，SNR 绝对值比真实系统低约 4 个数量级。
2、修复零新增公共字段：像元视场/波段中心由既有焦距、像元间距、波段上下限导出。

允许范围：
- 模块/目录：src/sbirs_sensor/foundation（噪声模型）、include/1q/sbirs_sensor/config/SbirsHardwareConfig.h（仅两处默认值：背景亮度 0→2.0、探测器温度 80→0，修订 2/3）、tests/unit/sbirs_sensor、docs/sbirs_sensor、场景验收读数更新。
- 类/函数：ComputeBackgroundNoiseStatistics（光子项 E_ph+Ω_pixel 正确式、NEP 计入 RSS、全部项统一"噪声能量分母"口径）；ResolveEffectiveNoiseW（回退逻辑保留）；相关注释。
- 测试/文档：噪声模型单测重写/新增；管线测试期望自动同口径；模块权威文档回写。

明确禁止范围：
- 公共头结构性变更（新字段/新枚举/schema）；跨模块类型；EOS 模块（保持原口径，跨模块统一另立议题）。
- 测试阈值不放宽；不加兼容开关。

行为边界：
- 输入：SbirsHardwareConfig 既有字段（NEP、背景亮度、温度、读出、焦距、像元间距、波段上下限、积分时间）。
- 合成：N_eff=√((NEP·t)²+N_photon²+(热项)²+(读出·t)²)，N_photon=√(P_bg·t·E_ph)，P_bg=L·A·τ_opt·Ω_pixel，Ω_pixel=(pitch/f)²，E_ph=hc/λ_center（λ_center=(下限+上限)/2，默认 4 μm 单色近似，修订 4）；SNR=P·t/N_eff 与 d_max 闭式形状不变。
- 错误回退：三项+NEP 全部退化（Ω 或 f 非法等）时维持 NEP 回退；热项仅 T>0 计入。
- 数值口径（修订 1/2/3/4）：旗舰场景 N_eff≈1.87e-15 W、SNR≈382（25.8 dB）、Pd≈1.0、d_max≈2.4e10 m；检测门 0.001 无翻转风险。

验收门：
- 构建 Release 全绿；sbirs 单测全绿（噪声测试重写）+ 契约 14 + 集成 29。
- 特征化：场景重跑 79/80 不回归、SNR/Pd/d_max 按新口径核对（382/1.0/≈2.4e10）；精度评估层重跑对比。

非目标：
- EOS 噪声口径；波段积分 E_ph；背景亮度随场景/视轴变化（地球/冷空差异）；d_max 时间动态。

## 修订记录

修订 1（2026-09-02，用户裁定）：采纳 N_eff=√(NEP探测器²+光子噪声²) RSS 合成；热噪声退役为显式选配（仅探测器温度 >0 计入）。来源：用户选项"采纳 NEP+光子噪声合成（推荐）"。
修订 2（2026-09-02，用户裁定）：背景辐射亮度默认 0→2.0 W/(sr·m²)，光子噪声默认生效，场景可覆盖。来源：用户选项"默认 2.0，开箱真实（推荐）"。
修订 3（2026-09-02，用户裁定）：探测器温度默认 80→0，对齐公共头注释"默认 0 只用 NEP 标量"。来源：用户选项"默认改 0，对齐注释（推荐）"。
修订 4（2026-09-02，用户裁定）：E_ph 取波段中心单色近似（默认 (3+5)/2=4 μm）；接受 SNR/Pd/d_max/验收读数全面更新并重跑核对（含精度评估层）。来源：用户选项"接受，重跑核对（推荐）"与 §2 问题 4。

## §4 运行记录（Stage C 后填写）

<!-- 1、实现范围。
2、验证命令与结果。
3、权威回写去向：哪个结论写进了哪个文件。
4、残留风险。
5、后续冻结项。
-->
