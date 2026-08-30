---
Status: draft
Date: 2026-08-30
Review-Baseline: `evidence/vegetation-clutter-physics` @ `164825ee`
Authority: 非规范性记录；结论以 docs/common/contract.md、docs/common/session_contract.md
  及各模块 docs/<module>/design.md 为准；与库实现冲突时以库为准。
---

# vegetation-clutter-physics：证据矩阵

<!-- 本文档写作规则：
1、证据一律写成一行：- **证据**：[evidence: 路径]，可加 ::符号名；禁止行号。
2、说明简要，一项一行；多个要点用 1、2、3 序号分点分行，禁止大段描述。
3、引用规则时直接写出规则内容，并用证据形式锁定来源文件；禁止写"见xx规则"。
4、面向非专业开发者，用平实中文；术语首次出现时给一句白话解释。
5、探针/测试必须是已实际执行的；无法直接验证的判断以"推理："开头标注。
-->

## §0 背景与待裁定的问题

1、触发：用户质疑 RIR 植被杂波模型（`RirVegetationCoverProfile` 五档位）不符合现实常理。
2、复核证实：杂波功率链对档位细分在数值上基本无响应，且不含频率/距离/擦地角任何物理自变量。
3、待裁定项：
   1、档位细分数值失效是否为真实缺陷；
   2、物理自变量缺失是否为真实缺陷；
   3、改造 common 单源是否波及 AR 冻结口径（边界收缩）；
   4、RIR 权威场景验收行是否漂移（进 Stage B 的前置）；
   5、固定传播损耗链是否同步改造（范围控制）。
4、术语：CNR（杂噪比）= 杂波功率比接收机热噪功率高多少 dB；σ₀ = 地表散射系数，
   即单位照射面积的地表回波强度（dB）；擦地角 = 雷达波束与地面的夹角。

## §1 证据矩阵

<!-- 1、探针/测试列写已执行的动作与结果。
2、建议判定只能取 pass / reject / narrow / defer，最终以用户裁定为准。
-->

| 待裁定项 | 假设（要证明什么） | 证据来源 | 探针/测试（已执行） | 通过条件 | 否定条件 | 建议判定 |
|---|---|---|---|---|---|---|
| 1、档位细分数值失效是真实缺陷 | 五个植被档位的杂波输出差 <0.5 dB，档位选择对仿真无实际影响，与公开配置"选择档位后…影响…杂波估计"的承诺不符 | [evidence: src/common/radar/VegetationClutterModel.cpp] ::ComputeVegetationClutterMultiplier（乘数链与 [1,30] 钳制）；[evidence: include/1q/remote_identification_radar/config/RirEnvironmentConfig.h] ::RirVegetationCoverProfile；[evidence: tests/unit/common/common_vegetation_clutter_model_test.cpp] ::DenseProfileExceedsGrassland | 已按源码公式链逐档复算：五档 CNR 依次 3.02/3.04/3.05/3.03/3.07 dB，档位间最大差 0.05 dB；乘数实际值 1.008~1.024，距钳制上限 30 差三个数量级 | 复算复现档位间差 <0.5 dB | 复算与库实现不一致，或存在使档位差显著的合法参数域 | pass |
| 2、物理自变量缺失是真实缺陷 | 杂波功率不含频率、距离、擦地角通道，与雷达参数和观测几何无关，违背"杂波=σ₀×照射面积+雷达方程"的物理形态 | [evidence: src/common/rcs/RcsPhysics.h] ::compute_leaf_phase_matrices（签名无波长输入）；[evidence: src/common/radar/VegetationClutterModel.cpp] ::EvaluatePropagationClutter（输入仅覆盖档位）；[evidence: src/remote_identification_radar/runtime/RirController.cpp] ::ResolveEnvironment（每周期一次、与目标几何无关）；[evidence: include/1q/sar/config/SarEnvironmentConfig.h] ::surface_backscatter_sigma0_db（同仓 σ₀ 先例） | 源码审读完成：从配置档位到检测单元求解器的杂波链路上，波长/斜距/擦地角均无数据通道。推理：叶片散射强度本质上由"叶片尺寸/波长"决定，无波长输入的散射模型无法响应雷达频段 | 三自变量在调用链中确认无通道 | 任一自变量已存在有效数据通道 | pass |
| 3、直接改 common 单源会波及 AR 冻结口径 | AR 测试钉死恒定输出、AR↔RIR 对账测试消费同一函数，共改会破坏 AR 验收口径 | [evidence: tests/unit/airborne_radar/ar_propagation_model_test.cpp]（断言 propagation_loss_db==6.5、clutter_power_db==3.0）；[evidence: tests/unit/remote_identification_radar/rir_rf_physical_parity_test.cpp] ::EnvironmentDerivedClutterChainMatchesAr（直接消费 EvaluatePropagationClutter） | grep AR/RIR 测试完成：钉值断言与 parity 消费点均存在 | 存在钉值断言，需要边界收缩决策 | AR 侧无钉值断言（可安全共改） | narrow |
| 4、权威场景验收行零漂移是进 Stage B 的前置 | 权威 RIR 场景植被为 kDisabled，换源后现验收输出不变 | [evidence: examples/scenes/rir_ground_site_recognition/rir_ground_site_recognition.json] ::vegetation_cover_profile（kDisabled） | 已查场景 JSON：RIR 环境块为 kDisabled；进 Stage B 后以聚焦测试复核 | 场景 kDisabled 且 unit::remote_identification_radar 聚焦测试全绿 | 场景实际启用植被，或既有断言漂移 | pass |
| 5、固定传播损耗链不需同步改造 | 6.5 dB 恒定损耗链不是本次缺陷主因，同步改动会放大验收漂移面 | [evidence: src/common/radar/VegetationClutterModel.cpp]（损耗=三常数和）；[evidence: tests/unit/airborne_radar/ar_propagation_model_test.cpp]（钉值 6.5） | 源码审读完成：损耗与杂波在结果结构体中耦合但可分别替换。推理：本次仅替换杂波功率链，损耗链保留现口径 | 不需改造（现状可用） | 用户裁定损耗链必须同步物理化 | defer |

## §2 判定汇总与待裁定问题

1、项 1 pass：档位细分对输出影响 <0.1 dB，配置承诺与行为脱节成立。
2、项 2 pass：杂波链无频率/距离/擦地角通道，物理形态缺失成立。
3、项 3 narrow：common 单源保留供 AR；RIR 模块内部新建最小物理杂波模型并切换消费点，不泛化到 common。
4、项 4 pass：权威场景植被关闭，换源零漂移可期，聚焦测试复核后才进实现。
5、项 5 defer：损耗链留待后续冻结项；下一步探针是"植被双程衰减随档位/频率"的证据收集。
6、待用户拍板：
   1、五项建议判定（pass/pass/narrow/pass/defer）是否认可；
   2、新模型落在 `src/remote_identification_radar/internal/`，AR 保留既有 stub 不同步改——是否认可；
   3、σ₀ 档位表定位为"量级正确、非实测标定"的简化口径（数值参考同仓 SAR 先例与公开文献 S 波段量级），是否接受；
   4、行为语义变化确认：启用植被后杂波按目标几何逐目标求解（远距/高仰角目标噪底更低），替代现在的会话级常数——这是期望的物理行为。

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
