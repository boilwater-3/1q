---
Status: final
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
7、方案对比（修订 1 探针，已执行；由用户评审项 4 时提出"是否移除植被相关逻辑"触发）：
   1、移除方案改动面：公开头 [evidence: include/1q/remote_identification_radar/config/RirEnvironmentConfig.h]（枚举+字段）；回放编解码 [evidence: src/remote_identification_radar/session/RirReplayFlatbufferCodec.cpp] 环境段序列化植被档位，删字段须按失败闭合标识符纪律 bump 标识符并补拒绝测试；examples 基础配置、加载器与场景 JSON；四份 RIR 单测；
   2、移除的连锁死配置：植被是杂波分项唯一环境源，删除后 [evidence: include/1q/remote_identification_radar/config/RirHardwareConfig.h] ::clutter_suppression_gain_db（MTI 增益，回放/验收记录/配置校验三处消费）失配，按干净边界须一并删除，公开面再扩；
   3、最小物理模型方案改动面：仅 `src/remote_identification_radar/internal/` 新模型、`RirPropagationModel` 内部换源、控制器逐目标注入、四份单测更新；公开 API/回放 schema/示例配置零改动；
   4、推理：零漂移证据对两方案作用相反——它使"建"更便宜（无验收风险），而"删"的成本在公开面与回放纪律，与档位是否被场景使用无关；
   5、保留的领域理由：地基探无人机雷达低仰角观测物理上存在地杂波；检测 SINR 分项账本已设杂波通道与 MTI 增益，删源将使该通道永久为零。

## §3 冻结契约（用户讨论结束后填写）

### Frozen Contract

修订 1 裁定后冻结（2026-08-30）：用户裁定"建最小物理模型"、五项判定与最小边界认可、
σ₀ 简化口径接受。

已证明的需求：
- RIR 植被杂波对档位细分无数值响应（<0.1 dB）且不含频率/距离/擦地角通道；
  新最小物理模型须使杂波按"σ₀×杂波区面积+雷达方程"逐目标求解并响应这三类自变量。

允许范围：
- 模块/目录：仅 `src/remote_identification_radar/`。
- 新增：`internal/RirSurfaceClutterModel.h/.cpp`（σ₀ 档位表+杂波区几何+CNR 求解）。
- 修改：`internal/RirPropagationModel.h/.cpp`（结果收缩为仅传播损耗）；
  `runtime/RirController.h/.cpp`（会话级杂波常数灌入改为逐目标求解）；
  模块 `CMakeLists.txt`（登记新源文件）。
- 测试：重写 `tests/unit/remote_identification_radar/rir_propagation_model_test.cpp`；
  新增 `rir_surface_clutter_model_test.cpp` 特征化测试；
  按数值漂移更新 `rir_atmospheric_physics_test.cpp`、`rir_self_contained_pipeline_test.cpp`；
  不改 `tests/unit/common/`。
- 文档：`docs/remote_identification_radar/algorithms.md`、`boundaries.md`、`data-flow.md`
  杂波口径段落；本矩阵 §4 运行记录。

明确禁止范围：
- 公开头文件 `include/1q/remote_identification_radar/**`（枚举/字段/回放 schema 原样）；
- `src/common/` 与 AR 模块（既有 stub 原样供 AR 使用）；
- 传播损耗链（6.5 dB 恒定构成，defer 项 5）；
- `examples/` 场景与加载器；
- σ₀ 实测标定（按"量级正确、非实测标定"简化口径落地并在注释声明）。

行为边界：
- 输入：植被档位、发射机（有效载频/匹配滤波带宽）、天线（主瓣增益+有效波束宽度，
  经既有 `RirResolveEffectiveBeamwidth` 两级回退解析）、全链路双程传播损耗
  （天气+植被恒定+逐目标大气物理）、目标斜距与俯仰视线角、接收机热噪功率。
- 物理：擦地角 ψ = θel/2 − look_el（主瓣俯仰半波束宽减目标俯仰视线角），下限 1°；
  look_el ≥ θel/2（主瓣完全离地）→ 杂波 0 W；
  杂波区面积 A = min(R·θaz·δR/cosψ, R²·θaz·θel/sinψ)，δR = c/(2B)（脉压后距离单元，
  与测距误差模型 δ_R 口径同源）；
  σ₀(ψ) = σ₀表 + 10·log10(sinψ/sin10°)，σ₀表（dB，S 波段量级声明值）：
  草地 −28 / 稀疏林地 −22 / 落叶林 −18 / 针叶林 −16 / 热带密林 −14；
  杂波回波经 `RirRadarEquations::ComputeEchoPower_dBW`（主瓣峰值增益，主瓣杂波近似）；
  频率响应由雷达方程 λ² 项自然进入；σ₀ 视为与频段无关（声明简化）。
- 输出：逐目标杂波等效噪声瓦（经 `ComputeEquivalentClutterNoiseW` 单源换算，
  保留 ±120 dB 相对钳制口径）。
- 错误/回退：kDisabled → 0 W（行为不变）；斜距 ≤0、热噪非正、载频/带宽非正、
  波束宽度解析非正 → 0 W 免疫退化输入。
- 生命周期/调试：无新增日志；验收记录与回放消费口径不变（配置字段未动）。

验收门：
- 构建：本机 release preset 串行构建成功。
- 聚焦测试：`ctest -R "unit::remote_identification_radar"` 全绿；
  `ctest -R "unit::common"` 全绿（common 未改动的回归确认）。
- 契约测试：contract 层存在 RIR 用例时一并全绿。
- 特征化测试：档位序（σ₀ 表序 → CNR 同序）、几何响应（地平线强/主瓣离地为 0）、
  距离响应（脉冲限制区 CNR 随距离下降）、关断=0、确定性；
  植被关闭场景零漂移（`EnvironmentEffectsKeepTargetDetectable` 等回归语义保持）。

非目标：
- 旁瓣杂波、方向图加权的杂波积分、多普勒谱杂波模型（MTI 增益维持标量配置）；
- AR 迁移新模型（后续冻结项再评估）；
- 植被双程衰减随档位/频率分层（defer 项 5 后续探针）；
- 主瓣离地硬截断的波束边缘渐变（声明简化）。

## 修订记录

<!-- 编号条目（修订 1、修订 2……）：日期、裁定内容、来源（用户指令/新证据）；不静默改写既有条目。 -->

1、修订 1（2026-08-30）：用户评审项 4 时提出"进一步考虑是否移除植被相关逻辑"；补充移除方案与最小物理模型方案的改动面对比（§2 第 7 条），方向裁定待用户拍板。同日用户已裁定：五项判定与最小边界认可（common stub 保留给 AR、新模型落 RIR 内部、损耗链 defer）；σ₀ 简化口径接受。

## §4 运行记录（Stage C 后填写）

1、实现范围：
   1、新增 `src/remote_identification_radar/internal/RirSurfaceClutterModel.h/.cpp`
      （σ₀ 档位表+杂波区几何+雷达方程逐目标主瓣杂波；评审后补擦地角上钳 89°，
      防超宽波束 cosψ 变号）；
   2、`RirPropagationModel` 收缩为传播损耗层（common stub 保留供 AR）；
   3、`RirController` 逐目标接线：`ResolveTargetClutterPowerW` 按目标几何求解，
      检测 cell/回退检测器/排除归因三点消费，kDisabled 恒 0 与旧口径逐位一致；
   4、测试：`rir_propagation_model_test` 重写为损耗专用；新增
      `rir_surface_clutter_model_test` 特征化 10 用例（σ₀ 档位序/擦地角几何/
      距离衰减/频率响应/退化输入免疫含波束宽度非正/确定性/量级带）；
      大气与管线测试由 160° 全向波束改 20°/10° 并显式指定驻留中心（原宽波束
      依赖已随旧口径失效）。
2、验证命令与结果：
   - `cmake --build --preset llvm-ninja-release-local`: pass
   - `ctest -R "unit::remote_identification_radar"`: pass（239 用例，含新增特征化）
   - `ctest -R "unit::common"`: pass（common 未改动回归确认）
   - `ctest -R "integration::remote_identification_radar"`: pass
   - `ctest -R "contract::remote_identification_radar|contract::public_api"`: pass
   - code-review 车道：0 阻断；2 中项（文档未回写→本记录前已完成；擦地角上钳→已修）
     2 低项（波束宽度退化用例→已补；23.6° 注释勘误→已修）
3、权威回写去向：
   - `docs/remote_identification_radar/algorithms.md`：RF 链回退口径改逐目标
     杂波模型描述 + 证据行补特征化测试；
   - `docs/remote_identification_radar/boundaries.md`：阶段 3 清单"植被杂波"改
     "植被传播损耗"；环境事实段补杂波物理模型边界；
   - `docs/remote_identification_radar/data-flow.md`：检测 bullet 补逐目标杂波
     数据流（`ResolveTargetClutterPowerW` → `RirSurfaceClutterModel`）。
4、残留风险：
   - σ₀ 档位表为 S 波段量级声明值，非实测标定（用户裁定接受）；
   - 主瓣离地为硬截断，无波束边缘渐变（声明简化）；
   - 传播损耗链 6.5 dB 恒定（defer 项 5，后续冻结项）；
   - 超宽俯仰波束（如 160° 全向档）杂波开启时物理上巨量主瓣杂波，上钳 89° 兜底，
     场景配置应使用物理自洽的波束宽度。
5、后续冻结项：
   - 植被双程衰减随档位/频率分层（原 defer 项 5 的探针）；
   - AR 侧迁移逐目标杂波模型评估；
   - σ₀ 实测标定数据引入。
