# 跨模块开放议题

Status: active
Authority: 非规定性记录

本文登记调查中发现但尚未定论的跨模块架构议题，不构成契约约束。条目推进到有结论时，应回写为契约规则（进 contract.md）或模块设计（进对应 design.md），并从本文移除。

## 当前修复优先级（2026-07-18 实时代码复核）

以下排序按“已经能证明存在运行时语义风险”优先于“需要 public API 迁移决策”，再优先于“纯机械重构”排列。这里的 P0/P1/P2 是修复顺序，不是线上安全等级；在完成对应失败测试和契约冻结前，不直接修改 public struct 或 replay schema。

| 优先级 | 条目 | 当前判断 | 首个交付物 |
|---|---|---|---|
| P2 | OQ-10i | 涉及 public API/ABI 或跨模块迁移，不能作为顺手清理 | Stage A 迁移契约和 consumer 影响清单 |

排序依据是当前 checkout 的代码和测试，不代表这些条目已经获准实施。原 OQ-10h、OQ-3、OQ-10c、
OQ-8、OQ-9、OQ-10a、OQ-10b、OQ-10d、OQ-10e、OQ-10f、OQ-10g、OQ-10j、OQ-10k、OQ-10l、OQ-10m、OQ-1 已完成；对应运行时语义和证据已迁入
SBIRS、Flight Dynamic、AR、ESR、EOS、SAR 的模块设计权威。

---

## OQ-10 四域对外配置结构体成员合理性与反直觉审查

对 5 个传感器模块（AR / ESR / EOS / SAR / SBIRS）的对外公开四域配置（hardware / mission / policy / environment）共 20 个头文件做了逐字段审查与实际消费路径核验，登记以下反用户直觉问题。按严重度分级；每条均附代码证据。结论前缀含 🔴严重 / 🟠中等 / 🟡轻微。

### OQ-10i 🟡 SBIRS `detector_area_m2` vs EOS `detector_area_cm2` 同物理量单位不一致

同是"探测器面积"，SBIRS 用 m²，EOS 用 cm²，跨域复制易错 4 个数量级。

- SBIRS：`SbirsHardwareConfig.detector_area_m2{1.0e-4f}`，`include/1q/sbirs_sensor/config/SbirsHardwareConfig.h`。
- EOS：`EosHardwareConfig.detector_area_cm2{0.25f}`，`include/1q/electro_optical_sensor/config/EosHardwareConfig.h:23`。

为何未决：两个传感器物理量纲习惯不同（红外探测器传统用 cm²），但对外公开 API 单位不一致增加跨域误用风险。统一单位是源码兼容性变更。

推进需要：
- 决定是否统一到 SI（m²），评估对 EOS 现有消费方与文档的影响；
- 若不统一，在 `docs/common/contract.md` 补一条"跨域同物理量单位须在字段名后缀标明"的规则并加 lint 守护；
- 字段名后缀已带单位（`_m2` / `_cm2`），最低限度应确保文档显著标注差异。
