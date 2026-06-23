# SAR 跨阶段现状审计总览(Phase 0–3)

Date: 2026-06-23

## 目的

本文件汇总对 `module_design.md` 各阶段描述与代码实际状态的审计结论。各阶段详细审计
见对应单阶段报告。本审计系列的方法:派子代理并行收集事实(仅定位,不下判断),由主审
对各阶段描述逐条比对并产出判断报告。

## 单阶段报告索引

| 阶段 | 报告 | 核心结论 |
|---|---|---|
| Phase 0 | `phase0_status_audit.md` | 基本一致,4 处轻微(注释/组织层面) |
| Phase 1 | `phase1_status_audit.md` | **2 处显著不一致**(重构承诺未落地)+ 3 处轻微 |
| 补齐(批次1-6) | `backfill_status_audit.md` | 6 模块交付,**输出层 2 处不一致**(magic 字节数 + 布局措辞) |
| Phase 2 | `phase2_status_audit.md` | **完全一致**,❌ 后置项如实标注 |
| Phase 3 | `phase3_status_audit.md` | BP+杂波已闭环,二阶补偿有意冻结 |

## 跨阶段不一致性汇总

### 显著(已据审计修正 `module_design.md` v2.5)

| # | 阶段 | 位置 | 原描述 | 实际 | 修正 |
|---|---|---|---|---|---|
| 1 | Phase 1 | L252 | `sqrt(range²+x²)` 提炼到 **geometry 层** | 实际在 imaging 层 `SarPhaseReference.cpp:71` | 改为"抽到 imaging 层 SarPhaseReference 模块" |
| 2 | Phase 1 | L265, L338 | RDA 内部改调 `ComputeDopplerParams` | RDA 内联 `2v²/(λR0)`(`SarRda.cpp:111-113`),`ComputeDopplerParams` 对 RDA 是死代码 | 标注"未落地",指向 `phase1_status_audit.md` |
| 3 | 补齐 | L403 | Binary magic "8 字节" + "float32 交错实虚" | magic **7 字节** + **planar**(实部全块+虚部全块) | 改为"7 字节"+"planar";交错用于 sidecar .raw |

### 轻微(未改文档,报告记录)

| # | 阶段 | 位置 | 描述 | 实际 |
|---|---|---|---|---|
| 4 | Phase 0 | `SarTraceSession.h:3` / `SarReplaySession.h:4` | doxygen 自称"占位门面" | 实现为完整真实实现 |
| 5 | Phase 1 | L318 | `IsContiguous()` 公共 API | 实际 private(功能仍生效) |
| 6 | Phase 1 | L283 | `AntennaResolution(antenna, synthetic_aperture)` | 实际 4 参数 |
| 7 | Phase 1 | L218 | `Compress2D` 复用 `FftRows`/`FftCols` | 实际仅 `FftCols` |
| 8 | 补齐 | `ImageFormatter.{h:35,cpp:71}` | 代码注释"6 bytes" | 实际 7 字节(代码注释自相矛盾) |
| 9 | 补齐 | L404 | GeoTIFF manifest "投影说明" | 仅有 `format` 字符串,无真实投影字段 |
| 10 | 补齐 | `conanfile.py`/`ProjectOptions.cmake` | Conan `enable_hdf5` vs CMake `ONEQ_ENABLE_HDF5_OUTPUT` | 两者未显式联动 |

## 各阶段完成度判定

| 阶段 | 文档原标记 | 审计判定 | 备注 |
|---|---|---|---|
| Phase 0 | ✅ 完成(公共 API 冻结) | **维持 ✅** | 公共契约确实完整冻结,轻微项不影响 |
| Phase 1 | ✅ 完成(最小可审批闭环) | **维持 ✅** | 功能闭环达成;2 处不一致是"重构承诺未落地",非功能缺失 |
| 补齐 | ✅ 已实现(批次1-6) | **维持 ✅** | 6 模块均有源+测试;输出层措辞修正 |
| Phase 2 | ✅ 完成 | **维持 ✅** | 完全一致 |
| Phase 3 | 🟡 部分 | **v2.4 已改为**:BP+杂波已落地,二阶补偿有意冻结 | 见 `phase3_status_audit.md` |

## 处置边界

- 本次审计**只修改文档**:`module_design.md`(v2.4→v2.5)+ 新增 5 份单阶段审计报告 + 本总览。
- **未修改任何 C++ 源代码逻辑**。
- 代码注释自相矛盾项(`ImageFormatter.{h,cpp}` 的"6 bytes")属代码侧,未在本次触碰——如需修正属独立的注释清理,可后续单独处理。
- 显著不一致 #2(RDA→ComputeDopplerParams 未接线)**不构成接线授权**;补接线需单独审批,且涉及 RDA 内部重构,应先评估是否值得消除重复实现。

## 审计方法与可信度

- 事实收集:4 个 Explore 子代理并行,覆盖 Phase 0/1/补齐/Phase 2;Phase 3 在上一轮已审。
- 关键差异(显著项 #1、#2、#3)均由主审**二次直接读取源码确认**(`SarPhaseReference.cpp:71`、`SarRda.cpp:111-113` + grep `ComputeDopplerParams`、`ImageFormatter.cpp:71-86`),非仅依赖子代理报告。
- 各单阶段报告均含文件路径 + 行号,可独立复核。

## 非目标

- 不重开任何冻结项(Auto/CSA/Omega-K/辐射定标/二阶补偿/PGA 闭环)。
- 不修改代码逻辑或公共 API。
- 不构成阶段升格或新能力授权。
- 不外推到 Phase 4/5。
