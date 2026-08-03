---
Status: active
Last-reviewed: 2026-08-03
Authority: SAR 模块级边界、非目标与设计变更规则
Answers: SAR 有哪些模块级禁令与边界、哪些非目标、配置/环境/校验的特殊语义、文档变更规则
---

# SAR 模块边界

本文承载 SAR 的模块级边界、非目标、反直觉配置语义和设计变更规则。算法级边界（RDA 不 fallback、
MoCo 阈值等）见 [algorithms.md](algorithms.md)。

## 与 common 契约的关系

SAR 遵守 `docs/common/contract.md`：

1. public API 只暴露稳定 session/config/input/output/trace/replay 门面。`SarSession` 是对外门面，
   只委托内部 `SarController`；Controller、ProcessingPipeline、CompositionRoot 不通过 public header 暴露。
2. `SarSessionConfigBuilder` 是薄封装（整域赋值 + `Build()` 返回副本）；语义档位是
   `SarProfileConstants.h` 中的预定义结构体常量（如 `profiles::kHighResolutionImagingMission`、
   `profiles::kL3BackprojectionProcessing`），不承担 leaf setter 或隐式 validation。档位常量是完整子域
   结构体，整域赋值会重置未管理字段（如 `scene_center_*`、`l3_waypoints`），正确用法是"先赋档位、
   再设场景数据"。
3. SAR 输出遵守三层模型：系统输出、结构化结果、调试视图分离。
4. `SarSession::StepWithResult` 在运行期配置和成像链路前调用 `ValidateSarCycleInput`；存在 error 级
   问题时记录 `invalid_cycle_input` abort 并按既有语义复用上一帧（符合 contract.md
   §实现安全与失败语义规则 3）。
5. SAR runtime config 属于立即提交；`SarController` 在每次 pipeline 执行前捕获 raw pulse、trajectory、
   pulse ID 和 PRF 分数余量，执行 abort 时恢复这些跨周期状态并按需复用上一有效输出。配置不随执行
   失败回滚，执行状态也不得被失败周期污染。

## dt_sec 校验边界（反直觉，勿按"四模块一致"补齐）

`ValidateSarCycleInput` 对 `dt_sec` 仅校验有限性 + 正值，**故意不含** EOS/SBIRS 的
`dt_sec ≤ 10/frame_rate_hz` 上界。

1. SAR 配置中无 `frame_rate_hz` 概念——其合成孔径时间由孔径几何（平台速度、方位分辨率、斜距）决定，
   而非成像帧率。
2. dt 的合理性由 PRF 分数余量、孔径拼接和跨周期 raw history 约束（见 data-flow.md）。
3. 该差异已由 `SarInputValidation.cpp` 的实测校验链与 `SarCycleInput` 无 frame_rate 字段共同固化。

不得为"四模块一致"给 SAR 加 frame_rate 上界。

[evidence: tests/unit/sar/sar_input_validation_test]

## Environment 几何、传播与地表背景契约

`SarEnvironmentConfig` 是 public 四域配置之一，当前五个字段均有确定语义。它只作用于 session
内部生成 raw echo；外部完整孔径 raw IQ 已位于外部生成器接收机之后，session 必须逐样本保留，
不得再次施加坐标转换、大气衰减或地表背景。

内部生成路径的几何转换：

1. 场景中心经纬度和 `terrain_reference_altitude_m` 组成局部原点，平台、点目标和 L3 航路点使用同一转换。
2. `use_flat_earth_geometry=true` 时，`x = R cos(lat0) Δlon`、`y = R Δlat`、
   `z = altitude - terrain_reference_altitude`。
3. `use_flat_earth_geometry=false` 时，经 WGS-84 LLA→ECEF→ENU；两条路径的原点和轴语义一致。
4. 非法 LLA 或转换失败必须结构化中止，不能静默回退到另一条几何路径。

内部回波的大气与地表模型：

1. 若 `enable_atmospheric_attenuation=true`，单程比损耗 `gamma_db_per_km` 对斜距 `r_m` 形成双程损耗，
   每个散射体的复振幅相应衰减；关闭时严格退化为 1（无衰减）。
2. 地表单元 RCS 由 `surface_backscatter_sigma0_db` 和期望地距/方位分辨率之积决定；当前低成本背景用
   场景中心周围确定性 3×3 代表性单元相干叠加到 raw IQ，其平均功率进入干扰/噪声账本，
   **不得计为点目标 signal power**。
3. 地形参考高程、大气比损耗和 sigma0 必须有限；大气比损耗还必须非负。replay roundtrip 保真全部
   environment 字段，但不改变上述来源边界。

[evidence: tests/unit/sar/sar_session_pipeline_test]
[evidence: tests/unit/sar/sar_session_config_builder_test]
[evidence: tests/replay/sar/sar_replay_codec_roundtrip_test]

## 专项序列验证边界

`batch_validation::sar` 使用多静态散射体和平台几何验证当前成像能力，不把 SAR 伪装成目标跟踪器。

六类序列覆盖：多散射点分辨、squint 门控恢复、raw/range-compression/L1 阶段切换、非法 runtime 组合
原子拒绝、无效输入恢复、低 SNR 恢复。

影响退出码的硬检查：
1. replay 输出数完整。
2. 预期非执行周期数。
3. failure marker 数。
4. 恢复后重新产图。
5. 特定序列的非法 runtime patch 原子拒绝。
6. range-compression-only 阶段。

属于 warning/error 观测项（不影响退出码）：completed stage 低于 L1、图像质量缺失、SNR 非有限、
熵非正、跨场景趋势。batch 没有直接读取 lifecycle recorder 或断言完整 ring-buffer 状态，因此不得把
场景名扩大为这些内部状态的硬契约。场景 ID 与运行方式由 `examples/batch_validation/README.md` 维护。

## 非目标

1. 不恢复旧会话工厂或旧文档树。
2. 不把 internal algorithm object 变成 public 替代入口。
3. 不把历史 evidence 文档重新常驻 `docs/sar/`。
4. 不为形式对称把 SAR 输入改造成其它模块的 external adapter 模型。
5. 不用测试阈值放宽替代模型、坐标、算法和契约问题的拆分。

上述边界由文档结构守护和 public API 契约测试守护。

[evidence: tests/contract/check_sar_doc_governance]
[evidence: tests/contract/check_public_api_boundary]

## 设计变更规则

1. public header、session/config/input/output 变化必须同步本文档集和 public API contract 测试。
2. pipeline、轨迹、raw history、聚焦路径或算法限制变化必须同步 algorithms.md。
3. 能力启用、否决或替代关系必须在 algorithms.md 的 `[evidence: ...]` 标注中记录依据。
4. 历史原因只保留摘要说明，不恢复被删除的旧审计文档目录。
5. 验证优先使用 `unit::sar`、`contract::sar`、`replay::sar`、`batch_validation::sar`、SAR guards
   以及 CTest `sar_cxx11_compat`（labels `compatibility;sar`）；当前没有独立的 `integration::sar` 分区。
