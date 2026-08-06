---
Status: active
Last-reviewed: 2026-08-03
Authority: SAR 算法登记与实现边界
Answers: SAR 用了哪些算法、各自实现到什么地步、边界在哪、哪些刻意不实现
---

# SAR 算法登记

本文是 SAR 算法清单与边界的权威。算法本身的逐步逻辑读代码（`src/sar/`）；本文只回答"用没用/到哪步/
为什么不做"。模块级边界（dt_sec、环境几何、public API 边界）见 [boundaries.md](boundaries.md)。

## 算法登记表

| 算法 | 意图（一句话） | 实现状态 | 证据 |
|---|---|---|---|
| LFM 波形 / 匹配滤波 | 基础发射波形和距离压缩匹配滤波器 | session-wired | [evidence: tests/unit/sar/sar_signal_chain_test] |
| 点目标 raw echo 生成 | 从平台轨迹和点目标生成 raw history | session-wired | [evidence: tests/unit/sar/sar_session_pipeline_test] |
| Pulse ring buffer | 跨周期累计 aperture | session-wired | [evidence: tests/unit/sar/sar_controller_runtime_state_test] |
| RDA 聚焦 | L1 broadside stripmap 基础聚焦 | session-wired | [evidence: tests/unit/sar/sar_rda_test] |
| 一阶运动补偿 (L2 MoCo) | 直线轨迹扰动相位补偿 | session-wired | [evidence: tests/unit/sar/sar_motion_compensation_test] |
| BP/GBP 小场景聚焦 | L3 转弯/小场景参考成像 | session-wired | [evidence: tests/unit/sar/sar_gbp_test] |
| 图像质量评估 | 峰值、分辨率、熵、对比度等摘要 | session-wired | [evidence: tests/unit/sar/sar_image_quality_test] |
| Stripmap Omega-K | 大斜视/宽波束友好聚焦 | characterized | [evidence: tests/unit/sar/sar_omega_k_focusing_test] |
| Spotlight Omega-K | 聚束模式聚焦 | experimental | [evidence: tests/unit/sar/sar_omega_k_spotlight_test] |
| ScanSAR Omega-K | 扫描模式聚焦 | experimental | [evidence: tests/unit/sar/sar_omega_k_scansar_test] |
| Multilook | 聚焦后图像域非相干多视 | internal/受控 | [evidence: tests/unit/sar/sar_multilook_test] |
| 辐射定标 | 从已知 RCS 观测求解定标因子 | internal/受控 | [evidence: tests/unit/sar/sar_radiometric_calibration_test] |

实现状态取值：
- **session-wired**：已接入 `SarProcessingPipeline`，覆盖 config、输出/abort、replay 与 session 集成。
- **characterized**：有局部测试 + 确定性质量/失效矩阵，但未接入 session 主链。
- **experimental**：可编译，局部单元测试覆盖，但无质量矩阵。
- **internal/受控**：内部能力，不扩大为 public contract。

## LFM、匹配滤波与距离压缩

- **意图**：每周期由 hardware config 生成 LFM waveform 与匹配滤波器，作为 RDA/BP 的基础处理。
- **实现边界**：
  1. waveform 生成失败会中止周期。
  2. L1 RDA 与 L3 BP 的显式前置条件是 `enable_raw_echo_generation`；关闭时成像配置在执行前被拒绝。
  3. 只有 RDA/BP 实际成功执行内部距离压缩后才置位 `has_range_compressed_echo`；当前没有独立距离压缩
     载荷或独立完成阶段。
- **证据**：[evidence: tests/unit/sar/sar_session_pipeline_test]

## 点目标 raw echo 生成

- **意图**：从平台轨迹、点目标和 LFM 波形生成 raw history，经 `PulseRingBuffer` 组成 aperture。
- **实现边界**：
  1. 接收链按单站雷达方程处理回波幅度，叠加确定性复高斯热噪声；天线二维方向图参与幅度调制。
  2. `estimated_snr_db` 是完整孔径内加噪前点目标平均接收功率与"接收机热噪声 + 分布式地表背景功率"
     之比；地表背景不得伪装成目标信号。
  3. 功率、增益、损耗、噪声系数的单变量变化必须分别满足正、正、负、负的方向性。
- **反直觉点**：环境开启时点目标按真实斜距承受双程大气衰减，地表 sigma0 背景相干叠加到 IQ——但
  这只作用于内部生成路径，外部 raw IQ 路径完全不施加上述链路预算或噪声。
- **证据**：[evidence: tests/unit/sar/sar_session_pipeline_test]

## 外部 raw IQ 消费

- **意图**：调用方提供完整孔径 IQ 样本，session 只校验输入并消费（不重新生成 echo）。
- **实现边界**：
  1. external raw IQ 已位于接收机之后，session 不得再次施加链路预算或噪声。
  2. 该路径将 `estimated_snr_db` 标为不可估计（`-inf`），记录
     `sar.external_raw_iq_snr_unavailable`，并跳过 `minimum_snr_db` 门控。
  3. L1 可不带轨迹，L2 需要 actual/ideal 双轨迹，L3 需要 actual 轨迹；session 只消费已是
     scene-center-relative ENU 的轨迹。
- **反直觉点**：不得以峰均功率比冒充 SNR；现有 public 输入未携带信号/噪声分量元数据。
- **证据**：[evidence: tests/unit/sar/sar_session_pipeline_test]
- **证据**：[evidence: tests/replay/sar/sar_replay_codec_roundtrip_test]

## retain_raw_phase_history 与 focused_image 的 replay 契约

- **意图**：`retain_raw_phase_history` 控制结构化 `SarCycleResult` 是否返回本次实际用于成像的完整孔径；
  `focused_image` 是 replay 权威输出。
- **实现边界**：
  1. 关闭 `retain_raw_phase_history` 时不复制矩阵，`raw_phase_history` 为空且 source 为 `kNone`。
  2. 开启时必须同时启用 raw echo generation，否则 session 初始化和 runtime patch 均拒绝。
  3. 内部生成标记 `kInternallyGenerated`；external raw IQ 标记 `kExternalRawIq`，I/Q 顺序和值保持输入。
  4. 这两个产品属于结构化执行结果，不进入 `SarOutputFrame`；失败周期不发布未完成孔径。
- **反直觉点**：cycle-result replay 以精确值比较全部字段，任一像素变化必须报告 divergence；Replay 的
  `cycle_output` 唯一 payload 是 `SarCycleResult`，不接受缺少 focused image 的 standalone
  `SarOutputFrame` 兼容路径。codec 先构造并验证完整局部结果，成功后才一次提交，失败不得部分覆盖
  调用方输出。
- **证据**：[evidence: tests/replay/sar/sar_replay_codec_roundtrip_test]
- **证据**：[evidence: tests/replay/sar/sar_replay_session_test]

## RDA 聚焦

- **意图**：L1 broadside stripmap 的基础聚焦路径。
- **实现边界**：
  1. 当前 RDA 是 broadside 基础路径；单脉冲孔径明确拒绝，不存在 fallback。
  2. `max_allowed_squint_angle_deg` 是启用成像路径的执行门，必须有限且位于 `[0°, 90°)`。超限以
     `squint_angle_exceeds_limit` 中止，不自动切换到其它算法。
  3. raw-echo-only 模式不执行该成像门。
  4. RDA focused image 是否完整保留由 `retain_focused_image` policy 控制；关闭时只输出占位元数据。
  5. RDA 误差用相位曲率、Doppler margin、3dB 宽度、entropy、contrast 等诊断解释，不通过放宽阈值掩盖。
- **squint 几何定义**（`ComputeSquintAngleDeg`，`src/sar/pipeline/SarProcessingPipeline.cpp`）：
  单脉冲平台状态下，`squint = asin(|v·LOS| / (|v|·|LOS|))`，其中 `v` 为平台速度、`LOS` 为平台指向
  场景中心的视线（ENU 局部坐标，平台相对 `scene_center_*`）。即 **squint = 视线偏离正侧视的角度**：
  正侧视（LOS ⊥ 航迹）为 `0°`，前视/后视（LOS ∥ 航迹）趋近 `90°`——**不是**"视线与航迹夹角"
  本身（两者互补为 `90°`）。内部生成路径以当前周期平台状态计算；外部脉冲/轨迹路径取积累窗内
  所有脉冲的最大 squint。**反直觉点**：多数文献把 squint 定义为视线与航迹夹角（前视 0°），
  本库采用补角定义；场景编排（如平台相对目标区的飞行方向）须按"正侧视 = 0°"设计。
- **反直觉点**：生产相位参考是空间变化的（随慢时间行与距离单元变化），不是全图常数旋转。只有
  `CompareImagesWithGlobalPhaseReference` 在测试/质量比较中估计并消除单个全局常相位，用于比较归一化
  后的图像形状；它不会回写生产图像，也不能掩盖空间变化相位误差。
- **证据**：[evidence: tests/unit/sar/sar_rda_test]
- **证据**：[evidence: tests/unit/sar/sar_image_quality_test]

## 一阶运动补偿 (L2 MoCo)

- **意图**：在 RDA 前对 raw history 施加一阶运动补偿，解决直线轨迹扰动下的一部分相位误差。
- **实现边界**：
  1. 一阶 MoCo **没有运行时 NRMS/coherence 执行门限**——只要 `enable_l2_motion_compensation` 开启就
     无条件执行；返回值只反映输入合法性（轨迹/history 形状、配置有限性），不反映补偿质量。
  2. 不授权二阶补偿或自动算法选择。
  3. 补偿质量须按"横向偏移 × 目标距离单元"矩阵解释，不能只用参考目标概括整个偏移档位。
- **反直觉点（NRMS/coherence 数字的来源）**：证据矩阵测试中出现的 `NRMS < 0.25`、`coherence > 0.97` 是
  **用来判定一阶 MoCo 在哪里失效的特征化阈值**，不是 MoCo 自身的通过判据。MoCo 自己的特征化测试
  使用 `NRMS < 0.3`、`coherence > 0.95`。不得把证据矩阵的失效判定门限误认为运行时执行门限。
- **反直觉点（根因诊断）**：对强转弯场景，失效根因通常是 RDA 轨迹假设，**不应简单归因为二阶残余
  相位**。证据：参考点（二阶残余恒为零）从 9m 起已失效，排除了"残余相位是主因"假设。具体数值：
  6m 时参考目标 `delay=20` 的 NRMS 为 0.176959（通过），但同一 6m 档位的 `delay=12` 已为 0.329653
  （失败）；9m 参考目标为 0.272569（失败）。
- **证据**：[evidence: tests/unit/sar/sar_motion_compensation_test]
- **证据**：[evidence: tests/unit/sar/sar_second_order_motion_compensation_evidence_test]
- **证据**：[evidence: tests/unit/sar/sar_l2_l3_fidelity_matrix_test]

## BP/GBP 小场景聚焦

- **意图**：L3 路径用实际逐脉冲几何承接航路点/转弯小场景，用 backprojection 内核聚焦。
- **实现边界**：
  1. 受尺寸门约束（`kMaxApprovedDimension=128`，azimuth+range pixel count 超 128 被拒绝），不作为
     无限规模生产聚焦器。对比：RDA size gate 为 1024×1024。
  2. 不默认引入并行、GPU 或快速 BP。
  3. BP quality summary 只输出当前可稳定承诺的摘要；米制分辨率有效性与 RDA 不完全相同。
- **证据**：[evidence: tests/unit/sar/sar_gbp_test]

## 能力晋级门

适用于内部候选算法（Omega-K、Spotlight、ScanSAR 等），不创建新的 public 类型。候选算法必须逐级
提供证据；不得因已有 internal 实现而跳级或新增未接线算法族。

| 级别 | 进入条件 | 当前算法 |
|---|---|---|
| `experimental` | 可编译，且局部单元测试明确输入、输出与拒绝边界 | Spotlight Omega-K、ScanSAR Omega-K |
| `characterized` | 除局部测试外，已有确定性、质量/失效矩阵或受控证据 | stripmap Omega-K |
| `production-eligible` | 已冻结场景范围、门限、失败语义与集成证据；仍未改变 session 装配 | 当前无候选 |
| `session-wired` | 已接入 `SarProcessingPipeline`，覆盖 config、输出/abort、replay 与 session 集成 | L1 RDA、L3 BP |

### Stripmap Omega-K 的 Stage A 冻结边界

Stripmap Omega-K 的 Stage A 矩阵冻结为 L1、匀速直线、broadside stripmap，且只允许以
`BuildRawPulseHistory` 生成的孔径调用 `FocusStripmapOmegaK`。Spotlight、ScanSAR、squint、L2/L3 轨迹
与 session 接线均不在本候选内。

- **反直觉点（缩小配置的假阴性）**：复用 RDA 单元测试的缩小配置（1 GHz、20 Hz PRF、2 m/s、100 MHz、
  9×64）时，最大 Stolt 频移超过距离频率支持区，公共有效列为零并稳定拒绝；保持其余参数不变、仅把
  平台速度提高到 5 m/s 后，同一路径恢复公共支持并确定性聚焦成功。**直接原因是缩小配置破坏了
  Omega-K 的几何比例，而不是内部编排器普遍不可用**。
- **当前支持判定**：按全部 PRF FFT 行取交集，不考虑实际能量占用的多普勒带宽；这是保守模型，但尚无
  证据证明应放宽。参考距离不参与公共支持区计算，只参与 front-end 参考相位。
- **为何停在 characterized**：仓库仍没有可追溯、独立的物理真值数据，synthetic fixture 会被 truth
  eligibility 拒绝为非物理证据。下一 probe 是提供带 manifest、digest、provenance、
  `physical_evidence=true` 和 `independently_generated=true` 的独立 L1 真值。

[evidence: tests/unit/sar/sar_omega_k_l1_raw_history_stage_a_test]
[evidence: tests/unit/sar/sar_omega_k_truth_eligibility_test]

## Multilook 与辐射定标

- **Multilook 意图**：聚焦后图像域非相干多视，消费任意 focused complex image，不侵入聚焦器主链路。
- **辐射定标意图**：后处理标量定标能力，从已知 RCS 观测求解定标因子；入口为 `ExecuteCalibrationRequests`
  （批量请求执行）。
- **实现边界**：
  1. 辐射定标当前不扩大为完整生产级雷达方程 public contract。
  2. 实现保持 internal（`src/sar/calibration/`），按当前契约不扩展为 public API。
- **证据**：[evidence: tests/unit/sar/sar_multilook_test]
- **证据**：[evidence: tests/unit/sar/sar_radiometric_calibration_test]

## 非目标（刻意不实现的算法）

1. **完整 CSA（Chirp Scaling Algorithm）**：当前无独立增量，Omega-K 覆盖主要需求。证据：所有孔径
   |alpha| max<0.018，RDA worst NRMS=1.38 均显著超阈值；CSA 仅 geometry+oracle，main flow 0% implemented。
   [evidence: tests/unit/sar/sar_csa_complete_focusing_evidence_test]
2. **PGA 闭环（Phase Gradient Autofocus）**：当前 MoCo 已覆盖直线扰动场景（补偿后 NRMS<0.17，
   Coherence>0.985），PGA 闭环不进入默认生产路径。
   [evidence: tests/unit/sar/sar_pga_autofocus_closure_evidence_test]
3. **二阶 MoCo**：强转弯失败主因是 RDA 轨迹假设，不是简单二阶相位补偿（见上方 MoCo 反直觉点）。
   `grep -r 'SecondOrder\|second_order' src/sar/` 返回零命中。
4. **缺失脉冲自动修复 / NUFFT / 生产 RDA 自动接入**：保留诊断和拒绝矩阵，不默认启用自动修复
   （gap_ratio≥1.5 硬拒绝）。
   [evidence: tests/unit/sar/sar_missing_pulse_rejection_matrix_test]
