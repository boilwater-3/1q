# SAR Phase 2 现状审计报告

Date: 2026-06-23

## 1. 审计目的

`module_design.md` 把 Phase 2(相位重参考独立自由函数 + GBP 扩展 + 质量指标闭环)标为
`✅ 完成`。本审计逐条比对该描述与 `src/sar/imaging/{SarPhaseReference,SarImageQuality,SarGbp,SarSlowTimeResampling*}.*`
及对应测试的实际实现。本审计不修改代码。

## 2. 审计结论

Phase 2 **全部一致,无实质契约漂移**。文档 L344-377 与完成度矩阵 L41-46 的描述与实现
完全吻合,包括 ❌ 后置项(产品 QA、辐射精度、空间变相位误差)均如实标注。

| 审计项 | 实际状态 | 结论 |
|---|---|---|
| 相位重参考独立自由函数模块 | `SarPhaseReference.{h,cpp}` 全部符号 | ✅ 一致 |
| RDA 抽模块 + phase_reference_applied/mode 诊断 | RDA 集成完整 | ✅ 一致 |
| 跨算法全局相位对齐 | `EstimateGlobalPhaseOffset`/`CompareImagesWithGlobalPhaseReference` | ✅ 一致 |
| 成像质量评估(峰值/3dB/米制/PSLR/ISLR/熵/对比度/相干比较) | `EvaluateImageQuality` 全部指标 | ✅ 一致 |
| GBP/BP 共享内核 + kMaxApprovedDimension=128 | 共享 `FocusSmallSceneBackprojection` | ✅ 一致 |
| 慢时间重采样(超设计)+ DiagnoseSlowTimeGaps(gap_ratio≥1.5) | 完整 + 结构化拒绝理由 | ✅ 一致 |
| Session 诊断可见性(sar.rda_peak / SarOutputFrame / replay / trace) | 标量摘要全链路 | ✅ 一致 |
| ❌ 产品 QA 后置(`ComputeImageMetrics` 缺) | 确实不存在 | ✅ 如实标注 |
| ❌ 辐射精度冻结缺口(`radiometric_accuracy_db`) | 确实不存在 | ✅ 如实标注 |
| ❌ 空间变相位误差不做 | 比较工具仅消全局常数相位 | ✅ 如实标注 |

## 3. 相位重参考实证(`src/sar/imaging/SarPhaseReference.{h,cpp}`)

- `PhaseReferenceMode{kNative,kCenterBroadside}`(`:14`)、`PhaseReferenceConfig`(默认 kCenterBroadside)、`PhaseReferenceDiagnostics`、`NeedsPhaseReference`、`ApplyBroadsideCenterPhaseReference`、`EstimateGlobalPhaseOffset`、`ApplyGlobalPhaseOffset`(额外 helper)全部存在。
- RDA 集成:`SarRda.cpp:206` 构造 config、`:213` 调用、`:217-218` 记录 `phase_reference_applied`/`mode`。
- 空/无效输入保护:返回 false,不抛异常(`SarPhaseReference.cpp:47-59,91-109`)。
- **公共层仅暴露 `SarPhaseReferenceMode` 摘要枚举**(`SarCycleResult.h:50`),`PhaseReferenceConfig`/`Diagnostics`/`NeedsPhaseReference`/`Apply*`/`Estimate*` 均 internal。✅ 与"独立内部 API;public 仅暴露结果摘要枚举"一致。

## 4. 成像质量实证(`src/sar/imaging/SarImageQuality.{h,cpp}`)

- `MainlobeEstimationMethod{k3dB,k20dB}`、`ImageQualityConfig`(pixel spacings + compute_contrast)、`ImageQualityMetrics`(valid/peak_row/col/magnitude/mainlobe_method/range_width_3db_bins/azimuth_width_3db_bins/resolution_m_valid/range_resolution_3db_m/azimuth_resolution_3db_m/pslr_db/islr_db/entropy_nats/image_contrast)、`ImageComparisonMetrics`(valid/phase_offset_rad/normalized_rms_error/coherent_correlation)全部存在。
- **公共入口函数名是 `EvaluateImageQuality`**(非 `ComputeImageMetrics`)。
- `CompareImagesWithGlobalPhaseReference` 归一化全局幅度 + 去全局常数相位(`.cpp:193-203`)。
- ❌ 项核实:`ComputeImageMetrics` 全仓库零命中(仅文档);`radiometric_accuracy_db` 仅在 docs。✅ 两个 ❌ 标注如实。

## 5. GBP/BP 实证(`src/sar/imaging/SarGbp.{h,cpp}`)

- `FocusSmallSceneGbp`/`FocusSmallSceneBp` 均薄包装内部 `FocusSmallSceneBackprojection`,仅 `BackprojectionTraversal{kPixelMajor,kPulseMajor}` 区分。
- `kMaxApprovedDimension=128U`(`:15`),`IsValid` 门控方位+距离像素数(`:24-25`)。
- `GbpDiagnostics`(evaluated_pixels/accumulated_samples/out_of_bounds_samples/max_approved_dimension/range_interpolation="linear"/traversal_order)一致。
- **无多线程、无 GPU**:全 `src/sar/` grep `setNumThreads`/`setUseGpu`/`num_threads`/`omp`/`OpenMP`/`parallel` 零相关命中。✅ 与"无多线程、无 GPU"一致。

## 6. 慢时间重采样实证(超设计)

- `DiagnoseSlowTimeGaps`(`SarSlowTimeResampling.h:49-51`),拒绝阈值 `gap_ratio >= 1.5`(`.cpp:47`),输出 rejected_gap_count/suspected_missing_pulse_count/first_rejected_gap_index/maximum_gap_ratio/resampling_allowed。
- 结构化拒绝理由(`SarSlowTimeResamplingExecutor.h:22-30`):`SlowTimeResamplingRejectionReason{kNone,kInvalidRequestId,kInvalidExpectedInterval,kInvalidTimeAxis,kInvalidRawHistory,kMissingPulseGap,kResamplingFailure}`(6 个 + kNone)。

## 7. Session 诊断可见性实证

- `sar.rda_peak` 字符串(`SarImagingExecutor.cpp:138-160`)含:peak index、doppler_rate、azimuth_sample_spacing、azimuth_phase_curvature、azimuth_quadratic_phase_span、max_geometric_doppler、doppler_nyquist_margin、**phase_reference_mode/applied**、range/azimuth_width_3db_bins、**range/azimuth_resolution_3db_m**、image_entropy_nats、**image_contrast**。✅ 与"包含 range/azimuth 米制分辨率与 image_contrast;相位参考模式和是否应用"一致。
- `SarOutputFrame`(`SarCycleResult.h:90-115`):`phase_reference_mode`/`image_quality_mainlobe_method`/`range_width_3db_bins`/`azimuth_width_3db_bins`/`range_resolution_3db_m`/`azimuth_resolution_3db_m`/`image_entropy_nats`/`image_contrast`/`has_image_quality_metrics`/`image_resolution_m_valid`/`phase_reference_applied`。
- FlatBuffers replay(`SarReplayFlatbufferCodec.cpp:52-59` 编码,`:72-87` 解码)+ TraceSink JSON(`SarTraceSession.cpp:42-56`)同步输出上述标量。✅ 一致。

## 8. 测试覆盖实证

| 文件 | 存在 | TEST 数 |
|---|---|---|
| `sar_phase_reference_test.cpp` | ✅ | 4 |
| `sar_image_quality_test.cpp` | ✅ | 8 |
| `sar_gbp_test.cpp` | ✅ | 10 |
| `sar_slow_time_resampling_test.cpp` | ✅ | 4 |
| `sar_slow_time_resampling_executor_test.cpp` | ✅ | 4 |
| `sar_raw_history_slow_time_resampling_test.cpp` | ✅ | 3 |

`CompareImagesWithGlobalPhaseReference`/`EstimateGlobalPhaseOffset` 还在 `sar_gbp_test`、`sar_motion_compensation_test`、`sar_reference_scenario_matrix_test`(11 调用)、PRF 质量矩阵测试中被复用。✅ 与"已用于 RDA↔GBP/BP、慢时间重采样、运动补偿、参考场景矩阵测试"一致。

## 9. 仅有的轻微说明(非不一致)

1. **公共入口函数名**:实际是 `EvaluateImageQuality`,文档 L374 在描述 ❌ 缺口时引用了不存在的 `ComputeImageMetrics`(作为"缺失项"的引用,内部自洽)。非事实矛盾,仅提示公共入口名。
2. **`SarRda.h` 额外暴露** `EstimateAzimuthWidth3dbBins`/`EstimateImageEntropyNats`(`:79-80`)作为 `EvaluateImageQuality` 薄包装,文档未列。非不一致,仅额外表面。

## 10. 处置建议

**无需修正**。Phase 2 文档与实现完全一致,所有 ❌ 后置项均如实标注。轻微说明 1、2 非事实错误,可选补充。

## 11. 本审计的非目标

- 不修改任何代码或文档(Phase 2 无不一致)。
- 不构成产品 QA、辐射精度或空间变相位误差修复授权(均明确后置)。
- 不外推到 Phase 3/4/5。
