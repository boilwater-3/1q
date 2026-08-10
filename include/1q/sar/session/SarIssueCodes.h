/**
 * @file SarIssueCodes.h
 * @brief SAR issue code 注册表：本模块全部 code 常量的单一事实来源（规则 14c）。
 * @note 仅用于日志映射、不产生为 issue code 的兜底串（如 sar.validation_rejected）不在此登记。
 */

#ifndef ONEQ_SAR_SESSION_SAR_ISSUE_CODES_H_
#define ONEQ_SAR_SESSION_SAR_ISSUE_CODES_H_

namespace sar {
namespace session {
namespace codes {

// ===== 输入校验问题（"sar.validation.<snake_case>"）=====

/** @brief 周期步长非有限。 */
constexpr char kNonFiniteCycleDeltaTime[] = "sar.validation.non_finite_cycle_delta_time";

/** @brief 周期步长非法（<= 0）。 */
constexpr char kInvalidCycleDeltaTime[] = "sar.validation.invalid_cycle_delta_time";

/** @brief 平台含非有限数值字段。 */
constexpr char kNonFinitePlatformField[] = "sar.validation.non_finite_platform_field";

/** @brief 点目标含非有限数值字段。 */
constexpr char kNonFiniteTargetField[] = "sar.validation.non_finite_target_field";

/** @brief 外部脉冲状态含非有限数值字段。 */
constexpr char kNonFinitePulseField[] = "sar.validation.non_finite_pulse_field";

/** @brief 脉冲序号不连续或时间非单调递增。 */
constexpr char kInvalidPulseSequence[] = "sar.validation.invalid_pulse_sequence";

/** @brief 运行期配置含非法硬件/任务字段。 */
constexpr char kInvalidConfig[] = "sar.validation.invalid_config";

/** @brief 距离采样窗口容不下完整 LFM 脉冲。 */
constexpr char kSampleWindowTooSmallForPulse[] =
    "sar.validation.sample_window_too_small_for_pulse";

/** @brief RDA 尺寸超出批准运行门限（性能批准前限制场景规模）。 */
constexpr char kRdaSizeGate[] = "sar.validation.rda_size_gate";

/** @brief RDA 成像需要启用回波生成。 */
constexpr char kRdaRequiresRawEcho[] = "sar.validation.rda_requires_raw_echo";

/** @brief L2 运动补偿配置非法（需回波/RDA 且速度误差非负）。 */
constexpr char kInvalidL2MotionCompensationConfig[] =
    "sar.validation.invalid_l2_motion_compensation_config";

/** @brief L3 BP 配置非法（需回波与有效航路点且禁用 L1/L2 路径）。 */
constexpr char kInvalidL3BpConfig[] = "sar.validation.invalid_l3_bp_config";

/** @brief L3 BP 尺寸超出 128x128 批准门限。 */
constexpr char kL3BpSizeGate[] = "sar.validation.l3_bp_size_gate";

/** @brief 载频非正。 */
constexpr char kCarrierFrequencyNotPositive[] =
    "sar.validation.carrier_frequency_not_positive";

/** @brief 带宽非正。 */
constexpr char kBandwidthNotPositive[] = "sar.validation.bandwidth_not_positive";

/** @brief 脉冲重复频率（PRF）非正。 */
constexpr char kPulseRepetitionFrequencyNotPositive[] =
    "sar.validation.pulse_repetition_frequency_not_positive";

/** @brief 采样率非正。 */
constexpr char kSampleRateNotPositive[] = "sar.validation.sample_rate_not_positive";

/** @brief 天线长度非正。 */
constexpr char kAntennaLengthNotPositive[] = "sar.validation.antenna_length_not_positive";

/** @brief 硬件链路预算字段非法（功率须为正，噪声系数/损耗须非负且有限）。 */
constexpr char kHardwareLinkBudgetInvalid[] = "sar.validation.hardware_link_budget_invalid";

/** @brief 标称斜距非正。 */
constexpr char kNominalSlantRangeNotPositive[] =
    "sar.validation.nominal_slant_range_not_positive";

/** @brief 平台速度非正。 */
constexpr char kPlatformSpeedNotPositive[] = "sar.validation.platform_speed_not_positive";

/** @brief 方位向脉冲数为零。 */
constexpr char kAzimuthPulseCountZero[] = "sar.validation.azimuth_pulse_count_zero";

/** @brief 距离向采样数为零。 */
constexpr char kRangeSampleCountZero[] = "sar.validation.range_sample_count_zero";

/** @brief 期望地面距离/方位分辨率非正。 */
constexpr char kDesiredResolutionNotPositive[] =
    "sar.validation.desired_resolution_not_positive";

/** @brief 保留原始相位历史需要启用回波生成。 */
constexpr char kRetainRawHistoryRequiresRawEcho[] =
    "sar.validation.retain_raw_history_requires_raw_echo";

/** @brief 最大允许斜视角非法（须有限且在 [0, 90) 度）。 */
constexpr char kSquintAngleInvalid[] = "sar.validation.squint_angle_invalid";

/** @brief 环境标量字段非法（须有限且大气损耗非负）。 */
constexpr char kEnvironmentConfigInvalid[] = "sar.validation.environment_config_invalid";

// ===== 执行/外部输入诊断（非 validation 前缀；含 13b 排除诊断与 abort 失败码）=====

/** @brief 外部 raw IQ 无信噪比元数据，跳过链路预算与最小 SNR 门限（kInfo）。 */
constexpr char kExternalRawIqSnrUnavailable[] = "sar.external_raw_iq_snr_unavailable";

/** @brief 设备关机（非执行周期短路）。 */
constexpr char kSensorPoweredOff[] = "sar.sensor_powered_off";

/** @brief 运动补偿（一阶运动补偿误差诊断，kInfo）。 */
constexpr char kMotionCompensation[] = "sar.motion_compensation";

/** @brief RDA 峰值定位（聚焦图像峰值与多普勒诊断，kInfo）。 */
constexpr char kRdaPeak[] = "sar.rda_peak";

/** @brief BP 峰值定位（kInfo）。 */
constexpr char kBpPeak[] = "sar.bp_peak";

/** @brief BP 遍历顺序（kInfo）。 */
constexpr char kBpTraversal[] = "sar.bp_traversal";

/** @brief L3 航路点轨迹生成（kInfo）。 */
constexpr char kL3Trajectory[] = "sar.l3_trajectory";

/** @brief L2 扰动轨迹生成（kInfo）。 */
constexpr char kL2Trajectory[] = "sar.l2_trajectory";

/** @brief 外部脉冲状态被忽略（L1 RDA 且 L2 关闭时，kInfo）。 */
constexpr char kExternalRawIqTrajectoryIgnored[] =
    "sar.external_raw_iq_trajectory_ignored";

/** @brief 外部完整孔径 raw IQ 消费诊断（kInfo）。 */
constexpr char kExternalRawIq[] = "sar.external_raw_iq";

/** @brief 脉冲环缓冲复用与状态诊断（kInfo）。 */
constexpr char kPulseRingBuffer[] = "sar.pulse_ring_buffer";

/** @brief 目标实际斜距与标称斜距严重错配（kWarning）。 */
constexpr char kSlantRangeMismatch[] = "sar.slant_range_mismatch";

/** @brief 回波裁剪（脉冲采样溢出采样窗口，kWarning）。 */
constexpr char kRawEchoClipping[] = "sar.raw_echo_clipping";

/** @brief LFM 波形生成失败。 */
constexpr char kWaveformGenerationFailed[] = "sar.waveform_generation_failed";

/** @brief 估计 SNR 低于配置最小有效值。 */
constexpr char kSnrBelowMinimum[] = "sar.snr_below_minimum";

/** @brief 孔径斜视角超出配置成像限制。 */
constexpr char kSquintAngleExceedsLimit[] = "sar.squint_angle_exceeds_limit";

/** @brief 退化图像峰值（聚焦图像零峰值功率，管线无信号产出）。 */
constexpr char kDegenerateImagePeak[] = "sar.degenerate_image_peak";

/** @brief 运行期状态恢复被拒绝。 */
constexpr char kRuntimeStateRestoreRejected[] = "sar.runtime_state_restore_rejected";

/** @brief 运动补偿失败。 */
constexpr char kMotionCompensationFailed[] = "sar.motion_compensation_failed";

/** @brief RDA 聚焦失败。 */
constexpr char kRdaFailed[] = "sar.rda_failed";

/** @brief RDA 公共聚焦图像导出失败。 */
constexpr char kRdaPublicImageExportFailed[] = "sar.rda_public_image_export_failed";

/** @brief L2 轨迹历史与最新原始孔径不匹配。 */
constexpr char kL2TrajectoryHistoryMismatch[] = "sar.l2_trajectory_history_mismatch";

/** @brief L3 BP 聚焦失败。 */
constexpr char kL3BpFailed[] = "sar.l3_bp_failed";

/** @brief L3 BP 公共聚焦图像导出失败。 */
constexpr char kL3BpPublicImageExportFailed[] = "sar.l3_bp_public_image_export_failed";

/** @brief L3 轨迹历史与最新原始孔径不匹配。 */
constexpr char kL3TrajectoryHistoryMismatch[] = "sar.l3_trajectory_history_mismatch";

/** @brief 平台几何转换失败（LLA 无法转为配置的本地几何）。 */
constexpr char kPlatformGeometryFailed[] = "sar.platform_geometry_failed";

/** @brief L1 条带航迹生成失败。 */
constexpr char kTrackGenerationFailed[] = "sar.track_generation_failed";

/** @brief L3 航路点几何转换失败。 */
constexpr char kL3WaypointGeometryFailed[] = "sar.l3_waypoint_geometry_failed";

/** @brief L3 航路点不覆盖固定 PRF 脉冲时间范围。 */
constexpr char kL3WaypointCoverage[] = "sar.l3_waypoint_coverage";

/** @brief L2 轨迹生成失败。 */
constexpr char kL2TrackGenerationFailed[] = "sar.l2_track_generation_failed";

/** @brief 外部 raw IQ 需要 L1 RDA 或 L3 BP 成像。 */
constexpr char kExternalRawIqRequiresL1Rda[] = "sar.external_raw_iq_requires_l1_rda";

/** @brief 外部 raw IQ 形状不匹配（须与配置完整孔径精确一致）。 */
constexpr char kExternalRawIqShapeMismatch[] = "sar.external_raw_iq_shape_mismatch";

/** @brief 外部 raw IQ 需逐行实际脉冲状态（BP/L2 路径）。 */
constexpr char kExternalRawIqTrajectoryRequired[] =
    "sar.external_raw_iq_trajectory_required";

/** @brief 外部 raw IQ 需逐行理想脉冲状态（L2 路径）。 */
constexpr char kExternalRawIqIdealTrajectoryRequired[] =
    "sar.external_raw_iq_ideal_trajectory_required";

/** @brief 外部 raw IQ 含非有限采样。 */
constexpr char kExternalRawIqNonFinite[] = "sar.external_raw_iq_non_finite";

/** @brief 外部实际脉冲状态轨迹非法（须有限、连续且时间严格递增）。 */
constexpr char kExternalRawIqInvalidTrajectory[] =
    "sar.external_raw_iq_invalid_trajectory";

/** @brief 外部理想脉冲状态轨迹非法（须有限、连续且时间严格递增）。 */
constexpr char kExternalRawIqInvalidIdealTrajectory[] =
    "sar.external_raw_iq_invalid_ideal_trajectory";

/** @brief 脉冲环缓冲不可用。 */
constexpr char kPulseBufferUnavailable[] = "sar.pulse_buffer_unavailable";

/** @brief 点目标几何转换失败。 */
constexpr char kTargetGeometryFailed[] = "sar.target_geometry_failed";

/** @brief 回波生成失败（点目标与地面背景）。 */
constexpr char kRawEchoFailed[] = "sar.raw_echo_failed";

/** @brief 脉冲写入环缓冲失败。 */
constexpr char kPulseBufferPushFailed[] = "sar.pulse_buffer_push_failed";

/** @brief 脉冲历史不可用（无法提供连续最新孔径）。 */
constexpr char kPulseHistoryUnavailable[] = "sar.pulse_history_unavailable";

/** @brief 脉冲距离采样数与预期不符。 */
constexpr char kPulseSampleCountMismatch[] = "sar.pulse_sample_count_mismatch";

}  // namespace codes
}  // namespace session
}  // namespace sar

#endif  // ONEQ_SAR_SESSION_SAR_ISSUE_CODES_H_
