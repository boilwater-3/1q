/**
 * @file SbirsIssueCodes.h
 * @brief SBIRS issue code 注册表：本模块全部 code 常量的单一事实来源（规则 14c）。
 * @note 仅用于日志映射、不产生为 issue code 的兜底串（如 sbirs.validation_rejected）不在此登记。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_ISSUE_CODES_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_ISSUE_CODES_H_

namespace sbirs_sensor {
namespace session {
namespace codes {

// ===== 输入校验问题（"sbirs.validation.<snake_case>"）=====

/** @brief 周期步长非法（非正或非有限值）。 */
constexpr char kInvalidCycleDeltaTime[] = "sbirs.validation.invalid_cycle_delta_time";

/** @brief 周期步长超过帧周期合理范围（> 10 倍帧周期，即 10 / frame_rate_hz）。 */
constexpr char kCycleDeltaTimeExceedsFramePeriod[] =
    "sbirs.validation.cycle_delta_time_exceeds_frame_period";

/** @brief 卫星位置缺失、非有限或为零向量。 */
constexpr char kInvalidSatellitePosition[] = "sbirs.validation.invalid_satellite_position";

/** @brief 目标物理输入非法（ID/位置/温度/辐射率/投影面积/速度等未满足有限与正数要求）。 */
constexpr char kInvalidTargetPhysical[] = "sbirs.validation.invalid_target_physical";

/** @brief 硬件波长带非法（须为正且有下界小于上界）。 */
constexpr char kWavelengthBandInvalid[] = "sbirs.validation.wavelength_band_invalid";

/** @brief 硬件光学孔径非正。 */
constexpr char kOpticalApertureNotPositive[] = "sbirs.validation.optical_aperture_not_positive";

/** @brief 任务视场角非正。 */
constexpr char kMissionFovNotPositive[] = "sbirs.validation.mission_fov_not_positive";

/** @brief 扫描起始方位角非法（须为有限值且在 [-180, 180)）。 */
constexpr char kInvalidScanStartAzimuth[] = "sbirs.validation.invalid_scan_start_azimuth";

/** @brief 扫描跨度非法（须为有限值且在 (0, 360]）。 */
constexpr char kInvalidScanSpan[] = "sbirs.validation.invalid_scan_span";

/** @brief 扫描方向非法。 */
constexpr char kInvalidScanDirection[] = "sbirs.validation.invalid_scan_direction";

/** @brief 距离门非法（min/max 未有序或为负）。 */
constexpr char kInvalidRangeGate[] = "sbirs.validation.invalid_range_gate";

/** @brief 帧率非正。 */
constexpr char kFrameRateNotPositive[] = "sbirs.validation.frame_rate_not_positive";

/** @brief 扫描速率非法（须为非负有限值）。 */
constexpr char kInvalidScanRate[] = "sbirs.validation.invalid_scan_rate";

/** @brief 窄视场指向最大转动速率非法（须为正且有限）。 */
constexpr char kInvalidNarrowPointingSlewRate[] =
    "sbirs.validation.invalid_narrow_pointing_slew_rate";

/** @brief 窄视场指向沉降容差非法（须为非负有限值）。 */
constexpr char kInvalidNarrowPointingSettleTolerance[] =
    "sbirs.validation.invalid_narrow_pointing_settle_tolerance";

/** @brief 检测门限非法（须非负）。 */
constexpr char kInvalidDetectionThresholds[] = "sbirs.validation.invalid_detection_thresholds";

/** @brief 误差模型 sigma 非法（须为非负有限值）。 */
constexpr char kInvalidErrorModelSigmas[] = "sbirs.validation.invalid_error_model_sigmas";

/** @brief 探测器带宽非法（须为正且有限）。 */
constexpr char kInvalidDetectorBandwidth[] = "sbirs.validation.invalid_detector_bandwidth";

/** @brief 指向扰动幅值与频率非法（须为非负有限值）。 */
constexpr char kInvalidPointingDisturbanceValues[] =
    "sbirs.validation.invalid_pointing_disturbance_values";

/** @brief 指向扰动相关时间非法（须为正且有限）。 */
constexpr char kInvalidPointingDisturbanceCorrelation[] =
    "sbirs.validation.invalid_pointing_disturbance_correlation";

/** @brief 指向扰动振动频率非法（幅值非零时须为正）。 */
constexpr char kInvalidPointingDisturbanceVibrationFrequency[] =
    "sbirs.validation.invalid_pointing_disturbance_vibration_frequency";

/** @brief 调度器最大并发 NFOV 锁定数非法（须 >= 1）。 */
constexpr char kInvalidSchedulerNfovLocks[] = "sbirs.validation.invalid_scheduler_nfov_locks";

/** @brief 跟踪模式非法。 */
constexpr char kInvalidTrackingMode[] = "sbirs.validation.invalid_tracking_mode";

/** @brief 估计跟踪后端非法。 */
constexpr char kInvalidEstimatedTrackingBackend[] =
    "sbirs.validation.invalid_estimated_tracking_backend";

/** @brief 跟踪门丢失周期数非法（须 >= 1）。 */
constexpr char kInvalidTrackingGateLossCycles[] =
    "sbirs.validation.invalid_tracking_gate_loss_cycles";

// ===== 执行诊断（规则 13b/14c）=====

/** @brief 目标被地球遮挡（视线被地球遮蔽）。 */
constexpr char kTargetOcculted[] = "sbirs.target_occulted";

/** @brief 目标超出作用距离（不在距离门 [min, max] 内）。 */
constexpr char kTargetOutOfRange[] = "sbirs.target_out_of_range";

/** @brief 目标宽视场外（不在 WFOV 扫描覆盖内）。 */
constexpr char kTargetOutOfWfov[] = "sbirs.target_out_of_wfov";

/** @brief 目标信噪比低于门限（低于 WFOV 最低 SNR）。 */
constexpr char kTargetSnrBelowThreshold[] = "sbirs.target_snr_below_threshold";

/** @brief 设备关机（非执行周期中止）。 */
constexpr char kSensorPoweredOff[] = "sbirs.sensor_powered_off";

}  // namespace codes
}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_ISSUE_CODES_H_
