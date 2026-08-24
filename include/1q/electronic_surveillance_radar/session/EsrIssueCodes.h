/**
 * @file EsrIssueCodes.h
 * @brief ESR issue code 注册表：本模块全部 code 常量的单一事实来源（规则 14c）。
 * @note 仅用于日志映射、不产生为 issue code 的兜底串（如 esr.validation_rejected）不在此登记。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_ISSUE_CODES_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_ISSUE_CODES_H_

namespace electronic_surveillance_radar {
namespace session {
namespace codes {

// ===== 输入校验问题（"esr.validation.<snake_case>"）=====

/** @brief 平台姿态角含非有限数值。 */
constexpr char kNonFinitePlatformNumericField[] =
    "esr.validation.non_finite_platform_numeric_field";

/** @brief RF 发射帧非法（不匹配周期窗口）。 */
constexpr char kInvalidRfEmissionFrame[] = "esr.validation.invalid_rf_emission_frame";

/** @brief 平台身份/ECEF 运动学无效（实体标识为零或位置/速度含非有限值）。 */
constexpr char kInvalidPlatformKinematics[] = "esr.validation.invalid_platform_kinematics";

/** @brief 平台 ECEF 不可定位（无法转换为有效 WGS84 LLA）。 */
constexpr char kUnlocatablePlatformEcef[] = "esr.validation.unlocatable_platform_ecef";

/** @brief 周期步长非有限值。 */
constexpr char kNonFiniteCycleDeltaTime[] = "esr.validation.non_finite_cycle_delta_time";

/** @brief 周期步长非法（<= 0）。 */
constexpr char kInvalidCycleDeltaTime[] = "esr.validation.invalid_cycle_delta_time";

/** @brief 周期起始时刻非有限值。 */
constexpr char kInvalidCycleStartTime[] = "esr.validation.invalid_cycle_start_time";

/** @brief 扫描速率非正（须为有限正值）。 */
constexpr char kScanRateNotPositive[] = "esr.validation.scan_rate_not_positive";

/** @brief 任务枚举无效（工作模式/扫描起始位/扫描序列须为已知值）。 */
constexpr char kMissionEnumInvalid[] = "esr.validation.mission_enum_invalid";

/** @brief 接收频段下界高于或等于上界。 */
constexpr char kReceiverBandLowerAboveUpper[] =
    "esr.validation.receiver_band_lower_above_upper";

/** @brief 方位波束宽度非正。 */
constexpr char kBeamAzWidthNotPositive[] = "esr.validation.beam_az_width_not_positive";

/** @brief 俯仰波束宽度非正。 */
constexpr char kBeamElWidthNotPositive[] = "esr.validation.beam_el_width_not_positive";

/** @brief 接收机 RF 硬件参数非法（须有限且物理有效，含同址路径）。 */
constexpr char kReceiverRfHardwareInvalid[] = "esr.validation.receiver_rf_hardware_invalid";

/** @brief 调谐计划无效（调谐窗口须有限、非空且在硬件频段内）。 */
constexpr char kTuningPlanInvalid[] = "esr.validation.tuning_plan_invalid";

/** @brief 显式扫描边界含非有限数值。 */
constexpr char kExplicitScanBoundsNotFinite[] =
    "esr.validation.explicit_scan_bounds_not_finite";

/** @brief 显式扫描方位起止颠倒（起始 >= 结束）。 */
constexpr char kExplicitScanBoundsAzSwapped[] =
    "esr.validation.explicit_scan_bounds_az_swapped";

/** @brief 显式扫描俯仰起止颠倒（起始 >= 结束）。 */
constexpr char kExplicitScanBoundsElSwapped[] =
    "esr.validation.explicit_scan_bounds_el_swapped";

/** @brief 扫描中心角非有限数值。 */
constexpr char kScanCenterNotFinite[] = "esr.validation.scan_center_not_finite";

/** @brief 检测策略无效（SNR 须有限、Pfa 在 (0,1)、脉冲数与门限倍率须为正）。 */
constexpr char kDetectionPolicyInvalid[] = "esr.validation.detection_policy_invalid";

/** @brief 环境预设或启用的大气物理参数无效。 */
constexpr char kEnvironmentInvalid[] = "esr.validation.environment_invalid";

// ===== 执行诊断（规则 13b/14c）=====

/** @brief 发射源同址干扰（正常周期按发射源排除的 kInfo 诊断，规则 13b）。 */
constexpr char kEmissionCoSite[] = "esr.emission_co_site";

/** @brief 发射源接收功率为零（正常周期按发射源排除的 kInfo 诊断）。 */
constexpr char kEmissionZeroPower[] = "esr.emission_zero_power";

/** @brief 发射源低于检测门限（正常周期按发射源排除的 kInfo 诊断）。 */
constexpr char kEmissionBelowThreshold[] = "esr.emission_below_threshold";

/** @brief 设备关机（非执行周期中止）。 */
constexpr char kSensorPoweredOff[] = "esr.sensor_powered_off";

/** @brief 射频接收被拒（RF v2 接收机拒绝本周期）。 */
constexpr char kRfReceiverRejected[] = "esr.rf_receiver_rejected";

}  // namespace codes
}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_ISSUE_CODES_H_
