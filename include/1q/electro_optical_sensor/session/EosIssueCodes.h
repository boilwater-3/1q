/**
 * @file EosIssueCodes.h
 * @brief EOS issue code 注册表：本模块全部 code 常量的单一事实来源（规则 14c）。
 * @note 仅用于日志映射、不产生为 issue code 的兜底串（如 eos.validation_rejected）不在此登记。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_ISSUE_CODES_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_ISSUE_CODES_H_

namespace electro_optical_sensor {
namespace session {
namespace codes {

// ===== 输入校验问题（"eos.validation.<snake_case>"）=====

/** @brief 平台位姿含非有限数值（位置/速度/姿态角）。 */
constexpr char kNonFinitePlatformNumericField[] =
    "eos.validation.non_finite_platform_numeric_field";

/** @brief 目标含非有限数值字段。 */
constexpr char kNonFiniteTargetNumericField[] = "eos.validation.non_finite_target_numeric_field";

/** @brief 目标斜距非法（<= 0）。 */
constexpr char kInvalidTargetRange[] = "eos.validation.invalid_target_range";

/** @brief 目标温度非法（<= 0）。 */
constexpr char kInvalidTargetTemperature[] = "eos.validation.invalid_target_temperature";

/** @brief 目标辐射效率非法（不在 [0, 1]）。 */
constexpr char kInvalidTargetEmissivity[] = "eos.validation.invalid_target_emissivity";

/** @brief 目标反射率非法（不在 [0, 1]）。 */
constexpr char kInvalidTargetReflectance[] = "eos.validation.invalid_target_reflectance";

/** @brief 目标能量平衡校验失败（辐射效率 + 反射率 > 1，kInfo 级）。 */
constexpr char kInconsistentTargetEnergyBalance[] =
    "eos.validation.inconsistent_target_energy_balance";

/** @brief 目标投影面积非法（<= 0）。 */
constexpr char kInvalidTargetProjectedArea[] = "eos.validation.invalid_target_projected_area";

/** @brief 周期步长非有限值。 */
constexpr char kNonFiniteCycleDeltaTime[] = "eos.validation.non_finite_cycle_delta_time";

/** @brief 周期步长非法（<= 0）。 */
constexpr char kInvalidCycleDeltaTime[] = "eos.validation.invalid_cycle_delta_time";

/** @brief 周期步长超过帧周期合理范围（> 10 倍帧周期，即 10 / frame_rate_hz）。 */
constexpr char kCycleDeltaTimeExceedsFramePeriod[] =
    "eos.validation.cycle_delta_time_exceeds_frame_period";

/** @brief 水平视场角非正。 */
constexpr char kHorizontalFovNotPositive[] = "eos.validation.horizontal_fov_not_positive";

/** @brief 垂直视场角非正。 */
constexpr char kVerticalFovNotPositive[] = "eos.validation.vertical_fov_not_positive";

/** @brief 扫描速率非正。 */
constexpr char kScanRateNotPositive[] = "eos.validation.scan_rate_not_positive";

/** @brief 帧率非正。 */
constexpr char kFrameRateNotPositive[] = "eos.validation.frame_rate_not_positive";

/** @brief 扫描方位起止颠倒（起始 >= 结束）。 */
constexpr char kScanRangeAzSwapped[] = "eos.validation.scan_range_az_swapped";

/** @brief 环境预设无效。 */
constexpr char kEnvironmentPresetInvalid[] = "eos.validation.environment_preset_invalid";

/** @brief 启用物理模型时大气物理参数无效（压力/温度/湿度越界）。 */
constexpr char kAtmosphericPhysicsInvalid[] = "eos.validation.atmospheric_physics_invalid";

// ===== 执行诊断（规则 13b/14c）=====

/** @brief 目标视场外（正常周期按目标排除的 kInfo 诊断，规则 13b）。 */
constexpr char kTargetOutOfFov[] = "eos.target_out_of_fov";

/** @brief 设备关机（非执行周期中止）。 */
constexpr char kSensorPoweredOff[] = "eos.sensor_powered_off";

/** @brief 管线输出违反内部契约（kOutputContract 相位）。 */
constexpr char kPipelineContractViolation[] = "eos.pipeline_contract_violation";

/** @brief 运行期状态恢复被拒绝。 */
constexpr char kRuntimeStateRestoreRejected[] = "eos.runtime_state_restore_rejected";

}  // namespace codes
}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_ISSUE_CODES_H_
