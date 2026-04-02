/**
 * @file RadarRuntimeConfigBuilder.h
 * @brief 定义运行期可变配置的补丁模型与链式构造器。
 */

#ifndef AIRBORNE_RADAR_CONFIG_RADAR_RUNTIME_CONFIG_BUILDER_H_
#define AIRBORNE_RADAR_CONFIG_RADAR_RUNTIME_CONFIG_BUILDER_H_

#include "1q/airborne_radar/config/RadarOrientationConfig.h"
#include "1q/airborne_radar/config/SignalPipelineConfig.h"
#include "1q/airborne_radar/environment/EnvironmentTypes.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace common {
namespace config {

/**
 * @brief RadarRuntimeConfigPatch 描述运行期可变参数补丁。
 *
 * @note “可外部调整”定义：调用方可在不重建 `RadarSession` 的前提下，
 * 通过 `RadarSession::ApplyRuntimeConfig(...)` 直接提交修改。
 */
struct RadarRuntimeConfigPatch {
  bool has_signal_pipeline_config{false};        /**< [补丁标志] 是否覆盖整套信号流水线配置 */
  SignalPipelineConfig signal_pipeline_config{}; /**< [可外部调整] 整套信号流水线配置 */

  bool has_signal_detection_config{false};         /**< [补丁标志] 是否覆盖探测域关键配置 */
  SignalDetectionConfig signal_detection_config{}; /**< [可外部调整] 探测域关键配置 */

  bool has_signal_beam_control_config{false}; /**< [补丁标志] 是否覆盖波束控制域关键配置 */
  SignalBeamControlConfig signal_beam_control_config{}; /**< [可外部调整] 波束控制域关键配置 */

  bool has_environment_model_config{false}; /**< [补丁标志] 是否覆盖整套环境模型配置 */
  environment::EnvironmentModelConfig
      environment_model_config{}; /**< [可外部调整] 整套环境模型配置 */

  bool has_jamming_detection_threshold_db{false}; /**< [补丁标志] 是否更新干扰判定阈值 */
  float jamming_detection_threshold_db{6.0f};     /**< [可外部调整] 干扰判定阈值（单位：dB） */

  bool has_platform_attitude_deg{false};       /**< [补丁标志] 是否更新平台姿态 */
  PlatformAttitudeDeg platform_attitude_deg{}; /**< [可外部调整] 平台姿态角（单位：deg） */

  bool has_work_sub_mode{false};                          /**< [补丁标志] 是否更新雷达工作子模式 */
  RadarWorkSubMode work_sub_mode{RadarWorkSubMode::kTws}; /**< [可外部调整] 雷达工作子模式 */

  bool has_scan_center_deg{false};       /**< [补丁标志] 是否更新扫描中心 */
  AzimuthElevationDeg scan_center_deg{}; /**< [可外部调整] 扫描中心（单位：deg） */

  bool has_dwell_center_deg{false};       /**< [补丁标志] 是否更新驻留中心 */
  AzimuthElevationDeg dwell_center_deg{}; /**< [可外部调整] 驻留中心（单位：deg） */

  bool has_commanded_beamwidth_deg{false};         /**< [补丁标志] 是否更新指令态波束宽度 */
  CommandedBeamwidthDeg commanded_beamwidth_deg{}; /**< [可外部调整] 指令态波束宽度（单位：deg） */

  bool has_commanded_beamwidth_enabled{false}; /**< [补丁标志] 是否更新指令态波束使能 */
  bool commanded_beamwidth_enabled{false};     /**< [可外部调整] 指令态波束使能开关 */
};

/**
 * @brief RadarRuntimeConfigBuilder 提供运行期补丁的链式构造。
 */
class ONEQ_API RadarRuntimeConfigBuilder {
 public:
  /** @brief 覆盖整套信号流水线配置。 */
  RadarRuntimeConfigBuilder& WithSignalPipelineConfig(const SignalPipelineConfig& config) {
    patch_.has_signal_pipeline_config = true;
    patch_.signal_pipeline_config = config;
    return *this;
  }

  /** @brief 覆盖探测域关键配置。 */
  RadarRuntimeConfigBuilder& WithSignalDetectionConfig(const SignalDetectionConfig& config) {
    patch_.has_signal_detection_config = true;
    patch_.signal_detection_config = config;
    return *this;
  }

  /** @brief 启用或禁用物理 RCS 覆盖。 */
  RadarRuntimeConfigBuilder& EnablePhysicalRcsOverride(bool enable = true) {
    patch_.has_signal_detection_config = true;
    patch_.signal_detection_config.rcs_physics.enable_physical_rcs = enable;
    return *this;
  }

  /** @brief 设置物理 RCS 模型频率（单位：Hz，<=0 时回退发射机载频）。 */
  RadarRuntimeConfigBuilder& WithRcsPhysicsFrequencyHz(float hz) {
    patch_.has_signal_detection_config = true;
    patch_.signal_detection_config.rcs_physics.frequency_hz = hz;
    return *this;
  }

  /** @brief 设置物理 RCS 混合比例 [0,1]。 */
  RadarRuntimeConfigBuilder& WithRcsPhysicsMixRatio(float ratio) {
    patch_.has_signal_detection_config = true;
    patch_.signal_detection_config.rcs_physics.physics_mix_ratio = ratio;
    return *this;
  }

  /** @brief 设置圆柱散射权重 [0,1]。 */
  RadarRuntimeConfigBuilder& WithRcsPhysicsCylinderWeight(float weight) {
    patch_.has_signal_detection_config = true;
    patch_.signal_detection_config.rcs_physics.cylinder_weight = weight;
    return *this;
  }

  /** @brief 设置目标等效半径范围（单位：m）。 */
  RadarRuntimeConfigBuilder& WithRcsPhysicsEquivalentRadiusRangeM(float min_radius_m,
                                                                  float max_radius_m) {
    patch_.has_signal_detection_config = true;
    auto& rcs_physics = patch_.signal_detection_config.rcs_physics;
    rcs_physics.min_equivalent_radius_m = min_radius_m;
    rcs_physics.max_equivalent_radius_m = max_radius_m;
    return *this;
  }

  /** @brief 设置物理 RCS 输出范围（单位：m^2）。 */
  RadarRuntimeConfigBuilder& WithRcsPhysicsOutputRangeM2(float min_rcs_m2, float max_rcs_m2) {
    patch_.has_signal_detection_config = true;
    auto& rcs_physics = patch_.signal_detection_config.rcs_physics;
    rcs_physics.min_rcs_m2 = min_rcs_m2;
    rcs_physics.max_rcs_m2 = max_rcs_m2;
    return *this;
  }

  /** @brief 设置双基地角偏移（单位：deg）。 */
  RadarRuntimeConfigBuilder& WithRcsPhysicsBistaticPsiOffsetDeg(float psi_offset_deg) {
    patch_.has_signal_detection_config = true;
    patch_.signal_detection_config.rcs_physics.bistatic_psi_offset_deg = psi_offset_deg;
    return *this;
  }

  /** @brief 覆盖波束控制域关键配置。 */
  RadarRuntimeConfigBuilder& WithSignalBeamControlConfig(const SignalBeamControlConfig& config) {
    patch_.has_signal_beam_control_config = true;
    patch_.signal_beam_control_config = config;
    return *this;
  }

  /** @brief 覆盖整套环境模型配置。 */
  RadarRuntimeConfigBuilder& WithEnvironmentModelConfig(
      const environment::EnvironmentModelConfig& config) {
    patch_.has_environment_model_config = true;
    patch_.environment_model_config = config;
    return *this;
  }

  /** @brief 更新干扰判定阈值（单位：dB）。 */
  RadarRuntimeConfigBuilder& WithJammingDetectionThresholdDb(float threshold_db) {
    patch_.has_jamming_detection_threshold_db = true;
    patch_.jamming_detection_threshold_db = threshold_db;
    return *this;
  }

  /** @brief 更新平台姿态。 */
  RadarRuntimeConfigBuilder& WithPlatformAttitudeDeg(
      const PlatformAttitudeDeg& platform_attitude_deg) {
    patch_.has_platform_attitude_deg = true;
    patch_.platform_attitude_deg = platform_attitude_deg;
    return *this;
  }

  /** @brief 更新雷达工作子模式。 */
  RadarRuntimeConfigBuilder& WithRadarWorkSubMode(RadarWorkSubMode work_sub_mode) {
    patch_.has_work_sub_mode = true;
    patch_.work_sub_mode = work_sub_mode;
    return *this;
  }

  /** @brief 更新扫描中心。 */
  RadarRuntimeConfigBuilder& WithScanCenterDeg(const AzimuthElevationDeg& scan_center_deg) {
    patch_.has_scan_center_deg = true;
    patch_.scan_center_deg = scan_center_deg;
    return *this;
  }

  /** @brief 更新驻留中心。 */
  RadarRuntimeConfigBuilder& WithDwellCenterDeg(const AzimuthElevationDeg& dwell_center_deg) {
    patch_.has_dwell_center_deg = true;
    patch_.dwell_center_deg = dwell_center_deg;
    return *this;
  }

  /** @brief 更新指令态波束宽度。 */
  RadarRuntimeConfigBuilder& WithCommandedBeamwidthDeg(
      const CommandedBeamwidthDeg& commanded_beamwidth_deg) {
    patch_.has_commanded_beamwidth_deg = true;
    patch_.commanded_beamwidth_deg = commanded_beamwidth_deg;
    return *this;
  }

  /** @brief 更新指令态波束宽度使能开关。 */
  RadarRuntimeConfigBuilder& EnableCommandedBeamwidth(bool enable = true) {
    patch_.has_commanded_beamwidth_enabled = true;
    patch_.commanded_beamwidth_enabled = enable;
    return *this;
  }

  /** @brief 生成运行期配置补丁。 */
  RadarRuntimeConfigPatch Build() const { return patch_; }

 private:
  RadarRuntimeConfigPatch patch_{};
};

}  // namespace config
}  // namespace common
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_RADAR_RUNTIME_CONFIG_BUILDER_H_
