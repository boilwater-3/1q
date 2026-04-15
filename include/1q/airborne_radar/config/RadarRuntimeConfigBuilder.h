/**
 * @file RadarRuntimeConfigBuilder.h
 * @brief 定义运行期可变配置的补丁模型与链式构造器。
 */

#ifndef AIRBORNE_RADAR_CONFIG_RADAR_RUNTIME_CONFIG_BUILDER_H_
#define AIRBORNE_RADAR_CONFIG_RADAR_RUNTIME_CONFIG_BUILDER_H_

#include "1q/airborne_radar/model/RadarOrientationConfig.h"
#include "1q/airborne_radar/environment/EnvironmentConfig.h"
#include "1q/airborne_radar/environment/EnvironmentRuntimeConfigPatch.h"
#include "1q/airborne_radar/config/SignalPipelineConfig.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace config {

using model::AzimuthElevationDeg;
using model::CommandedBeamwidthDeg;
using model::RadarWorkSubMode;

/**
 * @brief RadarRuntimeConfigPatch 描述运行期可变参数补丁。
 *
 * @note "可外部调整"定义：调用方可在不重建 `RadarSession` 的前提下，
 * 通过 `RadarSession::ApplyRuntimeConfig(...)` 暂存修改，并在下一次成功周期
 * 提交前统一生效。
 *
 * 支持两类运行期更新：
 * 1) 整域覆盖：`signal_pipeline_config` 与 `environment_runtime_config`；
 * 2) 叶子覆盖：工作子模式、扫描/驻留指向、指令态波束宽度等。
 * 当整域与叶子同时出现时，先应用整域再应用叶子，叶子具有最终优先级。
 */
struct RadarRuntimeConfigPatch {
  bool has_signal_pipeline_config{false};          /**< [补丁标志] 是否更新整套信号流水线配置 */
  config::SignalPipelineConfig signal_pipeline_config{}; /**< [可外部调整] 整套信号流水线配置 */

  bool has_environment_runtime_config{false};  /**< [补丁标志] 是否更新环境运行期配置 */
  environment::EnvironmentRuntimeConfigPatch
      environment_runtime_config{}; /**< [可外部调整] 环境运行期配置补丁 */

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
  RadarRuntimeConfigBuilder& WithSignalPipelineConfig(
      const config::SignalPipelineConfig& config) {
    patch_.has_signal_pipeline_config = true;
    patch_.signal_pipeline_config = config;
    return *this;
  }

  /** @brief 覆盖整套环境场景输入。 */
  RadarRuntimeConfigBuilder& WithEnvironmentScenarioConfig(
      const environment::EnvironmentScenarioConfig& config) {
    patch_.has_environment_runtime_config = true;
    patch_.environment_runtime_config.has_scenario_config = true;
    patch_.environment_runtime_config.scenario_config = config;
    return *this;
  }

  /** @brief 应用环境运行期补丁。 */
  RadarRuntimeConfigBuilder& WithEnvironmentRuntimeConfig(
      const environment::EnvironmentRuntimeConfigPatch& patch) {
    patch_.has_environment_runtime_config = true;
    patch_.environment_runtime_config = patch;
    return *this;
  }

  /** @brief 更新干扰判定阈值（单位：dB）。 */
  RadarRuntimeConfigBuilder& WithJammingDetectionThresholdDb(float threshold_db) {
    patch_.has_environment_runtime_config = true;
    patch_.environment_runtime_config.has_jamming_detection_threshold_db = true;
    patch_.environment_runtime_config.jamming_detection_threshold_db = threshold_db;
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
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_RADAR_RUNTIME_CONFIG_BUILDER_H_
