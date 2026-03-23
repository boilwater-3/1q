/**
 * @file InterceptComponentFactory.h
 * @brief 定义电子侦察流水线内部组件装配工厂。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_INTERCEPT_COMPONENT_FACTORY_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_INTERCEPT_COMPONENT_FACTORY_H_

#include "1q/electronic_surveillance_radar/pipeline/InterceptPipelineTypes.h"
#include "electronic_surveillance_radar/intercept/AngleErrorModel.h"
#include "electronic_surveillance_radar/intercept/ScanPatternGenerator.h"

namespace electronic_surveillance_radar {
namespace pipeline {
namespace internal {

/**
 * @brief InterceptComponentFactory 负责流水线配置到内部组件配置的映射。
 */
class InterceptComponentFactory final {
 public:
  /**
   * @brief 构造扫描排布配置。
   * @param[in] config 顶层流水线配置。
   * @return 扫描排布配置。
   */
  static intercept::ScanPatternConfig BuildScanPatternConfig(
      const InterceptPipelineConfig& config) {
    intercept::ScanPatternConfig scan_config;
    scan_config.start_az_deg = config.scan.scan_start_az_deg;
    scan_config.end_az_deg = config.scan.scan_end_az_deg;
    scan_config.start_el_deg = config.scan.scan_start_el_deg;
    scan_config.end_el_deg = config.scan.scan_end_el_deg;
    scan_config.az_step_deg = config.scan.az_step_deg;
    scan_config.el_step_deg = config.scan.el_step_deg;
    scan_config.start_pos = config.scan.scan_start_pos;
    scan_config.sequence = config.scan.scan_sequence;
    return scan_config;
  }

  /**
   * @brief 构造测角误差模型配置。
   * @param[in] config 顶层流水线配置。
   * @return 测角误差模型配置。
   */
  static intercept::AngleErrorModelConfig BuildAngleErrorModelConfig(
      const InterceptPipelineConfig& config) {
    intercept::AngleErrorModelConfig error_config;
    error_config.coefficient = config.algorithm.angle_error_coefficient;
    return error_config;
  }
};

}  // namespace internal
}  // namespace pipeline
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_INTERCEPT_COMPONENT_FACTORY_H_
