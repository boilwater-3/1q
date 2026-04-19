/**
 * @file PipelineConfig.h
 * @brief 内部 legacy 装配壳定义，仅用于过渡期内部实现。
 */

#ifndef AIRBORNE_RADAR_SRC_CONFIG_LEGACY_PIPELINE_CONFIG_H_
#define AIRBORNE_RADAR_SRC_CONFIG_LEGACY_PIPELINE_CONFIG_H_

#include "1q/airborne_radar/model/RadarOrientationConfig.h"
#include "airborne_radar/config/legacy/expert/ExpertPipelineConfig.h"

namespace airborne_radar {
namespace config {

/**
 * @brief 内部装配过渡壳，面向扩展管线接口的聚合配置。
 */
struct PipelineConfig {
  ExpertPipelineConfig expert{};               /**< 物理参数配置（探测/跟踪/生命周期/波束调度）。 */
  model::RadarOrientationConfig orientation{}; /**< 波束指向与扫描运行态（支持运行时热更新）。 */
};

}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_CONFIG_LEGACY_PIPELINE_CONFIG_H_
