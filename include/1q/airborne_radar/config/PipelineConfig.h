/**
 * @file PipelineConfig.h
 * @brief 定义面向外部调用方的信号流水线聚合配置壳。
 */

#ifndef AIRBORNE_RADAR_CONFIG_PIPELINE_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_PIPELINE_CONFIG_H_

#include "1q/airborne_radar/config/ConfigModel.h"
#include "1q/airborne_radar/config/expert/ExpertPipelineConfig.h"
#include "1q/airborne_radar/config/semantic/BeamControlConfig.h"
#include "1q/airborne_radar/config/semantic/DetectionConfig.h"
#include "1q/airborne_radar/config/semantic/LifecycleConfig.h"
#include "1q/airborne_radar/config/semantic/TrackingConfig.h"

namespace airborne_radar {
namespace config {

/**
 * @brief 公开信号流水线聚合配置壳。
 * @note `model` 决定流水线读取 semantic 还是 expert 子配置。
 */
struct PipelineConfig {
  PipelineConfigModel model{PipelineConfigModel::kSemantic}; /**< 当前配置模型。 */
  semantic::DetectionConfig detection{}; /**< semantic 探测配置。 */
  semantic::BeamControlConfig beam_control{}; /**< semantic 波束控制配置。 */
  semantic::TrackingConfig tracking{}; /**< semantic 跟踪配置。 */
  semantic::LifecycleConfig lifecycle{}; /**< semantic 航迹生命周期配置。 */
  expert::ExpertPipelineConfig expert{}; /**< expert 流水线配置。 */
};

}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_PIPELINE_CONFIG_H_
