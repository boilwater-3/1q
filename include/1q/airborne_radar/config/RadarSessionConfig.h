/**
 * @file RadarSessionConfig.h
 * @brief 定义 RadarSession 的初始化配置结构。
 */

#ifndef AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_H_

#include "1q/airborne_radar/config/ConfigModel.h"
#include "1q/airborne_radar/config/expert/ExpertPipelineConfig.h"
#include "1q/airborne_radar/config/semantic/BeamControlConfig.h"
#include "1q/airborne_radar/config/semantic/DetectionConfig.h"
#include "1q/airborne_radar/config/semantic/LifecycleConfig.h"
#include "1q/airborne_radar/config/semantic/TrackingConfig.h"
#include "1q/airborne_radar/environment/EnvironmentConfig.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace session {

/**
 * @brief RadarSession 初始化配置。
 * @note `pipeline_config_model` 决定会话启动时采用 semantic 还是 expert 流水线输入。
 */
struct ONEQ_API RadarSessionConfig {
  config::PipelineConfigModel pipeline_config_model{
      config::PipelineConfigModel::kSemantic}; /**< 流水线公开配置模型。 */
  config::semantic::DetectionConfig detection{}; /**< semantic 探测配置。 */
  config::semantic::BeamControlConfig beam_control{}; /**< semantic 波束控制配置。 */
  config::semantic::TrackingConfig tracking{}; /**< semantic 跟踪配置。 */
  config::semantic::LifecycleConfig lifecycle{}; /**< semantic 生命周期配置。 */
  config::expert::ExpertPipelineConfig expert_pipeline_config{}; /**< expert 流水线配置。 */
  environment::EnvironmentDefaultConfig environment_default_config{}; /**< 环境默认配置。 */
};

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_H_
