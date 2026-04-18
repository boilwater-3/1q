/**
 * @file PipelineConfig.h
 * @brief 定义面向外部调用方的信号流水线聚合配置壳。
 */

#ifndef AIRBORNE_RADAR_CONFIG_PIPELINE_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_PIPELINE_CONFIG_H_

#include "1q/airborne_radar/config/expert/ExpertPipelineConfig.h"
#include "1q/airborne_radar/model/RadarOrientationConfig.h"

namespace airborne_radar {
namespace config {

/**
 * @brief 公开信号流水线聚合配置壳。
 *
 * 所有流水线物理参数均通过 `expert` 字段表示。
 * `orientation` 字段承载运行期可热更新的波束指向与扫描状态，
 * 运行时补丁（RadarRuntimeConfigPatch）会在不重建会话的前提下修改此字段。
 */
struct PipelineConfig {
  expert::ExpertPipelineConfig expert{}; /**< 物理参数配置（探测/跟踪/生命周期/波束调度）。 */
  model::RadarOrientationConfig orientation{}; /**< 波束指向与扫描运行态（支持运行时热更新）。 */
};

}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_PIPELINE_CONFIG_H_
