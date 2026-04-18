/**
 * @file ConfigModel.h
 * @brief 定义公开流水线配置采用的建模模式枚举。
 */

#ifndef AIRBORNE_RADAR_CONFIG_CONFIG_MODEL_H_
#define AIRBORNE_RADAR_CONFIG_CONFIG_MODEL_H_

namespace airborne_radar {
namespace config {

/**
 * @brief 标识公开流水线配置当前采用的输入模型。
 */
enum class PipelineConfigModel {
  kSemantic = 0, /**< 使用 `semantic::*Config` 作为公开输入。 */
  kExpert = 1    /**< 使用 `expert::ExpertPipelineConfig` 作为公开输入。 */
};

}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_CONFIG_MODEL_H_
