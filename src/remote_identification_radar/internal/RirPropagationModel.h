/**
 * @file RirPropagationModel.h
 * @brief 定义 RIR 环境层最小传播与杂波组合模型（私有实现头）。
 *
 * 数值内核为 common 单源（`oneq::common::radar::VegetationClutterModel`），
 * 场景事实类型换 `Rir*`，零 AR 依赖；传播/杂波基线系数与 AR 一致保留在
 * common 实现内部（环境域"不含内部调参项"合约）。
 * @note 本文件仅供 RIR 模块内部使用，不作为公开 API；周期输入面接线随阶段 2-S。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_INTERNAL_RIR_PROPAGATION_MODEL_H_
#define REMOTE_IDENTIFICATION_RADAR_INTERNAL_RIR_PROPAGATION_MODEL_H_

#include "1q/remote_identification_radar/config/RirEnvironmentConfig.h"

namespace remote_identification_radar {
namespace internal {

/**
 * @brief RirPropagationResult 表示传播与杂波组合输出。
 */
struct RirPropagationResult {
  float propagation_loss_db{0.0f}; /**< 传播损耗（单位：dB）。 */
  float clutter_power_db{0.0f};    /**< 杂波功率（单位：dB）。 */
};

/**
 * @brief RirEnvironmentSceneState 当前周期环境场景事实（植被覆盖档位）。
 */
struct RirEnvironmentSceneState {
  config::RirVegetationCoverProfile vegetation_cover_profile{
      config::RirVegetationCoverProfile::kDisabled};
};

/**
 * @brief RirPropagationModel 提供组合式传播/杂波建模。
 */
class RirPropagationModel {
 public:
  /**
   * @brief 根据场景状态计算传播与杂波输出。
   * @param[in] scene_state 当前周期场景状态（含植被覆盖档位）。
   * @return 包含传播损耗与杂波功率的 RirPropagationResult。
   * @note 逐目标大气物理损耗由驻留链路预算按真实目标几何计算
   *       （`RirController::ComputeTargetAtmosphericLossDb`，common 大气单源），
   *       环境层不重复计算（避免硬编码几何的死计算）。
   */
  RirPropagationResult Evaluate(const RirEnvironmentSceneState& scene_state) const;
};

}  // namespace internal
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_INTERNAL_RIR_PROPAGATION_MODEL_H_
