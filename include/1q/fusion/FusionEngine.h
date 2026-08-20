/**
 * @file FusionEngine.h
 * @brief 定义多源关联 + 置信度融合引擎。
 */

#ifndef ONEQ_FUSION_FUSION_ENGINE_H_
#define ONEQ_FUSION_FUSION_ENGINE_H_

#include <cstdint>
#include <memory>
#include <vector>

#include "1q/api.hpp"
#include "1q/fusion/DetectionRecord.h"
#include "1q/fusion/FusionConfig.h"
#include "1q/fusion/FusedTarget.h"

namespace fusion {

/**
 * @brief 多源融合引擎（关联 + 置信度融合，增量式，带滑窗）。
 * @details 关联分层（冻结）：① 身份键直挂（调用方保证跨源一致）；② 无身份带位置 →
 *          nanoflann KD-tree 半径搜索；③ 仅方位 → 方位相干门限；④ 特征相似度门限。
 *          未关联探测创建新航迹（无身份航迹使用引擎合成键，≥ 2^63）。
 *          每周期成本 O(N log N + M)。
 * @note 同周期新建航迹不参与本周期后续关联（自下周期起）；
 *       无身份探测之间本周期互不合并（合并依赖身份键或后续周期航迹关联）。
 */
class ONEQ_API FusionEngine {
 public:
  /**
   * @brief 构造融合引擎。
   * @param[in] config 融合配置。
   */
  explicit FusionEngine(const FusionConfig& config);

  ~FusionEngine();

  FusionEngine(const FusionEngine&) = delete;
  FusionEngine& operator=(const FusionEngine&) = delete;

  /**
   * @brief 输入一个周期的探测记录，输出当前全部航迹的融合态势。
   * @param[in] detections 本周期探测记录列表。
   * @param[in] cycle 周期号（调用方单调递增）。
   * @return 融合目标态势列表（按航迹键升序，确定性输出）。
   */
  std::vector<FusedTarget> Update(const std::vector<DetectionRecord>& detections,
                                  std::uint64_t cycle);

  /**
   * @brief 清空全部航迹状态与合成键计数器。
   */
  void Reset();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace fusion

#endif  // ONEQ_FUSION_FUSION_ENGINE_H_
