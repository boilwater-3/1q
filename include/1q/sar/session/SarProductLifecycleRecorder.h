/**
 * @file SarProductLifecycleRecorder.h
 * @brief 定义 SAR 产品生命周期记录器。
 */

#ifndef ONEQ_SAR_SESSION_SAR_PRODUCT_LIFECYCLE_RECORDER_H_
#define ONEQ_SAR_SESSION_SAR_PRODUCT_LIFECYCLE_RECORDER_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/sar/session/SarCycleResult.h"

namespace sar {
namespace session {

/**
 * @brief SAR 成像产品生命周期事件类型。
 */
enum class SarProductLifecycleEventKind {
  kImageProduced = 0,     /**< 首次产出图像产品 */
  kProductUpdated = 1,    /**< 已有产品被更新 */
  kProductLost = 2,       /**< 已有产品在本周期丢失 */
  kProcessingFailed = 3,  /**< 处理失败 */
  kNoProduct = 4           /**< 本周期无产品 */
};

/**
 * @brief SAR 成像产品生命周期事件归因。
 */
enum class SarProductLifecycleReason {
  kNone = 0,             /**< 无特定原因 */
  kNoImageProduct = 1,   /**< 无图像产品 */
  kAbortReason = 2,      /**< 因 abort 终止 */
  kError = 3             /**< 错误 */
};

/**
 * @brief 单周期产品生命周期事件记录。
 */
struct ONEQ_API SarProductLifecycleEvent {
  std::uint32_t cycle_index{0U};                          /**< 周期序号 */
  SarProductLifecycleEventKind kind{SarProductLifecycleEventKind::kProductUpdated}; /**< 事件类型 */
  SarProductLifecycleReason reason{SarProductLifecycleReason::kNone}; /**< 事件归因 */
  SarProcessingStage completed_stage{SarProcessingStage::kNone}; /**< 本周期完成的处理阶段 */
  std::string abort_reason{};                             /**< abort 原因标签 */
  double estimated_snr_db{0.0};                           /**< 估算 SNR（dB） */
};

/**
 * @brief SAR 产品生命周期记录器配置。
 */
struct ONEQ_API SarProductLifecycleRecorderConfig {
  bool emit_no_product_events{false}; /**< 是否在无产品周期仍发出 kNoProduct 事件 */
};

/**
 * @brief 按成像产品生命周期记录图像生成/更新/丢失/处理失败/无产品事件。
 *
 * 私有状态与实现见 .cpp，避免在 public header 暴露判定逻辑。
 */
class ONEQ_API SarProductLifecycleRecorder {
 public:
  /**
   * @brief 构造记录器。
   * @param[in] config 记录器配置。
   */
  explicit SarProductLifecycleRecorder(
      SarProductLifecycleRecorderConfig config = SarProductLifecycleRecorderConfig{});

  /**
   * @brief 根据本周期结果更新产品生命周期状态并发出事件。
   *
   * 比较当前结果与内部持有的“是否已有产品”状态，判定首次产出、更新、丢失或处理失败，
 * 返回对应的生命周期事件列表；非执行周期不产生事件，也不推进内部状态。
   * @param[in] result 单周期结果。
   * @return 本周期触发的生命周期事件列表。
   */
  std::vector<SarProductLifecycleEvent> Update(const SarCycleResult& result);

  /**
   * @brief 重置内部产品状态（清空“是否已有产品”标记）。
   */
  void Reset();

 private:
  SarProductLifecycleRecorderConfig config_;
  bool has_product_{false};
};

}  // namespace session
}  // namespace sar

#endif  // ONEQ_SAR_SESSION_SAR_PRODUCT_LIFECYCLE_RECORDER_H_
