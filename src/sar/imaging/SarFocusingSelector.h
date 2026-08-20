/**
 * @file SarFocusingSelector.h
 * @brief SAR 内部确定性聚焦算法建议器。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_FOCUSING_SELECTOR_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_FOCUSING_SELECTOR_H_

#include <cstddef>

namespace sar {
namespace imaging {

/**
 * @name 聚焦算法批准的尺寸上限（单一决策源）。
 *
 * session runtime validation 与 focusing selector 共用这两个上限，避免两份副本
 * 各自维护导致漂移。语义为严格 `>`：等于上限值放行，超出才拒绝。
 * @{
 */
constexpr std::size_t kFocusingRdaSizeLimit = 1024U;
constexpr std::size_t kFocusingBackprojectionSizeLimit = 128U;
/** @} */

/**
 * @brief 判定 range/azimuth 维度是否超出给定聚焦尺寸上限。
 *
 * 任一维度严格大于 @p limit 即视为超出（边界值 limit 本身放行）。
 *
 * @param range_samples 距离向采样数。
 * @param azimuth_pulses 方位向脉冲数。
 * @param limit          允许的最大维度（含）。
 * @return 超出返回 true。
 */
inline bool ExceedsFocusingSizeLimit(std::size_t range_samples, std::size_t azimuth_pulses,
                                    std::size_t limit) {
  return range_samples > limit || azimuth_pulses > limit;
}

/**
 * @brief 轨迹保真度等级。
 */
enum class SelectorTrajectoryFidelity {
  kL1 = 1, /**< L1 理想匀速直线轨迹 */
  kL2 = 2, /**< L2 含运动补偿的扰动轨迹 */
  kL3 = 3,  /**< L3 多航点非线性轨迹 */
};

/**
 * @brief 聚焦用途。
 */
enum class SelectorPurpose {
  kRoutineImaging = 0,       /**< 常规成像 */
  kIndependentReference = 1, /**< 独立参考聚焦 */
  kL3NonlinearImaging = 2,    /**< L3 非线性成像 */
};

/**
 * @brief 推荐的聚焦算法。
 */
enum class RecommendedFocusingAlgorithm {
  kNone = 0, /**< 无推荐 */
  kRda = 1,  /**< 距离-多普勒算法 RDA */
  kGbp = 4,  /**< 小场景全局后向投影 GBP */
  kBp = 5,    /**< 逐脉冲后向投影 BP */
};

/**
 * @brief 推荐算法的命中原因。
 */
enum class FocusingSelectionReason {
  kNone = 0,                   /**< 无 */
  kL1RoutineRda = 1,           /**< L1 常规命中 RDA */
  kL2CompensatedRda = 2,       /**< L2 运动补偿后命中 RDA */
  kSmallSceneGbpReference = 3, /**< 小场景参考命中 GBP */
  kL3WaypointBp = 4,            /**< L3 多航点命中 BP */
};

/**
 * @brief 拒绝推荐的原因。
 */
enum class FocusingSelectionRejection {
  kNone = 0,                       /**< 无拒绝 */
  kInvalidSize = 1,                /**< 尺寸非法 */
  kPurposeTrajectoryMismatch = 2,  /**< 用途与轨迹保真度不匹配 */
  kL2CompensationRequired = 3,     /**< 需要启用 L2 运动补偿 */
  kL3WaypointsRequired = 4,        /**< 需要提供 L3 航点 */
  kAlgorithmUnavailable = 5,       /**< 算法不可用 */
  kAlgorithmSizeExceeded = 6,      /**< 算法尺寸超限 */
  kUnsupportedRequest = 7,         /**< 不支持的请求 */
};

/**
 * @brief 聚焦算法选择请求。
 */
struct FocusingSelectionRequest {
  SelectorTrajectoryFidelity trajectory_fidelity{SelectorTrajectoryFidelity::kL1}; /**< 轨迹保真度 */
  SelectorPurpose purpose{SelectorPurpose::kRoutineImaging}; /**< 聚焦用途 */
  std::size_t range_sample_count{0U};  /**< 距离向采样数 */
  std::size_t azimuth_pulse_count{0U}; /**< 方位向脉冲数 */
  bool rda_available{false};           /**< RDA 是否可用 */
  bool gbp_available{false};           /**< GBP 是否可用 */
  bool bp_available{false};            /**< BP 是否可用 */
  bool l2_compensation_enabled{false}; /**< 是否启用 L2 运动补偿 */
  bool l3_waypoints_available{false};  /**< 是否提供 L3 航点 */
};

/**
 * @brief 聚焦算法选择结果。
 */
struct FocusingSelectionResult {
  bool valid{false};                                            /**< 是否选出有效算法 */
  RecommendedFocusingAlgorithm recommended_algorithm{RecommendedFocusingAlgorithm::kNone}; /**< 推荐算法 */
  FocusingSelectionReason reason{FocusingSelectionReason::kNone};         /**< 命中原因 */
  FocusingSelectionRejection rejection{FocusingSelectionRejection::kNone}; /**< 拒绝原因 */
  SelectorTrajectoryFidelity trajectory_fidelity{SelectorTrajectoryFidelity::kL1}; /**< 轨迹保真度 */
  std::size_t range_sample_count{0U};  /**< 距离向采样数 */
  std::size_t azimuth_pulse_count{0U}; /**< 方位向脉冲数 */
};

/**
 * @brief 根据请求确定性推荐聚焦算法。
 * @param[in] request 聚焦选择请求。
 * @return 选择结果（含推荐算法、命中或拒绝原因）。
 */
FocusingSelectionResult RecommendFocusingAlgorithm(const FocusingSelectionRequest& request);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_FOCUSING_SELECTOR_H_
