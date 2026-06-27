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

enum class SelectorTrajectoryFidelity {
  kL1 = 1,
  kL2 = 2,
  kL3 = 3,
};

enum class SelectorPurpose {
  kRoutineImaging = 0,
  kIndependentReference = 1,
  kL3NonlinearImaging = 2,
};

enum class RecommendedFocusingAlgorithm {
  kNone = 0,
  kRda = 1,
  kGbp = 4,
  kBp = 5,
};

enum class FocusingSelectionReason {
  kNone = 0,
  kL1RoutineRda = 1,
  kL2CompensatedRda = 2,
  kSmallSceneGbpReference = 3,
  kL3WaypointBp = 4,
};

enum class FocusingSelectionRejection {
  kNone = 0,
  kInvalidSize = 1,
  kPurposeTrajectoryMismatch = 2,
  kL2CompensationRequired = 3,
  kL3WaypointsRequired = 4,
  kAlgorithmUnavailable = 5,
  kAlgorithmSizeExceeded = 6,
  kUnsupportedRequest = 7,
};

struct FocusingSelectionRequest {
  SelectorTrajectoryFidelity trajectory_fidelity{SelectorTrajectoryFidelity::kL1};
  SelectorPurpose purpose{SelectorPurpose::kRoutineImaging};
  std::size_t range_sample_count{0U};
  std::size_t azimuth_pulse_count{0U};
  bool rda_available{false};
  bool gbp_available{false};
  bool bp_available{false};
  bool l2_compensation_enabled{false};
  bool l3_waypoints_available{false};
};

struct FocusingSelectionResult {
  bool valid{false};
  RecommendedFocusingAlgorithm recommended_algorithm{RecommendedFocusingAlgorithm::kNone};
  FocusingSelectionReason reason{FocusingSelectionReason::kNone};
  FocusingSelectionRejection rejection{FocusingSelectionRejection::kNone};
  SelectorTrajectoryFidelity trajectory_fidelity{SelectorTrajectoryFidelity::kL1};
  std::size_t range_sample_count{0U};
  std::size_t azimuth_pulse_count{0U};
};

FocusingSelectionResult RecommendFocusingAlgorithm(const FocusingSelectionRequest& request);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_FOCUSING_SELECTOR_H_
