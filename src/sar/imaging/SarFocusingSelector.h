// ============================================================================
// 【未进行设计需求，不再扩展 — DEPRECATED】
// 本文件不参与构建（见 src/sar/CMakeLists.txt 的 SAR_ENGINE_SOURCES 注释），
// 仅作为探索性参考保留。请勿新增依赖或据此实施。
// ============================================================================

/**
 * @file SarFocusingSelector.h
 * @brief SAR 内部确定性聚焦算法建议器。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_FOCUSING_SELECTOR_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_FOCUSING_SELECTOR_H_

#include <cstddef>

namespace sar {
namespace imaging {

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
