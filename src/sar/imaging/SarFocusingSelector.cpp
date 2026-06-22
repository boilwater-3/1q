// ============================================================================
// 【未进行设计需求，不再扩展 — DEPRECATED】
// 本文件不参与构建（见 src/sar/CMakeLists.txt 的 SAR_ENGINE_SOURCES 注释），
// 仅作为探索性参考保留。请勿新增依赖或据此实施。
// ============================================================================

#include "sar/imaging/SarFocusingSelector.h"

namespace sar {
namespace imaging {

namespace {

constexpr std::size_t kRdaLimit = 1024U;
constexpr std::size_t kBackprojectionLimit = 128U;

FocusingSelectionResult Reject(const FocusingSelectionRequest& request,
                               FocusingSelectionRejection rejection) {
  FocusingSelectionResult result;
  result.rejection = rejection;
  result.trajectory_fidelity = request.trajectory_fidelity;
  result.range_sample_count = request.range_sample_count;
  result.azimuth_pulse_count = request.azimuth_pulse_count;
  return result;
}

FocusingSelectionResult Recommend(const FocusingSelectionRequest& request,
                                  RecommendedFocusingAlgorithm algorithm,
                                  FocusingSelectionReason reason) {
  FocusingSelectionResult result;
  result.valid = true;
  result.recommended_algorithm = algorithm;
  result.reason = reason;
  result.trajectory_fidelity = request.trajectory_fidelity;
  result.range_sample_count = request.range_sample_count;
  result.azimuth_pulse_count = request.azimuth_pulse_count;
  return result;
}

bool Exceeds(std::size_t rows, std::size_t cols, std::size_t limit) {
  return rows > limit || cols > limit;
}

}  // namespace

FocusingSelectionResult RecommendFocusingAlgorithm(const FocusingSelectionRequest& request) {
  if (request.range_sample_count == 0U || request.azimuth_pulse_count == 0U) {
    return Reject(request, FocusingSelectionRejection::kInvalidSize);
  }

  if (request.purpose == SelectorPurpose::kIndependentReference) {
    if (!request.gbp_available) {
      return Reject(request, FocusingSelectionRejection::kAlgorithmUnavailable);
    }
    if (Exceeds(request.range_sample_count, request.azimuth_pulse_count, kBackprojectionLimit)) {
      return Reject(request, FocusingSelectionRejection::kAlgorithmSizeExceeded);
    }
    return Recommend(request, RecommendedFocusingAlgorithm::kGbp,
                     FocusingSelectionReason::kSmallSceneGbpReference);
  }

  if (request.purpose == SelectorPurpose::kL3NonlinearImaging) {
    if (request.trajectory_fidelity != SelectorTrajectoryFidelity::kL3) {
      return Reject(request, FocusingSelectionRejection::kPurposeTrajectoryMismatch);
    }
    if (!request.l3_waypoints_available) {
      return Reject(request, FocusingSelectionRejection::kL3WaypointsRequired);
    }
    if (!request.bp_available) {
      return Reject(request, FocusingSelectionRejection::kAlgorithmUnavailable);
    }
    if (Exceeds(request.range_sample_count, request.azimuth_pulse_count, kBackprojectionLimit)) {
      return Reject(request, FocusingSelectionRejection::kAlgorithmSizeExceeded);
    }
    return Recommend(request, RecommendedFocusingAlgorithm::kBp,
                     FocusingSelectionReason::kL3WaypointBp);
  }

  if (request.purpose != SelectorPurpose::kRoutineImaging) {
    return Reject(request, FocusingSelectionRejection::kUnsupportedRequest);
  }
  if (request.trajectory_fidelity == SelectorTrajectoryFidelity::kL3) {
    return Reject(request, FocusingSelectionRejection::kPurposeTrajectoryMismatch);
  }
  if (request.trajectory_fidelity == SelectorTrajectoryFidelity::kL2 &&
      !request.l2_compensation_enabled) {
    return Reject(request, FocusingSelectionRejection::kL2CompensationRequired);
  }
  if (!request.rda_available) {
    return Reject(request, FocusingSelectionRejection::kAlgorithmUnavailable);
  }
  if (Exceeds(request.range_sample_count, request.azimuth_pulse_count, kRdaLimit)) {
    return Reject(request, FocusingSelectionRejection::kAlgorithmSizeExceeded);
  }
  const FocusingSelectionReason reason =
      request.trajectory_fidelity == SelectorTrajectoryFidelity::kL2
          ? FocusingSelectionReason::kL2CompensatedRda
          : FocusingSelectionReason::kL1RoutineRda;
  return Recommend(request, RecommendedFocusingAlgorithm::kRda, reason);
}

}  // namespace imaging
}  // namespace sar
