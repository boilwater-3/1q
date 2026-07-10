#include <gtest/gtest.h>

#include "sar/imaging/SarFocusingSelector.h"

namespace sar {
namespace imaging {
namespace {

FocusingSelectionRequest BaseRequest() {
  FocusingSelectionRequest request;
  request.range_sample_count = 64U;
  request.azimuth_pulse_count = 64U;
  request.rda_available = true;
  request.gbp_available = true;
  request.bp_available = true;
  return request;
}

TEST(SarFocusingSelectorTest, RecommendsApprovedExplicitPaths) {
  FocusingSelectionRequest request = BaseRequest();
  FocusingSelectionResult result = RecommendFocusingAlgorithm(request);
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.recommended_algorithm, RecommendedFocusingAlgorithm::kRda);
  EXPECT_EQ(result.reason, FocusingSelectionReason::kL1RoutineRda);

  request.trajectory_fidelity = SelectorTrajectoryFidelity::kL2;
  request.l2_compensation_enabled = true;
  result = RecommendFocusingAlgorithm(request);
  EXPECT_EQ(result.recommended_algorithm, RecommendedFocusingAlgorithm::kRda);
  EXPECT_EQ(result.reason, FocusingSelectionReason::kL2CompensatedRda);

  request = BaseRequest();
  request.purpose = SelectorPurpose::kIndependentReference;
  result = RecommendFocusingAlgorithm(request);
  EXPECT_EQ(result.recommended_algorithm, RecommendedFocusingAlgorithm::kGbp);

  request = BaseRequest();
  request.trajectory_fidelity = SelectorTrajectoryFidelity::kL3;
  request.purpose = SelectorPurpose::kL3NonlinearImaging;
  request.l3_waypoints_available = true;
  result = RecommendFocusingAlgorithm(request);
  EXPECT_EQ(result.recommended_algorithm, RecommendedFocusingAlgorithm::kBp);
}

TEST(SarFocusingSelectorTest, RejectsMissingPrerequisitesWithoutFallback) {
  FocusingSelectionRequest request = BaseRequest();
  request.trajectory_fidelity = SelectorTrajectoryFidelity::kL2;
  FocusingSelectionResult result = RecommendFocusingAlgorithm(request);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.rejection, FocusingSelectionRejection::kL2CompensationRequired);
  EXPECT_EQ(result.recommended_algorithm, RecommendedFocusingAlgorithm::kNone);

  request = BaseRequest();
  request.trajectory_fidelity = SelectorTrajectoryFidelity::kL3;
  result = RecommendFocusingAlgorithm(request);
  EXPECT_EQ(result.rejection, FocusingSelectionRejection::kPurposeTrajectoryMismatch);

  request.purpose = SelectorPurpose::kL3NonlinearImaging;
  result = RecommendFocusingAlgorithm(request);
  EXPECT_EQ(result.rejection, FocusingSelectionRejection::kL3WaypointsRequired);

  request.l3_waypoints_available = true;
  request.bp_available = false;
  result = RecommendFocusingAlgorithm(request);
  EXPECT_EQ(result.rejection, FocusingSelectionRejection::kAlgorithmUnavailable);
  EXPECT_EQ(result.recommended_algorithm, RecommendedFocusingAlgorithm::kNone);
}

TEST(SarFocusingSelectorTest, EnforcesEachAlgorithmsApprovedSizeBoundary) {
  FocusingSelectionRequest request = BaseRequest();
  request.range_sample_count = 1024U;
  request.azimuth_pulse_count = 1024U;
  EXPECT_TRUE(RecommendFocusingAlgorithm(request).valid);
  request.range_sample_count = 1025U;
  EXPECT_EQ(RecommendFocusingAlgorithm(request).rejection,
            FocusingSelectionRejection::kAlgorithmSizeExceeded);

  request = BaseRequest();
  request.purpose = SelectorPurpose::kIndependentReference;
  request.range_sample_count = 128U;
  request.azimuth_pulse_count = 128U;
  EXPECT_TRUE(RecommendFocusingAlgorithm(request).valid);
  request.azimuth_pulse_count = 129U;
  EXPECT_EQ(RecommendFocusingAlgorithm(request).rejection,
            FocusingSelectionRejection::kAlgorithmSizeExceeded);
}

TEST(SarFocusingSelectorTest, IsDeterministicAndDoesNotModifyRequest) {
  const FocusingSelectionRequest request = BaseRequest();
  const FocusingSelectionResult first = RecommendFocusingAlgorithm(request);
  const FocusingSelectionResult second = RecommendFocusingAlgorithm(request);
  EXPECT_EQ(first.valid, second.valid);
  EXPECT_EQ(first.recommended_algorithm, second.recommended_algorithm);
  EXPECT_EQ(first.reason, second.reason);
  EXPECT_EQ(first.rejection, second.rejection);
  EXPECT_EQ(request.range_sample_count, 64U);
  EXPECT_EQ(request.azimuth_pulse_count, 64U);
}

// ---- 共享 size 常量与边界判定的独立护栏 -----------------------------------
// 这些常量同时被 focusing selector 与 session runtime validation 消费；本组用例
// 直接锁定其值与严格 `>` 边界语义（上限值放行、上限+1 拒绝），任何一侧被改动都会
// 立即报红，而不必依赖两条调用路径间接捕获。

TEST(SarFocusingSelectorTest, SharedSizeLimitsHoldApprovedValues) {
  EXPECT_EQ(kFocusingRdaSizeLimit, 1024U);
  EXPECT_EQ(kFocusingBackprojectionSizeLimit, 128U);
}

TEST(SarFocusingSelectorTest, ExceedsFocusingSizeLimitAllowsBoundaryAndRejectsOverflow) {
  // 边界值本身放行（严格 > 语义）。
  EXPECT_FALSE(ExceedsFocusingSizeLimit(kFocusingRdaSizeLimit, kFocusingRdaSizeLimit,
                                        kFocusingRdaSizeLimit));
  EXPECT_FALSE(ExceedsFocusingSizeLimit(kFocusingBackprojectionSizeLimit,
                                        kFocusingBackprojectionSizeLimit,
                                        kFocusingBackprojectionSizeLimit));
  // 任一维度超出即拒绝。
  EXPECT_TRUE(ExceedsFocusingSizeLimit(kFocusingRdaSizeLimit + 1U, kFocusingRdaSizeLimit,
                                       kFocusingRdaSizeLimit));
  EXPECT_TRUE(ExceedsFocusingSizeLimit(kFocusingRdaSizeLimit, kFocusingRdaSizeLimit + 1U,
                                       kFocusingRdaSizeLimit));
  EXPECT_TRUE(ExceedsFocusingSizeLimit(kFocusingBackprojectionSizeLimit + 1U, 0U,
                                       kFocusingBackprojectionSizeLimit));
}

TEST(SarFocusingSelectorTest, ExceedsFocusingSizeLimitAllowsZeroAndSmallDimensions) {
  EXPECT_FALSE(ExceedsFocusingSizeLimit(0U, 0U, kFocusingBackprojectionSizeLimit));
  EXPECT_FALSE(ExceedsFocusingSizeLimit(1U, 1U, kFocusingRdaSizeLimit));
}

}  // namespace
}  // namespace imaging
}  // namespace sar
