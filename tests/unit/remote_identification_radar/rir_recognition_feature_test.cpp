// Copyright 2026. All Rights Reserved.
//
// @file rir_recognition_feature_test.cpp
// @brief 验证远程识别观测构造器与四类特征提取器的效能级行为。

#include <gtest/gtest.h>

#include <cmath>
#include <unordered_map>

#include "1q/remote_identification_radar/session/RirRecognitionResult.h"
#include "1q/remote_identification_radar/session/RirSceneTypes.h"
#include "remote_identification_radar/recognition/MotionFeatureExtractor.h"
#include "remote_identification_radar/recognition/PolarizationFeatureExtractor.h"
#include "remote_identification_radar/recognition/RangeProfileFeatureExtractor.h"
#include "remote_identification_radar/recognition/RcsFeatureExtractor.h"
#include "remote_identification_radar/recognition/RecognitionObservationBuilder.h"
#include "remote_identification_radar/recognition/RecognitionTracker.h"
#include "remote_identification_radar/recognition/RecognitionTypes.h"
#include "remote_identification_radar/tracking/RirTrackTypes.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using session::RirAspectRcsSample;
using session::RirPolarizationRcsSample;
using session::RirRangeRcsScatterer;
using session::RirRecognitionFeatureDimension;
using session::RirSceneTarget;
using tracking::RirTrackState;
using tracking::RirTrackStatus;

recognition::RirObservationContext MakeContext(float snr_db = 20.0f, float bandwidth_hz = 3.0e6f,
                                               float range_m = 100000.0f) {
  recognition::RirObservationContext context;
  context.snr_db = snr_db;
  context.bandwidth_hz = bandwidth_hz;
  context.range_m = range_m;
  context.dwell_sec = 0.05f;  // 标称驻留
  context.look_az_deg = -30.0f;
  context.look_el_deg = 5.0f;
  return context;
}

TEST(RirObservationBuilderTest, EmptyFeatureListsProduceZeroMask) {
  RirSceneTarget target;
  RirTrackState snapshot;  // 默认快照（kTentative）
  const recognition::RirFeatureSet set =
      recognition::RirObservationBuilder::Build(target, snapshot, MakeContext());
  EXPECT_EQ(set.valid_feature_mask, 0U);
  EXPECT_FALSE(set.rcs.valid);
  EXPECT_FALSE(set.motion.valid);
  EXPECT_FALSE(set.polarization.valid);
  EXPECT_FALSE(set.range_profile.valid);
}

TEST(RirRcsFeatureExtractorTest, SingleSampleAtLookAngleYieldsMeanWithinSnrTolerance) {
  RirAspectRcsSample sample;
  sample.aspect_az_deg = -30.0f;
  sample.aspect_el_deg = 5.0f;
  sample.rcs_dbsm = -3.0f;
  std::vector<RirAspectRcsSample> samples = {sample};

  const recognition::RirRcsObservation observation =
      recognition::RirRcsFeatureExtractor::Extract(samples, -30.0f, 5.0f, 20.0f, 15.0f);

  ASSERT_TRUE(observation.valid);
  EXPECT_LE(std::fabs(observation.mean_dbsm - (-3.0f)), 3.0f);
}

TEST(RirRcsFeatureExtractorTest, InsufficientAspectCoverageInvalidatesDimension) {
  // 多样本网格方位跨距 5° < 15° 下限。
  std::vector<RirAspectRcsSample> samples = {
      {-30.0f, 5.0f, -3.0f},
      {-28.0f, 5.0f, -2.5f},
      {-25.0f, 5.0f, -4.0f},
  };

  const recognition::RirRcsObservation observation =
      recognition::RirRcsFeatureExtractor::Extract(samples, -30.0f, 5.0f, 20.0f, 15.0f);

  EXPECT_FALSE(observation.valid);
}

TEST(RirMotionFeatureExtractorTest, ExtractsSpeedAndStraightFlightFromConfirmedTrack) {
  RirTrackState snapshot;
  snapshot.status = RirTrackStatus::kConfirmed;
  snapshot.speed = 1800.0f;
  snapshot.acceleration_mps2 = 12.0f;
  snapshot.velocity.x() = 1800.0f;
  snapshot.velocity.y() = 0.0f;
  snapshot.velocity.z() = 0.0f;
  snapshot.acceleration.x() = 12.0f;
  snapshot.acceleration.y() = 0.0f;
  snapshot.acceleration.z() = 0.0f;
  snapshot.position.z() = 20000.0f;

  const recognition::RirMotionObservation observation =
      recognition::RirMotionFeatureExtractor::Extract(snapshot, 30000.0f, 2500.0f);

  ASSERT_TRUE(observation.valid);
  EXPECT_FLOAT_EQ(observation.speed_mps, 1800.0f);
  EXPECT_FLOAT_EQ(observation.altitude_m, 50000.0f);
  EXPECT_FLOAT_EQ(observation.acceleration_mps2, 12.0f);
  EXPECT_TRUE(observation.is_straight);
  EXPECT_GT(observation.quality, 0.0f);
}

TEST(RirMotionFeatureExtractorTest, UnconfirmedTrackIsInvalid) {
  RirTrackState snapshot;  // kTentative
  const recognition::RirMotionObservation observation =
      recognition::RirMotionFeatureExtractor::Extract(snapshot, 0.0f, 0.0f);
  EXPECT_FALSE(observation.valid);
}

TEST(RirPolarizationFeatureExtractorTest, MissingChannelInvalidatesDimension) {
  // 任一通道 RCS 缺失（样本列表为空）→ 维度无效。
  std::vector<RirPolarizationRcsSample> samples;
  const recognition::RirPolarizationObservation observation =
      recognition::RirPolarizationFeatureExtractor::Extract(samples, -30.0f, 5.0f, 20.0f,
                                                            100000.0f);
  EXPECT_FALSE(observation.valid);
}

TEST(RirPolarizationFeatureExtractorTest, ChannelEnergyDifferenceMatchesRcsRatio) {
  RirPolarizationRcsSample sample;
  sample.aspect_az_deg = -30.0f;
  sample.aspect_el_deg = 5.0f;
  sample.channel_1_rcs_dbsm = -3.0f;
  sample.channel_2_rcs_dbsm = -6.0f;
  std::vector<RirPolarizationRcsSample> samples = {sample};

  const recognition::RirPolarizationObservation observation =
      recognition::RirPolarizationFeatureExtractor::Extract(samples, -30.0f, 5.0f, 20.0f,
                                                            100000.0f);

  ASSERT_TRUE(observation.valid);
  EXPECT_NEAR(observation.energy_difference_db, 3.0f, 1.5f);
}

TEST(RirRangeProfileFeatureExtractorTest, ScatterersProduceLengthAndPeakCount) {
  std::vector<RirRangeRcsScatterer> scatterers;
  const float offsets[] = {-4.0f, -1.0f, 0.0f, 2.0f, 5.0f};
  const float rcs[] = {0.0f, -3.0f, -6.0f, -3.0f, 0.0f};
  for (int i = 0; i < 5; ++i) {
    RirRangeRcsScatterer scatterer;
    scatterer.range_offset_m = offsets[i];
    scatterer.rcs_dbsm = rcs[i];
    scatterers.push_back(scatterer);
  }

  const recognition::RirRangeProfileObservation observation =
      recognition::RirRangeProfileFeatureExtractor::Extract(scatterers, 3.0e6f, 20.0f, 0.0f);

  ASSERT_TRUE(observation.valid);
  EXPECT_NEAR(observation.resolution_m, 50.0f, 0.1f);  // c/(2B) = 2.9979e8/(2·3e6) ≈ 49.97
  EXPECT_NEAR(observation.length_m, 9.0f, 50.0f);      // 容差等于距离分辨率
  EXPECT_EQ(observation.peak_count, 5U);
}

TEST(RirRangeProfileFeatureExtractorTest, ResolutionCoarserThanLimitInvalidatesDimension) {
  std::vector<RirRangeRcsScatterer> scatterers;
  RirRangeRcsScatterer scatterer;
  scatterer.range_offset_m = 0.0f;
  scatterer.rcs_dbsm = 0.0f;
  scatterers.push_back(scatterer);

  // 分辨率 50 m > 允许上限 10 m → 维度无效。
  const recognition::RirRangeProfileObservation observation =
      recognition::RirRangeProfileFeatureExtractor::Extract(scatterers, 3.0e6f, 20.0f, 10.0f);

  EXPECT_FALSE(observation.valid);
}

TEST(RecognitionFeatureExtractorTest, InvalidInputsReturnZeroQualityWithoutThrowing) {
  // 各提取器对无效输入返回 quality=0、valid=false，不抛异常。
  const recognition::RirRcsObservation rcs =
      recognition::RirRcsFeatureExtractor::Extract({}, -30.0f, 5.0f, 20.0f, 15.0f);
  EXPECT_EQ(rcs.quality, 0.0f);
  EXPECT_FALSE(rcs.valid);

  RirTrackState snapshot;
  const recognition::RirMotionObservation motion =
      recognition::RirMotionFeatureExtractor::Extract(snapshot, 0.0f, 0.0f);
  EXPECT_EQ(motion.quality, 0.0f);
  EXPECT_FALSE(motion.valid);

  const recognition::RirPolarizationObservation polarization =
      recognition::RirPolarizationFeatureExtractor::Extract({}, 0.0f, 0.0f, 20.0f, 100000.0f);
  EXPECT_EQ(polarization.quality, 0.0f);
  EXPECT_FALSE(polarization.valid);

  const recognition::RirRangeProfileObservation range_profile =
      recognition::RirRangeProfileFeatureExtractor::Extract({}, 3.0e6f, 20.0f, 0.0f);
  EXPECT_EQ(range_profile.quality, 0.0f);
  EXPECT_FALSE(range_profile.valid);
}

TEST(RirObservationBuilderTest, ConfirmedTrackWithAllFeaturesProducesFullMask) {
  RirSceneTarget target;
  target.aspect_rcs_samples.push_back({-30.0f, 5.0f, -3.0f});
  RirPolarizationRcsSample polarization;
  polarization.aspect_az_deg = -30.0f;
  polarization.aspect_el_deg = 5.0f;
  polarization.channel_1_rcs_dbsm = -3.0f;
  polarization.channel_2_rcs_dbsm = -6.0f;
  target.polarization_rcs_samples.push_back(polarization);
  target.range_rcs_scatterers.push_back({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});

  RirTrackState snapshot;
  snapshot.status = RirTrackStatus::kConfirmed;
  snapshot.speed = 1800.0f;
  snapshot.velocity.x() = 1800.0f;

  const recognition::RirFeatureSet set =
      recognition::RirObservationBuilder::Build(target, snapshot, MakeContext());

  EXPECT_EQ(
      set.valid_feature_mask & static_cast<std::uint8_t>(RirRecognitionFeatureDimension::kRcs),
      static_cast<std::uint8_t>(RirRecognitionFeatureDimension::kRcs));
  EXPECT_EQ(
      set.valid_feature_mask & static_cast<std::uint8_t>(RirRecognitionFeatureDimension::kMotion),
      static_cast<std::uint8_t>(RirRecognitionFeatureDimension::kMotion));
  EXPECT_EQ(set.valid_feature_mask &
                static_cast<std::uint8_t>(RirRecognitionFeatureDimension::kPolarization),
            static_cast<std::uint8_t>(RirRecognitionFeatureDimension::kPolarization));
  EXPECT_EQ(set.valid_feature_mask &
                static_cast<std::uint8_t>(RirRecognitionFeatureDimension::kRangeProfile),
            static_cast<std::uint8_t>(RirRecognitionFeatureDimension::kRangeProfile));
}

// -- 场景 4-7：低 SNR / 低带宽 / 强干扰 / 短驻留门控 -----------------------
// （原 TrackStateSnapshotEmitterTest 属 AR 航迹快照发射器行为，随解耦回归 AR，
//   不由本模块承担；RIR 侧运动特征不确定性由内部航迹协方差供给。）

TEST(RecognitionScenarioGateTest, LowSnrExcludesRcsAndPolarizationButKeepsMotion) {
  RirSceneTarget target;
  target.aspect_rcs_samples.push_back({0.0f, 20.0f, -3.0f});
  RirPolarizationRcsSample polarization;
  polarization.aspect_az_deg = 0.0f;
  polarization.aspect_el_deg = 20.0f;
  polarization.channel_1_rcs_dbsm = -3.0f;
  polarization.channel_2_rcs_dbsm = -5.0f;
  target.polarization_rcs_samples.push_back(polarization);

  RirTrackState snapshot;
  snapshot.status = RirTrackStatus::kConfirmed;
  snapshot.speed = 100.0f;
  snapshot.velocity.x() = 100.0f;

  recognition::RirObservationContext context = MakeContext();
  context.snr_db = 3.0f;  // 低于 6 dB 门限
  context.look_az_deg = 0.0f;
  context.look_el_deg = 20.0f;

  const recognition::RirFeatureSet set =
      recognition::RirObservationBuilder::Build(target, snapshot, context);

  EXPECT_EQ(
      set.valid_feature_mask & static_cast<std::uint8_t>(RirRecognitionFeatureDimension::kRcs), 0U);
  EXPECT_EQ(set.valid_feature_mask &
                static_cast<std::uint8_t>(RirRecognitionFeatureDimension::kPolarization),
            0U);
  EXPECT_EQ(
      set.valid_feature_mask & static_cast<std::uint8_t>(RirRecognitionFeatureDimension::kMotion),
      static_cast<std::uint8_t>(RirRecognitionFeatureDimension::kMotion));
}

TEST(RecognitionScenarioGateTest, LowBandwidthExcludesRangeProfile) {
  RirSceneTarget target;
  target.range_rcs_scatterers.push_back({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});
  RirTrackState snapshot;
  snapshot.status = RirTrackStatus::kConfirmed;
  snapshot.speed = 100.0f;
  snapshot.velocity.x() = 100.0f;

  recognition::RirObservationContext context = MakeContext();
  context.bandwidth_hz = 1.0e6f;  // 分辨率 150 m > 上限 50 m
  context.max_range_resolution_m = 50.0f;
  context.look_az_deg = 0.0f;
  context.look_el_deg = 20.0f;

  const recognition::RirFeatureSet set =
      recognition::RirObservationBuilder::Build(target, snapshot, context);

  EXPECT_EQ(set.valid_feature_mask &
                static_cast<std::uint8_t>(RirRecognitionFeatureDimension::kRangeProfile),
            0U);
}

TEST(RecognitionScenarioGateTest, StrongJammingExcludesPolarization) {
  RirSceneTarget target;
  RirPolarizationRcsSample polarization;
  polarization.aspect_az_deg = 0.0f;
  polarization.aspect_el_deg = 20.0f;
  polarization.channel_1_rcs_dbsm = -3.0f;
  polarization.channel_2_rcs_dbsm = -5.0f;
  target.polarization_rcs_samples.push_back(polarization);
  RirTrackState snapshot;
  snapshot.status = RirTrackStatus::kConfirmed;
  snapshot.speed = 100.0f;
  snapshot.velocity.x() = 100.0f;

  recognition::RirObservationContext context = MakeContext();
  // 强干扰（jnr > 20 dB）：有效 SNR 压到门限以下 → 极化维度不可用。
  context.snr_db = 20.0f - 25.0f;
  context.look_az_deg = 0.0f;
  context.look_el_deg = 20.0f;

  const recognition::RirFeatureSet set =
      recognition::RirObservationBuilder::Build(target, snapshot, context);

  EXPECT_EQ(set.valid_feature_mask &
                static_cast<std::uint8_t>(RirRecognitionFeatureDimension::kPolarization),
            0U);
}

TEST(RecognitionScenarioGateTest, ShortDwellLowersQualityAndSlowsObservationGrowth) {
  RirSceneTarget target;
  target.aspect_rcs_samples.push_back({0.0f, 20.0f, -3.0f});
  RirTrackState snapshot;
  snapshot.status = RirTrackStatus::kConfirmed;
  snapshot.speed = 100.0f;
  snapshot.velocity.x() = 100.0f;

  recognition::RirObservationContext nominal = MakeContext();
  nominal.snr_db = 6.5f;
  nominal.dwell_sec = 0.05f;
  nominal.look_az_deg = 0.0f;
  nominal.look_el_deg = 20.0f;
  const recognition::RirFeatureSet set_nominal =
      recognition::RirObservationBuilder::Build(target, snapshot, nominal);

  recognition::RirObservationContext short_dwell = MakeContext();
  short_dwell.snr_db = 6.5f;
  short_dwell.dwell_sec = 0.01f;
  short_dwell.look_az_deg = 0.0f;
  short_dwell.look_el_deg = 20.0f;
  const recognition::RirFeatureSet set_short =
      recognition::RirObservationBuilder::Build(target, snapshot, short_dwell);

  // 短驻留：质量因子下降。
  ASSERT_TRUE(set_nominal.rcs.valid);
  ASSERT_TRUE(set_short.rcs.valid);
  EXPECT_LT(set_short.rcs.quality, set_nominal.rcs.quality);
  // 短驻留 + 低 SNR：质量低于观测下限 → tracker 不计为观测（增长速率为 0）。
  recognition::RirTracker tracker;
  recognition::RirTracker::Options options;
  options.min_confirmed_hits = 1U;
  options.min_observation_count = 1U;
  options.accumulation_window_sec = 10.0f;
  options.acceptance_score = 0.6f;
  options.minimum_margin = 0.05f;
  options.result_hold_sec = 10.0f;
  options.max_range_m = 1.0e6f;
  tracker.SetOptions(options);

  tracking::RirTrackSnapshotList track_list;
  tracking::RirTrackState track;
  track.association_key = 1U;
  track.status = RirTrackStatus::kConfirmed;
  track.hit_count = 3U;
  track.speed = 100.0f;
  track.velocity.x() = 100.0f;
  track.gaussian_state.covariance =
      tracking::RirStateCovariance::Identity() * (1.0e6f / 3.0f);  // 大不确定度：运动质量也低于下限
  track_list.push_back(track);
  recognition::RirFeatureDatabase database;  // 空库：仅验证计数行为
  std::unordered_map<std::uint64_t, recognition::RirTracker::TrackObservationInput> observations;
  recognition::RirTracker::TrackObservationInput input;
  input.target = &target;
  input.context = short_dwell;
  observations[1U] = input;
  for (std::uint32_t cycle = 1U; cycle <= 3U; ++cycle) {
    tracker.UpdateCycle(track_list, observations, database, {}, static_cast<float>(cycle), cycle,
                        1U);
  }
  const session::RirRecognitionResult* result = tracker.FindResult(1U);
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result->observation_count, 0U);  // 短驻留 + 低质量：不计为观测

  // 标称驻留对照：同样低 SNR 下质量高于下限 → 观测正常积累。
  recognition::RirTracker nominal_tracker;
  nominal_tracker.SetOptions(options);
  input.context = nominal;
  std::unordered_map<std::uint64_t, recognition::RirTracker::TrackObservationInput>
      nominal_observations;
  nominal_observations[1U] = input;
  for (std::uint32_t cycle = 1U; cycle <= 3U; ++cycle) {
    nominal_tracker.UpdateCycle(track_list, nominal_observations, database, {},
                                static_cast<float>(cycle), cycle, 1U);
  }
  const session::RirRecognitionResult* nominal_result = nominal_tracker.FindResult(1U);
  ASSERT_NE(nominal_result, nullptr);
  EXPECT_EQ(nominal_result->observation_count, 3U);  // 标称驻留：增长速率正常
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
