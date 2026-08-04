// Copyright 2026. All Rights Reserved.
//
// @file ar_recognition_feature_test.cpp
// @brief 验证远程识别观测构造器与四类特征提取器的效能级行为。

#include <gtest/gtest.h>

#include <cmath>
#include <unordered_map>

#include <Eigen/Core>

#include "1q/airborne_radar/session/ArRecognitionResult.h"
#include "1q/airborne_radar/session/ArSceneTypes.h"
#include "1q/airborne_radar/session/TrackStateSnapshot.h"
#include "airborne_radar/recognition/MotionFeatureExtractor.h"
#include "airborne_radar/recognition/PolarizationFeatureExtractor.h"
#include "airborne_radar/recognition/RangeProfileFeatureExtractor.h"
#include "airborne_radar/recognition/RcsFeatureExtractor.h"
#include "airborne_radar/recognition/RecognitionObservationBuilder.h"
#include "airborne_radar/recognition/RecognitionTracker.h"
#include "airborne_radar/recognition/RecognitionTypes.h"
#include "airborne_radar/signal/tracking/BoostTrackPool.h"
#include "airborne_radar/signal/tracking/KalmanPredictor.h"
#include "airborne_radar/signal/tracking/KalmanUpdater.h"
#include "airborne_radar/signal/tracking/TrackLifecycleManager.h"
#include "airborne_radar/signal/tracking/TrackState.h"

namespace airborne_radar {
namespace tests {
namespace {

using session::ArRecognitionFeatureDimension;
using session::ArSceneTarget;
using session::AspectRcsSample;
using session::PolarizationRcsSample;
using session::RangeRcsScatterer;
using session::TrackStateSnapshot;
using session::TrackStatus;

recognition::RecognitionObservationContext MakeContext(float snr_db = 20.0f,
                                                       float bandwidth_hz = 3.0e6f,
                                                       float range_m = 100000.0f) {
  recognition::RecognitionObservationContext context;
  context.snr_db = snr_db;
  context.bandwidth_hz = bandwidth_hz;
  context.range_m = range_m;
  context.dwell_sec = 0.05f;  // 标称驻留
  context.look_az_deg = -30.0f;
  context.look_el_deg = 5.0f;
  return context;
}

TEST(RecognitionObservationBuilderTest, EmptyFeatureListsProduceZeroMask) {
  ArSceneTarget target;
  TrackStateSnapshot snapshot;  // 默认快照（kTentative）
  const recognition::RecognitionFeatureSet set =
      recognition::RecognitionObservationBuilder::Build(target, snapshot, MakeContext());
  EXPECT_EQ(set.valid_feature_mask, 0U);
  EXPECT_FALSE(set.rcs.valid);
  EXPECT_FALSE(set.motion.valid);
  EXPECT_FALSE(set.polarization.valid);
  EXPECT_FALSE(set.range_profile.valid);
}

TEST(RcsFeatureExtractorTest, SingleSampleAtLookAngleYieldsMeanWithinSnrTolerance) {
  AspectRcsSample sample;
  sample.aspect_az_deg = -30.0f;
  sample.aspect_el_deg = 5.0f;
  sample.rcs_dbsm = -3.0f;
  std::vector<AspectRcsSample> samples = {sample};

  const recognition::RcsObservation observation =
      recognition::RcsFeatureExtractor::Extract(samples, -30.0f, 5.0f, 20.0f, 15.0f);

  ASSERT_TRUE(observation.valid);
  EXPECT_LE(std::fabs(observation.mean_dbsm - (-3.0f)), 3.0f);
}

TEST(RcsFeatureExtractorTest, InsufficientAspectCoverageInvalidatesDimension) {
  // 多样本网格方位跨距 5° < 15° 下限。
  std::vector<AspectRcsSample> samples = {
      {-30.0f, 5.0f, -3.0f},
      {-28.0f, 5.0f, -2.5f},
      {-25.0f, 5.0f, -4.0f},
  };

  const recognition::RcsObservation observation =
      recognition::RcsFeatureExtractor::Extract(samples, -30.0f, 5.0f, 20.0f, 15.0f);

  EXPECT_FALSE(observation.valid);
}

TEST(MotionFeatureExtractorTest, ExtractsSpeedAndStraightFlightFromConfirmedTrack) {
  TrackStateSnapshot snapshot;
  snapshot.status = TrackStatus::kConfirmed;
  snapshot.speed = 1800.0f;
  snapshot.acceleration = 12.0f;
  snapshot.velocity_x = 1800.0f;
  snapshot.velocity_y = 0.0f;
  snapshot.velocity_z = 0.0f;
  snapshot.acceleration_x = 12.0f;
  snapshot.acceleration_y = 0.0f;
  snapshot.acceleration_z = 0.0f;
  snapshot.position_z = 20000.0f;

  const recognition::MotionObservation observation =
      recognition::MotionFeatureExtractor::Extract(snapshot, 30000.0f, 2500.0f);

  ASSERT_TRUE(observation.valid);
  EXPECT_FLOAT_EQ(observation.speed_mps, 1800.0f);
  EXPECT_FLOAT_EQ(observation.altitude_m, 50000.0f);
  EXPECT_FLOAT_EQ(observation.acceleration_mps2, 12.0f);
  EXPECT_TRUE(observation.is_straight);
  EXPECT_GT(observation.quality, 0.0f);
}

TEST(MotionFeatureExtractorTest, UnconfirmedTrackIsInvalid) {
  TrackStateSnapshot snapshot;  // kTentative
  const recognition::MotionObservation observation =
      recognition::MotionFeatureExtractor::Extract(snapshot, 0.0f, 0.0f);
  EXPECT_FALSE(observation.valid);
}

TEST(PolarizationFeatureExtractorTest, MissingChannelInvalidatesDimension) {
  // 任一通道 RCS 缺失（样本列表为空）→ 维度无效。
  std::vector<PolarizationRcsSample> samples;
  const recognition::PolarizationObservation observation =
      recognition::PolarizationFeatureExtractor::Extract(samples, -30.0f, 5.0f, 20.0f, 100000.0f);
  EXPECT_FALSE(observation.valid);
}

TEST(PolarizationFeatureExtractorTest, ChannelEnergyDifferenceMatchesRcsRatio) {
  PolarizationRcsSample sample;
  sample.aspect_az_deg = -30.0f;
  sample.aspect_el_deg = 5.0f;
  sample.channel_1_rcs_dbsm = -3.0f;
  sample.channel_2_rcs_dbsm = -6.0f;
  std::vector<PolarizationRcsSample> samples = {sample};

  const recognition::PolarizationObservation observation =
      recognition::PolarizationFeatureExtractor::Extract(samples, -30.0f, 5.0f, 20.0f,
                                                         100000.0f);

  ASSERT_TRUE(observation.valid);
  EXPECT_NEAR(observation.energy_difference_db, 3.0f, 1.5f);
}

TEST(RangeProfileFeatureExtractorTest, ScatterersProduceLengthAndPeakCount) {
  std::vector<RangeRcsScatterer> scatterers;
  const float offsets[] = {-4.0f, -1.0f, 0.0f, 2.0f, 5.0f};
  const float rcs[] = {0.0f, -3.0f, -6.0f, -3.0f, 0.0f};
  for (int i = 0; i < 5; ++i) {
    RangeRcsScatterer scatterer;
    scatterer.range_offset_m = offsets[i];
    scatterer.rcs_dbsm = rcs[i];
    scatterers.push_back(scatterer);
  }

  const recognition::RangeProfileObservation observation =
      recognition::RangeProfileFeatureExtractor::Extract(scatterers, 3.0e6f, 20.0f, 0.0f);

  ASSERT_TRUE(observation.valid);
  EXPECT_NEAR(observation.resolution_m, 50.0f, 0.1f);  // c/(2B) = 2.9979e8/(2·3e6) ≈ 49.97
  EXPECT_NEAR(observation.length_m, 9.0f, 50.0f);  // 容差等于距离分辨率
  EXPECT_EQ(observation.peak_count, 5U);
}

TEST(RangeProfileFeatureExtractorTest, ResolutionCoarserThanLimitInvalidatesDimension) {
  std::vector<RangeRcsScatterer> scatterers;
  RangeRcsScatterer scatterer;
  scatterer.range_offset_m = 0.0f;
  scatterer.rcs_dbsm = 0.0f;
  scatterers.push_back(scatterer);

  // 分辨率 50 m > 允许上限 10 m → 维度无效。
  const recognition::RangeProfileObservation observation =
      recognition::RangeProfileFeatureExtractor::Extract(scatterers, 3.0e6f, 20.0f, 10.0f);

  EXPECT_FALSE(observation.valid);
}

TEST(RecognitionFeatureExtractorTest, InvalidInputsReturnZeroQualityWithoutThrowing) {
  // 各提取器对无效输入返回 quality=0、valid=false，不抛异常。
  const recognition::RcsObservation rcs = recognition::RcsFeatureExtractor::Extract(
      {}, -30.0f, 5.0f, 20.0f, 15.0f);
  EXPECT_EQ(rcs.quality, 0.0f);
  EXPECT_FALSE(rcs.valid);

  TrackStateSnapshot snapshot;
  const recognition::MotionObservation motion =
      recognition::MotionFeatureExtractor::Extract(snapshot, 0.0f, 0.0f);
  EXPECT_EQ(motion.quality, 0.0f);
  EXPECT_FALSE(motion.valid);

  const recognition::PolarizationObservation polarization =
      recognition::PolarizationFeatureExtractor::Extract({}, 0.0f, 0.0f, 20.0f, 100000.0f);
  EXPECT_EQ(polarization.quality, 0.0f);
  EXPECT_FALSE(polarization.valid);

  const recognition::RangeProfileObservation range_profile =
      recognition::RangeProfileFeatureExtractor::Extract({}, 3.0e6f, 20.0f, 0.0f);
  EXPECT_EQ(range_profile.quality, 0.0f);
  EXPECT_FALSE(range_profile.valid);
}

TEST(RecognitionObservationBuilderTest, ConfirmedTrackWithAllFeaturesProducesFullMask) {
  ArSceneTarget target;
  target.aspect_rcs_samples.push_back({-30.0f, 5.0f, -3.0f});
  PolarizationRcsSample polarization;
  polarization.aspect_az_deg = -30.0f;
  polarization.aspect_el_deg = 5.0f;
  polarization.channel_1_rcs_dbsm = -3.0f;
  polarization.channel_2_rcs_dbsm = -6.0f;
  target.polarization_rcs_samples.push_back(polarization);
  target.range_rcs_scatterers.push_back({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});

  TrackStateSnapshot snapshot;
  snapshot.status = TrackStatus::kConfirmed;
  snapshot.speed = 1800.0f;
  snapshot.velocity_x = 1800.0f;

  const recognition::RecognitionFeatureSet set =
      recognition::RecognitionObservationBuilder::Build(target, snapshot, MakeContext());

  EXPECT_EQ(set.valid_feature_mask & static_cast<std::uint8_t>(ArRecognitionFeatureDimension::kRcs),
            static_cast<std::uint8_t>(ArRecognitionFeatureDimension::kRcs));
  EXPECT_EQ(set.valid_feature_mask &
                static_cast<std::uint8_t>(ArRecognitionFeatureDimension::kMotion),
            static_cast<std::uint8_t>(ArRecognitionFeatureDimension::kMotion));
  EXPECT_EQ(set.valid_feature_mask &
                static_cast<std::uint8_t>(ArRecognitionFeatureDimension::kPolarization),
            static_cast<std::uint8_t>(ArRecognitionFeatureDimension::kPolarization));
  EXPECT_EQ(set.valid_feature_mask &
                static_cast<std::uint8_t>(ArRecognitionFeatureDimension::kRangeProfile),
            static_cast<std::uint8_t>(ArRecognitionFeatureDimension::kRangeProfile));
}

TEST(TrackStateSnapshotEmitterTest, ConfirmedTrackExportsPositiveUncertaintyTrace) {
  signal::tracking::BoostTrackPool pool(2, 8);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 1;
  config.max_miss_before_lost = 0;
  config.max_lost_cycles = 3;

  signal::tracking::KalmanPredictorConfig pred_cfg;
  pred_cfg.noise_diff_coeff = 1.0f;
  signal::tracking::KalmanPredictor predictor(pred_cfg);
  signal::tracking::KalmanUpdaterConfig upd_cfg;
  upd_cfg.measurement_noise_std = 10.0f;
  signal::tracking::KalmanUpdater updater(upd_cfg);

  signal::tracking::TrackLifecycleManager manager(pool, config, &predictor, &updater);

  signal::tracking::TrackMeasurement measurement;
  measurement.raw_measurement.association_key = 1U;
  measurement.raw_measurement.position = Eigen::Vector3f(100.0f, 0.0f, 0.0f);
  measurement.raw_measurement.measurement_covariance = Eigen::Matrix3f::Identity() * 100.0f;
  measurement.filtered_feature.velocity = Eigen::Vector3f(10.0f, 0.0f, 0.0f);
  signal::tracking::CycleContext cycle;
  cycle.cycle_index = 1U;
  cycle.batch_id = 1U;
  cycle.dt_sec = 1.0f;
  manager.Update(cycle, {measurement});

  const session::TrackStateSnapshotList snapshots = manager.BuildTrackStateSnapshots();
  ASSERT_EQ(snapshots.size(), 1U);
  EXPECT_EQ(snapshots[0].status, session::TrackStatus::kConfirmed);
  EXPECT_GT(snapshots[0].estimation_uncertainty_trace, 0.0f);
}

// -- 场景 4-7：低 SNR / 低带宽 / 强干扰 / 短驻留门控 -----------------------

TEST(RecognitionScenarioGateTest, LowSnrExcludesRcsAndPolarizationButKeepsMotion) {
  ArSceneTarget target;
  target.aspect_rcs_samples.push_back({0.0f, 20.0f, -3.0f});
  PolarizationRcsSample polarization;
  polarization.aspect_az_deg = 0.0f;
  polarization.aspect_el_deg = 20.0f;
  polarization.channel_1_rcs_dbsm = -3.0f;
  polarization.channel_2_rcs_dbsm = -5.0f;
  target.polarization_rcs_samples.push_back(polarization);

  TrackStateSnapshot snapshot;
  snapshot.status = TrackStatus::kConfirmed;
  snapshot.speed = 100.0f;
  snapshot.velocity_x = 100.0f;

  recognition::RecognitionObservationContext context = MakeContext();
  context.snr_db = 3.0f;  // 低于 6 dB 门限
  context.look_az_deg = 0.0f;
  context.look_el_deg = 20.0f;

  const recognition::RecognitionFeatureSet set =
      recognition::RecognitionObservationBuilder::Build(target, snapshot, context);

  EXPECT_EQ(set.valid_feature_mask & static_cast<std::uint8_t>(ArRecognitionFeatureDimension::kRcs), 0U);
  EXPECT_EQ(set.valid_feature_mask &
                static_cast<std::uint8_t>(ArRecognitionFeatureDimension::kPolarization),
            0U);
  EXPECT_EQ(set.valid_feature_mask & static_cast<std::uint8_t>(ArRecognitionFeatureDimension::kMotion),
            static_cast<std::uint8_t>(ArRecognitionFeatureDimension::kMotion));
}

TEST(RecognitionScenarioGateTest, LowBandwidthExcludesRangeProfile) {
  ArSceneTarget target;
  target.range_rcs_scatterers.push_back({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});
  TrackStateSnapshot snapshot;
  snapshot.status = TrackStatus::kConfirmed;
  snapshot.speed = 100.0f;
  snapshot.velocity_x = 100.0f;

  recognition::RecognitionObservationContext context = MakeContext();
  context.bandwidth_hz = 1.0e6f;  // 分辨率 150 m > 上限 50 m
  context.max_range_resolution_m = 50.0f;
  context.look_az_deg = 0.0f;
  context.look_el_deg = 20.0f;

  const recognition::RecognitionFeatureSet set =
      recognition::RecognitionObservationBuilder::Build(target, snapshot, context);

  EXPECT_EQ(set.valid_feature_mask &
                static_cast<std::uint8_t>(ArRecognitionFeatureDimension::kRangeProfile),
            0U);
}

TEST(RecognitionScenarioGateTest, StrongJammingExcludesPolarization) {
  ArSceneTarget target;
  PolarizationRcsSample polarization;
  polarization.aspect_az_deg = 0.0f;
  polarization.aspect_el_deg = 20.0f;
  polarization.channel_1_rcs_dbsm = -3.0f;
  polarization.channel_2_rcs_dbsm = -5.0f;
  target.polarization_rcs_samples.push_back(polarization);
  TrackStateSnapshot snapshot;
  snapshot.status = TrackStatus::kConfirmed;
  snapshot.speed = 100.0f;
  snapshot.velocity_x = 100.0f;

  recognition::RecognitionObservationContext context = MakeContext();
  // 强干扰（jnr > 20 dB）：有效 SNR 压到门限以下 → 极化维度不可用。
  context.snr_db = 20.0f - 25.0f;
  context.look_az_deg = 0.0f;
  context.look_el_deg = 20.0f;

  const recognition::RecognitionFeatureSet set =
      recognition::RecognitionObservationBuilder::Build(target, snapshot, context);

  EXPECT_EQ(set.valid_feature_mask &
                static_cast<std::uint8_t>(ArRecognitionFeatureDimension::kPolarization),
            0U);
}

TEST(RecognitionScenarioGateTest, ShortDwellLowersQualityAndSlowsObservationGrowth) {
  ArSceneTarget target;
  target.aspect_rcs_samples.push_back({0.0f, 20.0f, -3.0f});
  TrackStateSnapshot snapshot;
  snapshot.status = TrackStatus::kConfirmed;
  snapshot.speed = 100.0f;
  snapshot.velocity_x = 100.0f;

  recognition::RecognitionObservationContext nominal = MakeContext();
  nominal.snr_db = 6.5f;
  nominal.dwell_sec = 0.05f;
  nominal.look_az_deg = 0.0f;
  nominal.look_el_deg = 20.0f;
  const recognition::RecognitionFeatureSet set_nominal =
      recognition::RecognitionObservationBuilder::Build(target, snapshot, nominal);

  recognition::RecognitionObservationContext short_dwell = MakeContext();
  short_dwell.snr_db = 6.5f;
  short_dwell.dwell_sec = 0.01f;
  short_dwell.look_az_deg = 0.0f;
  short_dwell.look_el_deg = 20.0f;
  const recognition::RecognitionFeatureSet set_short =
      recognition::RecognitionObservationBuilder::Build(target, snapshot, short_dwell);

  // 短驻留：质量因子下降。
  ASSERT_TRUE(set_nominal.rcs.valid);
  ASSERT_TRUE(set_short.rcs.valid);
  EXPECT_LT(set_short.rcs.quality, set_nominal.rcs.quality);
  // 短驻留 + 低 SNR：质量低于观测下限 → tracker 不计为观测（增长速率为 0）。
  recognition::RecognitionTracker tracker;
  recognition::RecognitionTracker::Options options;
  options.min_confirmed_hits = 1U;
  options.min_observation_count = 1U;
  options.accumulation_window_sec = 10.0f;
  options.acceptance_score = 0.6f;
  options.minimum_margin = 0.05f;
  options.result_hold_sec = 10.0f;
  options.max_range_m = 1.0e6f;
  tracker.SetOptions(options);

  session::TrackStateSnapshotList track_list;
  session::TrackStateSnapshot track;
  track.association_key = 1U;
  track.status = TrackStatus::kConfirmed;
  track.hit_count = 3U;
  track.speed = 100.0f;
  track.velocity_x = 100.0f;
  track.estimation_uncertainty_trace = 1.0e6f;  // 大不确定度：运动质量也低于下限
  track_list.push_back(track);
  recognition::RecognitionFeatureDatabase database;  // 空库：仅验证计数行为
  std::unordered_map<std::uint64_t, recognition::RecognitionTracker::TrackObservationInput>
      observations;
  recognition::RecognitionTracker::TrackObservationInput input;
  input.target = &target;
  input.context = short_dwell;
  observations[1U] = input;
  for (std::uint32_t cycle = 1U; cycle <= 3U; ++cycle) {
    tracker.UpdateCycle(track_list, observations, database, {}, static_cast<float>(cycle), cycle,
                        1U);
  }
  const session::ArRecognitionResult* result = tracker.FindResult(1U);
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result->observation_count, 0U);  // 短驻留 + 低质量：不计为观测

  // 标称驻留对照：同样低 SNR 下质量高于下限 → 观测正常积累。
  recognition::RecognitionTracker nominal_tracker;
  nominal_tracker.SetOptions(options);
  input.context = nominal;
  std::unordered_map<std::uint64_t, recognition::RecognitionTracker::TrackObservationInput>
      nominal_observations;
  nominal_observations[1U] = input;
  for (std::uint32_t cycle = 1U; cycle <= 3U; ++cycle) {
    nominal_tracker.UpdateCycle(track_list, nominal_observations, database, {},
                                static_cast<float>(cycle), cycle, 1U);
  }
  const session::ArRecognitionResult* nominal_result = nominal_tracker.FindResult(1U);
  ASSERT_NE(nominal_result, nullptr);
  EXPECT_EQ(nominal_result->observation_count, 3U);  // 标称驻留：增长速率正常
}

}  // namespace
}  // namespace tests
}  // namespace airborne_radar
