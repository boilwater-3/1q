// Copyright 2026. All Rights Reserved.
//
// @file track_lifecycle_test.cpp
// @brief 验证轨迹对象池与生命周期管理的基础状态机行为。

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "1q/airborne_radar/session/ArSceneTypes.h"
#include "airborne_radar/signal/tracking/BoostTrackPool.h"
#include "airborne_radar/signal/tracking/KalmanPredictor.h"
#include "airborne_radar/signal/tracking/KalmanUpdater.h"
#include "airborne_radar/signal/tracking/SynchronizedTrackPool.h"
#include "airborne_radar/signal/tracking/TrackLifecycleManager.h"
#include "airborne_radar/signal/tracking/TrackState.h"

namespace airborne_radar {
namespace tests {

namespace {

float SpeedOf(const session::ArSceneTarget& target) {
  return std::sqrt(target.velocity_x * target.velocity_x + target.velocity_y * target.velocity_y +
                   target.velocity_z * target.velocity_z);
}

class CountingTrackPool : public signal::tracking::ITrackPool {
 public:
  explicit CountingTrackPool(std::size_t capacity) : storage_(capacity) {
    free_list_.reserve(storage_.size());
    for (std::vector<signal::tracking::TrackState>::iterator it = storage_.begin();
         it != storage_.end(); ++it) {
      free_list_.push_back(&(*it));
    }
  }

  signal::tracking::TrackState* Acquire() override {
    ++acquire_calls_;
    if (free_list_.empty()) {
      return nullptr;
    }

    signal::tracking::TrackState* track = free_list_.back();
    free_list_.pop_back();
    *track = signal::tracking::TrackState{};
    ++in_use_count_;
    return track;
  }

  void Release(signal::tracking::TrackState* track) override {
    ++release_calls_;
    if (track == nullptr) {
      return;
    }

    free_list_.push_back(track);
    if (in_use_count_ > 0U) {
      --in_use_count_;
    }
  }

  std::size_t Capacity() const override { return storage_.size(); }

  std::size_t InUseCount() const override { return in_use_count_; }

  std::size_t acquire_calls() const { return acquire_calls_; }

  std::size_t release_calls() const { return release_calls_; }

 private:
  std::vector<signal::tracking::TrackState> storage_;
  std::vector<signal::tracking::TrackState*> free_list_;
  std::size_t in_use_count_{0};
  std::size_t acquire_calls_{0};
  std::size_t release_calls_{0};
};

signal::tracking::TrackMeasurement MakeCartesianMeasurement(std::uint64_t association_key,
                                                            float position_x, float velocity_x,
                                                            bool matched_existing_track = false,
                                                            std::uint64_t external_target_id = 0U) {
  signal::tracking::TrackMeasurement measurement;
  measurement.raw_measurement.association_key = association_key;
  measurement.raw_measurement.matched_existing_track = matched_existing_track;
  measurement.raw_measurement.external_target_id = external_target_id;

  measurement.raw_measurement.position = Eigen::Vector3f(position_x, 0.0f, 0.0f);
  measurement.raw_measurement.measurement_covariance = Eigen::Matrix3f::Identity();
  measurement.filtered_feature.velocity = Eigen::Vector3f(velocity_x, 0.0f, 0.0f);
  return measurement;
}

signal::tracking::CycleContext MakeCycle(std::uint32_t cycle_index, std::uint32_t batch_id,
                                         float dt_sec = 1.0f) {
  signal::tracking::CycleContext cycle;
  cycle.cycle_index = cycle_index;
  cycle.batch_id = batch_id;
  cycle.dt_sec = dt_sec;
  return cycle;
}

}  // namespace

TEST(TrackLifecycleManagerTest, ConfirmsTrackAfterConfiguredHits) {
  signal::tracking::BoostTrackPool pool(4, 16);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 2;
  config.max_miss_before_lost = 1;
  config.max_lost_cycles = 2;

  signal::tracking::TrackLifecycleManager manager(pool, config);

  signal::tracking::TrackMeasurement measurement;
  measurement.raw_measurement.association_key = 7;
  measurement.filtered_feature.velocity = Eigen::Vector3f(3.0f, 4.0f, 0.0f);
  measurement.filtered_feature.rcs = 1.5f;

  manager.Update(MakeCycle(1u, 1001u), {measurement});

  auto active_tracks = manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  EXPECT_EQ(active_tracks[0]->status, signal::tracking::TrackStatus::kTentative);

  manager.Update(MakeCycle(2u, 1002u), {measurement});

  active_tracks = manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  EXPECT_EQ(active_tracks[0]->status, signal::tracking::TrackStatus::kConfirmed);

  const session::ArSceneTargetList snapshot = manager.BuildSceneTargetSnapshot();
  ASSERT_EQ(snapshot.size(), 1u);
  EXPECT_FLOAT_EQ(SpeedOf(snapshot[0]), 5.0f);
  EXPECT_FLOAT_EQ(snapshot[0].rcs, 1.5f);
}

TEST(TrackLifecycleManagerTest, RecyclesTrackAfterLostTimeout) {
  signal::tracking::BoostTrackPool pool(2, 8);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 1;
  config.max_miss_before_lost = 0;
  config.max_lost_cycles = 1;

  signal::tracking::TrackLifecycleManager manager(pool, config);

  signal::tracking::TrackMeasurement measurement;
  measurement.raw_measurement.association_key = 3;
  measurement.filtered_feature.velocity = Eigen::Vector3f(1.0f, 0.0f, 0.0f);

  manager.Update(MakeCycle(1u, 2001u), {measurement});

  manager.Update(MakeCycle(2u, 2002u), {});

  auto active_tracks = manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  EXPECT_EQ(active_tracks[0]->status, signal::tracking::TrackStatus::kLost);

  manager.Update(MakeCycle(3u, 2003u), {});

  active_tracks = manager.GetActiveTracks();
  EXPECT_TRUE(active_tracks.empty());

  const session::ArSceneTargetList snapshot = manager.BuildSceneTargetSnapshot();
  EXPECT_TRUE(snapshot.empty());
}

TEST(TrackLifecycleManagerTest, TrackStateSnapshotsPublishSingleBestKnownExternalTarget) {
  signal::tracking::BoostTrackPool pool(4, 16);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 1;
  config.max_miss_before_lost = 1;
  config.max_lost_cycles = 3;

  signal::tracking::TrackLifecycleManager manager(pool, config);

  manager.Update(MakeCycle(1u, 2201u), {MakeCartesianMeasurement(21u, 100.0f, 0.0f, false, 7001u),
                                        MakeCartesianMeasurement(22u, 140.0f, 0.0f, false, 7001u)});

  const session::TrackStateSnapshotList snapshots = manager.BuildTrackStateSnapshots();
  ASSERT_EQ(snapshots.size(), 1U);
  EXPECT_EQ(snapshots[0].external_target_id, 7001u);
  EXPECT_EQ(snapshots[0].association_key, 22u);
  EXPECT_EQ(snapshots[0].status, session::TrackStatus::kConfirmed);
}

TEST(TrackLifecycleManagerTest, LostTrackRehitUsesMeasurementVelocityWithoutSpeedSpike) {
  signal::tracking::BoostTrackPool pool(2, 8);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 1;
  config.max_miss_before_lost = 0;
  config.max_lost_cycles = 3;

  signal::tracking::KalmanPredictorConfig pred_cfg;
  pred_cfg.noise_diff_coeff = 1.0f;
  signal::tracking::KalmanPredictor predictor(pred_cfg);
  signal::tracking::KalmanUpdaterConfig upd_cfg;
  upd_cfg.measurement_noise_std = 1.0f;
  signal::tracking::KalmanUpdater updater(upd_cfg);

  signal::tracking::TrackLifecycleManager manager(pool, config, &predictor, &updater);

  manager.Update(MakeCycle(1u, 2301u), {MakeCartesianMeasurement(31u, 0.0f, 0.0f, false, 8001u)});
  manager.Update(MakeCycle(2u, 2302u), {});

  std::vector<const signal::tracking::TrackState*> active_tracks = manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1U);
  ASSERT_EQ(active_tracks[0]->status, signal::tracking::TrackStatus::kLost);

  manager.Update(MakeCycle(3u, 2303u), {MakeCartesianMeasurement(31u, 300.0f, 0.2f, true, 8001u)});

  const session::TrackStateSnapshotList snapshots = manager.BuildTrackStateSnapshots();
  ASSERT_EQ(snapshots.size(), 1U);
  EXPECT_EQ(snapshots[0].external_target_id, 8001u);
  EXPECT_EQ(snapshots[0].status, session::TrackStatus::kConfirmed);
  EXPECT_LT(snapshots[0].speed, 1.0f);
}

TEST(TrackLifecycleManagerTest, ReusedTrackGetsNewTrackIdAndKeepsGenerationCounter) {
  signal::tracking::BoostTrackPool pool(1, 1);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 1;
  config.max_miss_before_lost = 0;
  config.max_lost_cycles = 1;

  signal::tracking::TrackLifecycleManager manager(pool, config);

  manager.Update(MakeCycle(1u, 2101u), {MakeCartesianMeasurement(11u, 100.0f, 5.0f)});
  std::vector<const signal::tracking::TrackState*> active_tracks = manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  const std::uint64_t first_track_id = active_tracks[0]->track_id;
  EXPECT_EQ(active_tracks[0]->generation, 0u);

  manager.Update(MakeCycle(2u, 2102u), {});
  manager.Update(MakeCycle(3u, 2103u), {});
  EXPECT_TRUE(manager.GetActiveTracks().empty());

  manager.Update(MakeCycle(4u, 2104u), {MakeCartesianMeasurement(12u, 120.0f, 6.0f)});
  active_tracks = manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  EXPECT_NE(active_tracks[0]->track_id, first_track_id);
  EXPECT_EQ(active_tracks[0]->track_id, first_track_id + 1u);
  EXPECT_EQ(active_tracks[0]->generation, 1u);
}

TEST(BoostTrackPoolTest, DoubleReleaseDoesNotReissueSamePointerTwice) {
  signal::tracking::BoostTrackPool pool(1, 8);
  signal::tracking::TrackState* track = pool.Acquire();
  ASSERT_NE(track, nullptr);

  pool.Release(track);
  EXPECT_EQ(pool.InUseCount(), 0u);

  // Second release should be rejected.
  pool.Release(track);
  EXPECT_EQ(pool.InUseCount(), 0u);

  signal::tracking::TrackState* first = pool.Acquire();
  signal::tracking::TrackState* second = pool.Acquire();
  ASSERT_NE(first, nullptr);
  ASSERT_NE(second, nullptr);
  EXPECT_NE(first, second);

  pool.Release(first);
  pool.Release(second);
}

TEST(TrackLifecycleManagerTest, ExtraMissToleranceDelaysLostTransition) {
  signal::tracking::BoostTrackPool pool(2, 8);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 1;
  config.max_miss_before_lost = 0;
  config.max_lost_cycles = 2;

  signal::tracking::TrackLifecycleManager manager(pool, config);

  const signal::tracking::TrackMeasurement measurement =
      MakeCartesianMeasurement(31u, 100.0f, 10.0f, false);

  manager.Update(MakeCycle(1u, 3101u), {measurement});
  manager.Update(MakeCycle(2u, 3102u, 1.0f), {});

  std::vector<const signal::tracking::TrackState*> active_tracks = manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  EXPECT_EQ(active_tracks[0]->status, signal::tracking::TrackStatus::kLost);

  signal::tracking::TrackLifecycleManager protected_manager(pool, config);
  protected_manager.Update(MakeCycle(1u, 3201u), {measurement});
  signal::tracking::CycleContext protected_cycle = MakeCycle(2u, 3202u);
  protected_cycle.extra_miss_tolerance = 1u;
  protected_manager.Update(protected_cycle, {});

  active_tracks = protected_manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  EXPECT_EQ(active_tracks[0]->status, signal::tracking::TrackStatus::kConfirmed);
  EXPECT_EQ(active_tracks[0]->miss_count, 1u);
}

TEST(TrackLifecycleManagerTest, DeceptionSummaryExtendsLocalMissToleranceOnMiss) {
  signal::tracking::BoostTrackPool deception_pool(2, 8);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 1;
  config.max_miss_before_lost = 0;
  config.max_lost_cycles = 2;

  signal::tracking::TrackLifecycleManager deception_manager(deception_pool, config);
  signal::tracking::TrackMeasurement deception_measurement =
      MakeCartesianMeasurement(41u, 100.0f, 10.0f);
  deception_measurement.filtered_feature.jamming_detected = true;
  deception_measurement.filtered_feature.dominant_jamming_semantic =
      config::JammingSemantic::kDeception;
  deception_measurement.filtered_feature.jamming_severity = 0.8f;

  deception_manager.Update(MakeCycle(1u, 4101u), {deception_measurement});
  deception_manager.Update(MakeCycle(2u, 4102u), {});

  std::vector<const signal::tracking::TrackState*> active_tracks =
      deception_manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  EXPECT_EQ(active_tracks[0]->status, signal::tracking::TrackStatus::kConfirmed);

  deception_manager.Update(MakeCycle(3u, 4103u), {});
  active_tracks = deception_manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  EXPECT_EQ(active_tracks[0]->status, signal::tracking::TrackStatus::kLost);

  signal::tracking::BoostTrackPool noise_pool(2, 8);
  signal::tracking::TrackLifecycleManager noise_manager(noise_pool, config);
  signal::tracking::TrackMeasurement noise_measurement =
      MakeCartesianMeasurement(42u, 100.0f, 10.0f);
  noise_measurement.filtered_feature.jamming_detected = true;
  noise_measurement.filtered_feature.dominant_jamming_semantic =
      config::JammingSemantic::kNoiseSuppression;
  noise_measurement.filtered_feature.jamming_severity = 0.8f;

  noise_manager.Update(MakeCycle(1u, 4201u), {noise_measurement});
  noise_manager.Update(MakeCycle(2u, 4202u), {});

  active_tracks = noise_manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  EXPECT_EQ(active_tracks[0]->status, signal::tracking::TrackStatus::kLost);
}

TEST(TrackLifecycleManagerTest, UsesExternalDtForPredictionWhenProvided) {
  signal::tracking::BoostTrackPool pool(2, 8);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 1;
  config.max_miss_before_lost = 5;

  signal::tracking::KalmanPredictorConfig pred_cfg;
  pred_cfg.noise_diff_coeff = 1.0f;
  signal::tracking::KalmanPredictor predictor(pred_cfg);
  signal::tracking::KalmanUpdaterConfig upd_cfg;
  upd_cfg.measurement_noise_std = 1.0f;
  signal::tracking::KalmanUpdater updater(upd_cfg);

  signal::tracking::TrackLifecycleManager manager(pool, config, &predictor, &updater);

  signal::tracking::TrackMeasurement measurement;
  measurement.raw_measurement.association_key = 13u;

  measurement.raw_measurement.position = Eigen::Vector3f(100.0f, 0.0f, 0.0f);
  measurement.raw_measurement.measurement_covariance = Eigen::Matrix3f::Identity();
  measurement.filtered_feature.velocity = Eigen::Vector3f(10.0f, 0.0f, 0.0f);

  signal::tracking::CycleContext cycle_1;
  cycle_1.cycle_index = 1u;
  cycle_1.batch_id = 5001u;
  cycle_1.dt_sec = 1.0f;
  manager.Update(cycle_1, {measurement});

  signal::tracking::CycleContext cycle_2;
  cycle_2.cycle_index = 2u;
  cycle_2.batch_id = 5002u;
  cycle_2.dt_sec = 2.0f;
  manager.Update(cycle_2, {});

  const std::vector<const signal::tracking::TrackState*> active_tracks = manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  EXPECT_NEAR(active_tracks[0]->position(0), 120.0f, 1e-3f);
}

TEST(TrackLifecycleManagerTest, InvalidExternalDtSkipsUpdateWithoutStateChanges) {
  signal::tracking::BoostTrackPool pool(2, 8);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 1;
  config.max_miss_before_lost = 5;

  signal::tracking::KalmanPredictorConfig pred_cfg;
  pred_cfg.noise_diff_coeff = 1.0f;
  signal::tracking::KalmanPredictor predictor(pred_cfg);
  signal::tracking::KalmanUpdaterConfig upd_cfg;
  upd_cfg.measurement_noise_std = 1.0f;
  signal::tracking::KalmanUpdater updater(upd_cfg);

  signal::tracking::TrackLifecycleManager manager(pool, config, &predictor, &updater);

  signal::tracking::TrackMeasurement measurement;
  measurement.raw_measurement.association_key = 14u;

  measurement.raw_measurement.position = Eigen::Vector3f(100.0f, 0.0f, 0.0f);
  measurement.raw_measurement.measurement_covariance = Eigen::Matrix3f::Identity();
  measurement.filtered_feature.velocity = Eigen::Vector3f(10.0f, 0.0f, 0.0f);

  signal::tracking::CycleContext cycle_1;
  cycle_1.cycle_index = 1u;
  cycle_1.batch_id = 6001u;
  cycle_1.dt_sec = 1.0f;
  manager.Update(cycle_1, {measurement});

  signal::tracking::CycleContext cycle_3;
  cycle_3.cycle_index = 3u;
  cycle_3.batch_id = 6003u;
  cycle_3.dt_sec = 0.0f;
  manager.Update(cycle_3, {});

  const std::vector<const signal::tracking::TrackState*> active_tracks = manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  EXPECT_NEAR(active_tracks[0]->position(0), 100.0f, 1e-3f);
  EXPECT_EQ(active_tracks[0]->last_update_cycle, 1u);
}

TEST(TrackLifecycleManagerTest, InvalidExternalDtWithNonIncreasingCycleAlsoSkipsUpdate) {
  signal::tracking::BoostTrackPool pool(2, 8);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 1;
  config.max_miss_before_lost = 5;

  signal::tracking::KalmanPredictorConfig pred_cfg;
  pred_cfg.noise_diff_coeff = 1.0f;
  signal::tracking::KalmanPredictor predictor(pred_cfg);
  signal::tracking::KalmanUpdaterConfig upd_cfg;
  upd_cfg.measurement_noise_std = 1.0f;
  signal::tracking::KalmanUpdater updater(upd_cfg);

  signal::tracking::TrackLifecycleManager manager(pool, config, &predictor, &updater);

  signal::tracking::TrackMeasurement measurement;
  measurement.raw_measurement.association_key = 15u;

  measurement.raw_measurement.position = Eigen::Vector3f(100.0f, 0.0f, 0.0f);
  measurement.raw_measurement.measurement_covariance = Eigen::Matrix3f::Identity();
  measurement.filtered_feature.velocity = Eigen::Vector3f(10.0f, 0.0f, 0.0f);

  signal::tracking::CycleContext cycle_1;
  cycle_1.cycle_index = 1u;
  cycle_1.batch_id = 7001u;
  cycle_1.dt_sec = 1.0f;
  manager.Update(cycle_1, {measurement});

  signal::tracking::CycleContext repeated_cycle;
  repeated_cycle.cycle_index = 1u;
  repeated_cycle.batch_id = 7002u;
  repeated_cycle.dt_sec = 0.0f;
  manager.Update(repeated_cycle, {});

  const std::vector<const signal::tracking::TrackState*> active_tracks = manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  EXPECT_NEAR(active_tracks[0]->position(0), 100.0f, 1e-3f);
  EXPECT_EQ(active_tracks[0]->last_update_cycle, 1u);
}

TEST(TrackLifecycleManagerTest, ImmPathPredictsConfirmedTrackAcrossMissedCycles) {
  signal::tracking::BoostTrackPool pool(4, 16);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 1;
  config.max_miss_before_lost = 1;
  config.max_lost_cycles = 3;
  config.imm_activation_policy = signal::tracking::ImmActivationPolicy::kAllTracks;

  signal::tracking::KalmanPredictorConfig pred_cfg_1;
  pred_cfg_1.noise_diff_coeff = 0.5f;
  signal::tracking::KalmanPredictor pred_1(pred_cfg_1);

  signal::tracking::KalmanPredictorConfig pred_cfg_2;
  pred_cfg_2.noise_diff_coeff = 15.0f;
  signal::tracking::KalmanPredictor pred_2(pred_cfg_2);

  signal::tracking::KalmanUpdaterConfig upd_cfg;
  upd_cfg.measurement_noise_std = 1.0f;
  signal::tracking::KalmanUpdater upd_1(upd_cfg);
  signal::tracking::KalmanUpdater upd_2(upd_cfg);

  Eigen::MatrixXf transition_probability(2, 2);
  transition_probability << 0.95f, 0.05f, 0.05f, 0.95f;
  Eigen::VectorXf initial_weights(2);
  initial_weights << 0.5f, 0.5f;

  signal::tracking::TrackLifecycleManager manager(
      pool, config, {&pred_1, &pred_2}, {&upd_1, &upd_2}, transition_probability, initial_weights);

  signal::tracking::TrackMeasurement measurement;
  measurement.raw_measurement.association_key = 42;
  measurement.raw_measurement.matched_existing_track = false;

  measurement.raw_measurement.position = Eigen::Vector3f(100.0f, 0.0f, 0.0f);
  measurement.filtered_feature.velocity = Eigen::Vector3f(10.0f, 0.0f, 0.0f);
  measurement.raw_measurement.measurement_covariance = Eigen::Matrix3f::Identity();

  manager.Update(MakeCycle(1u, 3001u), {measurement});

  auto active_tracks = manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  EXPECT_EQ(active_tracks[0]->status, signal::tracking::TrackStatus::kConfirmed);
  EXPECT_FLOAT_EQ(active_tracks[0]->position(0), 100.0f);

  measurement.raw_measurement.matched_existing_track = true;
  measurement.raw_measurement.position = Eigen::Vector3f(110.0f, 0.0f, 0.0f);

  manager.Update(MakeCycle(2u, 3002u), {measurement});

  active_tracks = manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  EXPECT_NEAR(active_tracks[0]->position(0), 110.0f, 2.0f);
  const float covariance_before_miss = active_tracks[0]->gaussian_state.covariance(0, 0);

  manager.Update(MakeCycle(3u, 3003u), {});

  active_tracks = manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  EXPECT_GE(active_tracks[0]->position(0), 109.0f);
  EXPECT_GT(active_tracks[0]->gaussian_state.covariance(0, 0), covariance_before_miss);

  const session::ArSceneTargetList snapshot = manager.BuildSceneTargetSnapshot();
  ASSERT_EQ(snapshot.size(), 1u);
  EXPECT_FLOAT_EQ(snapshot[0].position_x, active_tracks[0]->position(0));
}

TEST(TrackLifecycleManagerTest, ConfirmedOnlyImmFallsBackToSingleModelBeforeImmCreation) {
  signal::tracking::BoostTrackPool pool(4, 16);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 1;
  config.max_miss_before_lost = 1;
  config.max_lost_cycles = 3;
  config.nominal_cycle_dt_sec = 1.0f;  // 每周期 1 秒，与 Predict(expected_initial, 1.0f) 对齐

  signal::tracking::KalmanPredictorConfig pred_cfg_1;
  pred_cfg_1.noise_diff_coeff = 0.5f;
  signal::tracking::KalmanPredictor pred_1(pred_cfg_1);

  signal::tracking::KalmanPredictorConfig pred_cfg_2;
  pred_cfg_2.noise_diff_coeff = 15.0f;
  signal::tracking::KalmanPredictor pred_2(pred_cfg_2);

  signal::tracking::KalmanUpdaterConfig upd_cfg;
  upd_cfg.measurement_noise_std = 1.0f;
  signal::tracking::KalmanUpdater upd_1(upd_cfg);
  signal::tracking::KalmanUpdater upd_2(upd_cfg);

  Eigen::MatrixXf transition_probability(2, 2);
  transition_probability << 0.95f, 0.05f, 0.05f, 0.95f;
  Eigen::VectorXf initial_weights(2);
  initial_weights << 0.5f, 0.5f;

  signal::tracking::TrackLifecycleManager manager(
      pool, config, {&pred_1, &pred_2}, {&upd_1, &upd_2}, transition_probability, initial_weights);

  signal::tracking::TrackMeasurement measurement;
  measurement.raw_measurement.association_key = 52u;
  measurement.raw_measurement.matched_existing_track = false;

  measurement.raw_measurement.position = Eigen::Vector3f(100.0f, 0.0f, 0.0f);
  measurement.filtered_feature.velocity = Eigen::Vector3f(10.0f, 0.0f, 0.0f);
  measurement.raw_measurement.measurement_covariance = Eigen::Matrix3f::Identity();

  manager.Update(MakeCycle(1u, 5201u), {measurement});

  manager.Update(MakeCycle(2u, 5202u), {});

  const std::vector<const signal::tracking::TrackState*> active_tracks = manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);

  signal::tracking::GaussianTrackState expected_initial;
  expected_initial.mean << 100.0f, 10.0f, 0.0f, 0.0f, 0.0f, 0.0f;
  expected_initial.covariance = signal::tracking::StateCovariance::Identity() * 100.0f;
  const signal::tracking::GaussianTrackState expected_predicted =
      pred_1.Predict(expected_initial, 1.0f);

  EXPECT_NEAR(active_tracks[0]->position(0), 110.0f, 1e-3f);
  EXPECT_NEAR(active_tracks[0]->gaussian_state.mean(0), expected_predicted.mean(0), 1e-3f);
  EXPECT_NEAR(active_tracks[0]->gaussian_state.covariance(0, 0),
              expected_predicted.covariance(0, 0), 1e-3f);
}

TEST(TrackLifecycleManagerTest, ConfirmedOnlyImmCreatesAndUsesFilterOnFirstConfirmedRehit) {
  signal::tracking::BoostTrackPool pool_confirmed(4, 16);
  signal::tracking::BoostTrackPool pool_all_tracks(4, 16);

  signal::tracking::LifecycleConfig confirmed_only_config;
  confirmed_only_config.confirm_hits = 1;
  confirmed_only_config.max_miss_before_lost = 1;
  confirmed_only_config.max_lost_cycles = 3;

  signal::tracking::LifecycleConfig all_tracks_config = confirmed_only_config;
  all_tracks_config.imm_activation_policy = signal::tracking::ImmActivationPolicy::kAllTracks;

  signal::tracking::KalmanPredictorConfig pred_cfg_1;
  pred_cfg_1.noise_diff_coeff = 0.5f;
  signal::tracking::KalmanPredictor pred_1(pred_cfg_1);

  signal::tracking::KalmanPredictorConfig pred_cfg_2;
  pred_cfg_2.noise_diff_coeff = 50.0f;
  signal::tracking::KalmanPredictor pred_2(pred_cfg_2);

  signal::tracking::KalmanUpdaterConfig upd_cfg;
  upd_cfg.measurement_noise_std = 1.0f;
  signal::tracking::KalmanUpdater upd_1(upd_cfg);
  signal::tracking::KalmanUpdater upd_2(upd_cfg);

  Eigen::MatrixXf transition_probability(2, 2);
  transition_probability << 0.95f, 0.05f, 0.05f, 0.95f;
  Eigen::VectorXf initial_weights(2);
  initial_weights << 0.5f, 0.5f;

  signal::tracking::TrackLifecycleManager confirmed_only_manager(
      pool_confirmed, confirmed_only_config, {&pred_1, &pred_2}, {&upd_1, &upd_2},
      transition_probability, initial_weights);
  signal::tracking::TrackLifecycleManager all_tracks_manager(
      pool_all_tracks, all_tracks_config, {&pred_1, &pred_2}, {&upd_1, &upd_2},
      transition_probability, initial_weights);

  signal::tracking::TrackMeasurement first;
  first.raw_measurement.association_key = 62u;
  first.raw_measurement.matched_existing_track = false;

  first.raw_measurement.position = Eigen::Vector3f(100.0f, 0.0f, 0.0f);
  first.filtered_feature.velocity = Eigen::Vector3f(10.0f, 0.0f, 0.0f);
  first.raw_measurement.measurement_covariance = Eigen::Matrix3f::Identity();

  const signal::tracking::CycleContext cycle_1 = MakeCycle(1u, 6201u);
  confirmed_only_manager.Update(cycle_1, {first});
  all_tracks_manager.Update(cycle_1, {first});

  signal::tracking::TrackMeasurement second = first;
  second.raw_measurement.matched_existing_track = true;
  second.raw_measurement.position = Eigen::Vector3f(150.0f, 0.0f, 0.0f);

  const signal::tracking::CycleContext cycle_2 = MakeCycle(2u, 6202u);
  confirmed_only_manager.Update(cycle_2, {second});
  all_tracks_manager.Update(cycle_2, {second});

  const std::vector<const signal::tracking::TrackState*> confirmed_only_tracks =
      confirmed_only_manager.GetActiveTracks();
  const std::vector<const signal::tracking::TrackState*> all_tracks =
      all_tracks_manager.GetActiveTracks();
  ASSERT_EQ(confirmed_only_tracks.size(), 1u);
  ASSERT_EQ(all_tracks.size(), 1u);

  EXPECT_NEAR(confirmed_only_tracks[0]->gaussian_state.mean(0),
              all_tracks[0]->gaussian_state.mean(0), 1e-3f);
  EXPECT_NEAR(confirmed_only_tracks[0]->gaussian_state.covariance(0, 0),
              all_tracks[0]->gaussian_state.covariance(0, 0), 1e-3f);
}

TEST(TrackLifecycleManagerTest, GlobalLockTrackPoolModePreservesLifecycleResults) {
  CountingTrackPool single_thread_pool(8);
  CountingTrackPool locked_pool(8);

  signal::tracking::LifecycleConfig single_thread_config;
  single_thread_config.confirm_hits = 1;
  single_thread_config.max_miss_before_lost = 1;
  single_thread_config.max_lost_cycles = 3;

  signal::tracking::LifecycleConfig locked_config = single_thread_config;
  locked_config.track_pool_thread_safety_mode =
      signal::tracking::TrackPoolThreadSafetyMode::kMultiThreadGlobalLock;

  signal::tracking::KalmanPredictorConfig pred_cfg;
  pred_cfg.noise_diff_coeff = 1.0f;
  signal::tracking::KalmanPredictor predictor(pred_cfg);

  signal::tracking::KalmanUpdaterConfig upd_cfg;
  upd_cfg.measurement_noise_std = 1.0f;
  signal::tracking::KalmanUpdater updater(upd_cfg);

  signal::tracking::TrackLifecycleManager single_thread_manager(
      single_thread_pool, single_thread_config, &predictor, &updater);
  signal::tracking::SynchronizedTrackPool wrapped_locked_pool(locked_pool);
  signal::tracking::TrackLifecycleManager locked_manager(wrapped_locked_pool, locked_config,
                                                         &predictor, &updater);

  const signal::tracking::TrackMeasurement first =
      MakeCartesianMeasurement(71u, 100.0f, 10.0f, false);
  const signal::tracking::TrackMeasurement second =
      MakeCartesianMeasurement(71u, 111.0f, 10.0f, true);

  single_thread_manager.Update(MakeCycle(1u, 7101u), {first});
  locked_manager.Update(MakeCycle(1u, 7101u), {first});
  single_thread_manager.Update(MakeCycle(2u, 7102u), {second});
  locked_manager.Update(MakeCycle(2u, 7102u), {second});
  single_thread_manager.Update(MakeCycle(3u, 7103u), {});
  locked_manager.Update(MakeCycle(3u, 7103u), {});

  const std::vector<const signal::tracking::TrackState*> single_thread_tracks =
      single_thread_manager.GetActiveTracks();
  const std::vector<const signal::tracking::TrackState*> locked_tracks =
      locked_manager.GetActiveTracks();
  ASSERT_EQ(single_thread_tracks.size(), 1u);
  ASSERT_EQ(locked_tracks.size(), 1u);

  EXPECT_EQ(single_thread_tracks[0]->status, locked_tracks[0]->status);
  EXPECT_NEAR(single_thread_tracks[0]->position(0), locked_tracks[0]->position(0), 1e-4f);
  EXPECT_NEAR(single_thread_tracks[0]->velocity(0), locked_tracks[0]->velocity(0), 1e-4f);
  EXPECT_NEAR(single_thread_tracks[0]->gaussian_state.mean(0),
              locked_tracks[0]->gaussian_state.mean(0), 1e-4f);
  EXPECT_NEAR(single_thread_tracks[0]->gaussian_state.covariance(0, 0),
              locked_tracks[0]->gaussian_state.covariance(0, 0), 1e-4f);
  EXPECT_EQ(single_thread_pool.acquire_calls(), locked_pool.acquire_calls());
  EXPECT_EQ(single_thread_pool.release_calls(), locked_pool.release_calls());
}

TEST(TrackLifecycleManagerTest, MixedLifecycleCycleOnlyAcquiresNewTracksAndRecyclesInRecyclePhase) {
  CountingTrackPool pool(8);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 1;
  config.max_miss_before_lost = 0;
  config.max_lost_cycles = 0;

  signal::tracking::TrackLifecycleManager manager(pool, config);

  const signal::tracking::TrackMeasurement first_track =
      MakeCartesianMeasurement(81u, 10.0f, 1.0f, false);
  const signal::tracking::TrackMeasurement second_track =
      MakeCartesianMeasurement(82u, 20.0f, 2.0f, false);

  manager.Update(MakeCycle(1u, 8101u), {first_track, second_track});
  EXPECT_EQ(pool.acquire_calls(), 2u);
  EXPECT_EQ(pool.release_calls(), 0u);
  EXPECT_EQ(pool.InUseCount(), 2u);

  const signal::tracking::TrackMeasurement first_track_rehit =
      MakeCartesianMeasurement(81u, 11.0f, 1.0f, true);
  const signal::tracking::TrackMeasurement new_track =
      MakeCartesianMeasurement(83u, 30.0f, 3.0f, false);
  manager.Update(MakeCycle(2u, 8102u), {first_track_rehit, new_track});

  EXPECT_EQ(pool.acquire_calls(), 3u);
  EXPECT_EQ(pool.release_calls(), 0u);
  EXPECT_EQ(pool.InUseCount(), 3u);

  const std::vector<const signal::tracking::TrackState*> after_mixed_cycle =
      manager.GetActiveTracks();
  ASSERT_EQ(after_mixed_cycle.size(), 3u);

  manager.Update(MakeCycle(3u, 8103u), {});

  EXPECT_EQ(pool.acquire_calls(), 3u);
  EXPECT_EQ(pool.release_calls(), 1u);
  EXPECT_EQ(pool.InUseCount(), 2u);

  const std::vector<const signal::tracking::TrackState*> after_recycle_cycle =
      manager.GetActiveTracks();
  EXPECT_EQ(after_recycle_cycle.size(), 2u);
}

TEST(TrackLifecycleManagerTest, GlobalLockTrackPoolModePreservesAcquireReleaseCounts) {
  CountingTrackPool pool(8);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 1;
  config.max_miss_before_lost = 0;
  config.max_lost_cycles = 0;
  config.track_pool_thread_safety_mode =
      signal::tracking::TrackPoolThreadSafetyMode::kMultiThreadGlobalLock;

  signal::tracking::SynchronizedTrackPool wrapped_pool(pool);
  signal::tracking::TrackLifecycleManager manager(wrapped_pool, config);

  manager.Update(MakeCycle(1u, 8201u), {MakeCartesianMeasurement(91u, 10.0f, 1.0f, false),
                                        MakeCartesianMeasurement(92u, 20.0f, 2.0f, false)});
  EXPECT_EQ(pool.acquire_calls(), 2u);
  EXPECT_EQ(pool.release_calls(), 0u);

  manager.Update(MakeCycle(2u, 8202u), {MakeCartesianMeasurement(91u, 11.0f, 1.0f, true),
                                        MakeCartesianMeasurement(93u, 30.0f, 3.0f, false)});
  EXPECT_EQ(pool.acquire_calls(), 3u);
  EXPECT_EQ(pool.release_calls(), 0u);

  manager.Update(MakeCycle(3u, 8203u), {});
  EXPECT_EQ(pool.acquire_calls(), 3u);
  EXPECT_EQ(pool.release_calls(), 1u);
  EXPECT_EQ(pool.InUseCount(), 2u);
}

TEST(TrackLifecycleManagerTest, BuildAssociationSeedsExportsPositionAndGaussianState) {
  signal::tracking::BoostTrackPool pool(4, 16);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 1;

  signal::tracking::KalmanPredictorConfig pred_cfg;
  pred_cfg.noise_diff_coeff = 1.0f;
  signal::tracking::KalmanPredictor predictor(pred_cfg);

  signal::tracking::KalmanUpdaterConfig upd_cfg;
  upd_cfg.measurement_noise_std = 1.0f;
  signal::tracking::KalmanUpdater updater(upd_cfg);

  signal::tracking::TrackLifecycleManager manager(pool, config, &predictor, &updater);

  signal::tracking::TrackMeasurement measurement;
  measurement.raw_measurement.association_key = 99u;
  measurement.raw_measurement.matched_existing_track = false;

  measurement.raw_measurement.position = Eigen::Vector3f(30.0f, 2.0f, -1.0f);
  measurement.filtered_feature.velocity = Eigen::Vector3f(5.0f, 0.0f, 0.0f);
  measurement.raw_measurement.measurement_covariance = Eigen::Matrix3f::Identity();

  manager.Update(MakeCycle(1u, 4001u), {measurement});

  const std::vector<signal::tracking::AssociationTrackSeed> seeds = manager.BuildAssociationSeeds();
  ASSERT_EQ(seeds.size(), 1u);
  EXPECT_EQ(seeds[0].association_key, 99u);
  EXPECT_TRUE(seeds[0].has_position);
  EXPECT_EQ(seeds[0].position, measurement.raw_measurement.position);
  EXPECT_TRUE(seeds[0].has_gaussian_state);
  EXPECT_FLOAT_EQ(seeds[0].gaussian_state.mean(0), measurement.raw_measurement.position(0));
  EXPECT_FLOAT_EQ(seeds[0].gaussian_state.mean(1), measurement.filtered_feature.velocity(0));
  EXPECT_FLOAT_EQ(seeds[0].gaussian_state.mean(2), measurement.raw_measurement.position(1));
  EXPECT_FLOAT_EQ(seeds[0].gaussian_state.mean(4), measurement.raw_measurement.position(2));
}

TEST(TrackLifecycleManagerTest, FilterWritebackUpdatesAccelerationFromVelocityDelta) {
  signal::tracking::BoostTrackPool pool(4, 16);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 1;

  signal::tracking::KalmanPredictorConfig pred_cfg;
  pred_cfg.noise_diff_coeff = 1.0f;
  signal::tracking::KalmanPredictor predictor(pred_cfg);

  signal::tracking::KalmanUpdaterConfig upd_cfg;
  upd_cfg.measurement_noise_std = 0.5f;
  signal::tracking::KalmanUpdater updater(upd_cfg);

  signal::tracking::TrackLifecycleManager manager(pool, config, &predictor, &updater);

  signal::tracking::TrackMeasurement first;
  first.raw_measurement.association_key = 501u;
  first.raw_measurement.matched_existing_track = false;

  first.raw_measurement.position = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  first.filtered_feature.velocity = Eigen::Vector3f(1.0f, 2.0f, 0.0f);
  first.raw_measurement.measurement_covariance = Eigen::Matrix3f::Identity();

  manager.Update(MakeCycle(1u, 9001u), {first});

  signal::tracking::TrackMeasurement second = first;
  second.raw_measurement.matched_existing_track = true;
  second.raw_measurement.position = Eigen::Vector3f(1.0f, 2.0f, 0.0f);
  second.filtered_feature.velocity = Eigen::Vector3f(3.0f, 5.0f, 0.0f);

  manager.Update(MakeCycle(2u, 9002u), {second});

  const std::vector<const signal::tracking::TrackState*> active = manager.GetActiveTracks();
  ASSERT_EQ(active.size(), 1u);
  EXPECT_GT(active[0]->acceleration.norm(), 0.0f);

  const session::ArSceneTargetList snapshot = manager.BuildSceneTargetSnapshot();
  ASSERT_EQ(snapshot.size(), 1u);
  EXPECT_NEAR(SpeedOf(snapshot[0]),
              std::sqrt(snapshot[0].velocity_x * snapshot[0].velocity_x +
                        snapshot[0].velocity_y * snapshot[0].velocity_y +
                        snapshot[0].velocity_z * snapshot[0].velocity_z),
              1e-4f);
}

// ===========================================================================
// RuntimeState 捕获/恢复 + SyncRuntimeTuning（此前 50% 分支覆盖）
// ===========================================================================

TEST(TrackLifecycleManagerTest, CaptureAndRestoreRoundTripsState) {
  signal::tracking::BoostTrackPool pool(4, 16);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 2;
  config.max_miss_before_lost = 1;

  signal::tracking::TrackLifecycleManager manager(pool, config);

  signal::tracking::TrackMeasurement measurement;
  measurement.raw_measurement.association_key = 42;
  measurement.filtered_feature.velocity = Eigen::Vector3f(3.0f, 4.0f, 0.0f);

  manager.Update(MakeCycle(1u, 100u), {measurement});
  manager.Update(MakeCycle(2u, 101u), {measurement});
  ASSERT_EQ(manager.GetActiveTracks().size(), 1u);

  // 捕获当前状态（已确认轨迹）
  const signal::tracking::TrackLifecycleRuntimeState state = manager.CaptureRuntimeState();
  EXPECT_NE(state.owner_identity, nullptr);

  // 恢复到同一状态：轨迹仍应存在
  manager.RestoreRuntimeState(state);
  auto tracks = manager.GetActiveTracks();
  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_EQ(tracks[0]->status, signal::tracking::TrackStatus::kConfirmed);
}

TEST(TrackLifecycleManagerTest, RestoreRejectsMismatchedSchemaVersion) {
  signal::tracking::BoostTrackPool pool(4, 16);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 1;

  signal::tracking::TrackLifecycleManager manager(pool, config);

  signal::tracking::TrackMeasurement measurement;
  measurement.raw_measurement.association_key = 1;
  manager.Update(MakeCycle(1u, 1u), {measurement});
  ASSERT_EQ(manager.GetActiveTracks().size(), 1u);

  // schema_version != 1 → 拒绝恢复，轨迹保留不变
  signal::tracking::TrackLifecycleRuntimeState bad_state;
  bad_state.schema_version = 0U;
  manager.RestoreRuntimeState(bad_state);
  EXPECT_EQ(manager.GetActiveTracks().size(), 1u);
}

TEST(TrackLifecycleManagerTest, SyncRuntimeTuningUpdatesConfigWithoutCrash) {
  signal::tracking::BoostTrackPool pool(4, 16);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 2;
  config.max_miss_before_lost = 1;

  signal::tracking::TrackLifecycleManager manager(pool, config);

  signal::tracking::LifecycleConfig new_config = config;
  new_config.confirm_hits = 3;
  new_config.max_miss_before_lost = 2;

  manager.SyncRuntimeTuning(new_config, 1.5f, 0.5f, {0.1f}, Eigen::MatrixXf::Identity(1, 1),
                            Eigen::VectorXf::Ones(1));
  SUCCEED();
}

}  // namespace tests
}  // namespace airborne_radar
