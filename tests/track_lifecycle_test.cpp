// Copyright 2026. All Rights Reserved.
//
// @file track_lifecycle_test.cpp
// @brief 验证轨迹对象池与生命周期管理的基础状态机行为。

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "airborne_radar/common/TrackTypes.h"
#include "airborne_radar/signal/tracking/TrackLifecycleManager.h"
#include "airborne_radar/signal/tracking/BoostTrackPool.h"
#include "airborne_radar/signal/tracking/SynchronizedTrackPool.h"
#include "airborne_radar/signal/tracking/KalmanPredictor.h"
#include "airborne_radar/signal/tracking/KalmanUpdater.h"

namespace airborne_radar { namespace tests {

namespace {

class CountingTrackPool : public signal::tracking::ITrackPool {
public:
  explicit CountingTrackPool(std::size_t capacity)
      : storage_(capacity) {
    free_list_.reserve(storage_.size());
    for (std::vector<common::TrackState>::iterator it = storage_.begin();
         it != storage_.end(); ++it) {
      free_list_.push_back(&(*it));
    }
  }

  common::TrackState *Acquire() override {
    ++acquire_calls_;
    if (free_list_.empty()) {
      return nullptr;
    }

    common::TrackState *track = free_list_.back();
    free_list_.pop_back();
    ++in_use_count_;
    return track;
  }

  void Release(common::TrackState *track) override {
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
  std::vector<common::TrackState> storage_;
  std::vector<common::TrackState *> free_list_;
  std::size_t in_use_count_{0};
  std::size_t acquire_calls_{0};
  std::size_t release_calls_{0};
};

signal::tracking::TrackMeasurement MakeCartesianMeasurement(
    std::uint64_t association_key, float position_x, float velocity_x,
    bool matched_existing_track = false) {
  signal::tracking::TrackMeasurement measurement;
  measurement.raw_measurement.association_key = association_key;
  measurement.raw_measurement.matched_existing_track = matched_existing_track;
  measurement.raw_measurement.has_cartesian_position = true;
  measurement.raw_measurement.position =
      Eigen::Vector3f(position_x, 0.0f, 0.0f);
  measurement.raw_measurement.measurement_covariance =
      Eigen::Matrix3f::Identity();
  measurement.filtered_feature.velocity =
      Eigen::Vector3f(velocity_x, 0.0f, 0.0f);
  return measurement;
}

signal::tracking::CycleContext MakeCycle(std::uint32_t cycle_index,
                                         std::uint32_t batch_id,
                                         float dt_sec = 1.0f) {
  signal::tracking::CycleContext cycle;
  cycle.cycle_index = cycle_index;
  cycle.batch_id = batch_id;
  cycle.dt_sec = dt_sec;
  return cycle;
}

} // namespace

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

  signal::tracking::CycleContext cycle_1;
  cycle_1.cycle_index = 1;
  cycle_1.batch_id = 1001;
  manager.Update(cycle_1, {measurement});

  auto active_tracks = manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  EXPECT_EQ(active_tracks[0]->status, common::TrackStatus::kTentative);

  signal::tracking::CycleContext cycle_2;
  cycle_2.cycle_index = 2;
  cycle_2.batch_id = 1002;
  manager.Update(cycle_2, {measurement});

  active_tracks = manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  EXPECT_EQ(active_tracks[0]->status, common::TrackStatus::kConfirmed);

  const common::TargetFeatureList snapshot = manager.BuildFeatureSnapshot();
  ASSERT_EQ(snapshot.size(), 1u);
  EXPECT_FLOAT_EQ(snapshot[0].current_track_speed, 5.0f);
  EXPECT_FLOAT_EQ(snapshot[0].current_track_rcs, 1.5f);
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

  signal::tracking::CycleContext cycle_1;
  cycle_1.cycle_index = 1;
  cycle_1.batch_id = 2001;
  manager.Update(cycle_1, {measurement});

  signal::tracking::CycleContext cycle_2;
  cycle_2.cycle_index = 2;
  cycle_2.batch_id = 2002;
  manager.Update(cycle_2, {});

  auto active_tracks = manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  EXPECT_EQ(active_tracks[0]->status, common::TrackStatus::kLost);

  signal::tracking::CycleContext cycle_3;
  cycle_3.cycle_index = 3;
  cycle_3.batch_id = 2003;
  manager.Update(cycle_3, {});

  active_tracks = manager.GetActiveTracks();
  EXPECT_TRUE(active_tracks.empty());

  const common::TargetFeatureList snapshot = manager.BuildFeatureSnapshot();
  EXPECT_TRUE(snapshot.empty());
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

  std::vector<const common::TrackState *> active_tracks = manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  EXPECT_EQ(active_tracks[0]->status, common::TrackStatus::kLost);

  signal::tracking::TrackLifecycleManager protected_manager(pool, config);
  protected_manager.Update(MakeCycle(1u, 3201u), {measurement});
  signal::tracking::CycleContext protected_cycle = MakeCycle(2u, 3202u);
  protected_cycle.extra_miss_tolerance = 1u;
  protected_manager.Update(protected_cycle, {});

  active_tracks = protected_manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  EXPECT_EQ(active_tracks[0]->status, common::TrackStatus::kConfirmed);
  EXPECT_EQ(active_tracks[0]->miss_count, 1u);
}

TEST(TrackLifecycleManagerTest,
     DeceptionSummaryExtendsLocalMissToleranceOnMiss) {
  signal::tracking::BoostTrackPool deception_pool(2, 8);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 1;
  config.max_miss_before_lost = 0;
  config.max_lost_cycles = 2;

  signal::tracking::TrackLifecycleManager deception_manager(deception_pool,
                                                            config);
  signal::tracking::TrackMeasurement deception_measurement =
      MakeCartesianMeasurement(41u, 100.0f, 10.0f);
  deception_measurement.filtered_feature.jamming_detected = true;
  deception_measurement.filtered_feature.dominant_jamming_semantic =
      common::JammingSemantic::kDeception;
  deception_measurement.filtered_feature.jamming_severity = 0.8f;

  deception_manager.Update(MakeCycle(1u, 4101u), {deception_measurement});
  deception_manager.Update(MakeCycle(2u, 4102u), {});

  std::vector<const common::TrackState *> active_tracks =
      deception_manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  EXPECT_EQ(active_tracks[0]->status, common::TrackStatus::kConfirmed);

  deception_manager.Update(MakeCycle(3u, 4103u), {});
  active_tracks = deception_manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  EXPECT_EQ(active_tracks[0]->status, common::TrackStatus::kLost);

  signal::tracking::BoostTrackPool noise_pool(2, 8);
  signal::tracking::TrackLifecycleManager noise_manager(noise_pool, config);
  signal::tracking::TrackMeasurement noise_measurement =
      MakeCartesianMeasurement(42u, 100.0f, 10.0f);
  noise_measurement.filtered_feature.jamming_detected = true;
  noise_measurement.filtered_feature.dominant_jamming_semantic =
      common::JammingSemantic::kNoiseSuppression;
  noise_measurement.filtered_feature.jamming_severity = 0.8f;

  noise_manager.Update(MakeCycle(1u, 4201u), {noise_measurement});
  noise_manager.Update(MakeCycle(2u, 4202u), {});

  active_tracks = noise_manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  EXPECT_EQ(active_tracks[0]->status, common::TrackStatus::kLost);
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

  signal::tracking::TrackLifecycleManager manager(pool, config, &predictor,
                                                  &updater);

  signal::tracking::TrackMeasurement measurement;
  measurement.raw_measurement.association_key = 13u;
  measurement.raw_measurement.has_cartesian_position = true;
  measurement.raw_measurement.position = Eigen::Vector3f(100.0f, 0.0f, 0.0f);
  measurement.raw_measurement.measurement_covariance =
      Eigen::Matrix3f::Identity();
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

  const std::vector<const common::TrackState *> active_tracks =
      manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  EXPECT_NEAR(active_tracks[0]->position(0), 120.0f, 1e-3f);
}

TEST(TrackLifecycleManagerTest, FallsBackToCycleDeltaWhenExternalDtIsInvalid) {
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

  signal::tracking::TrackLifecycleManager manager(pool, config, &predictor,
                                                  &updater);

  signal::tracking::TrackMeasurement measurement;
  measurement.raw_measurement.association_key = 14u;
  measurement.raw_measurement.has_cartesian_position = true;
  measurement.raw_measurement.position = Eigen::Vector3f(100.0f, 0.0f, 0.0f);
  measurement.raw_measurement.measurement_covariance =
      Eigen::Matrix3f::Identity();
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

  const std::vector<const common::TrackState *> active_tracks =
      manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  EXPECT_NEAR(active_tracks[0]->position(0), 120.0f, 1e-3f);
}

TEST(TrackLifecycleManagerTest, FallsBackToUnitDtWhenExternalDtInvalidAndCycleNotIncreasing) {
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

  signal::tracking::TrackLifecycleManager manager(pool, config, &predictor,
                                                  &updater);

  signal::tracking::TrackMeasurement measurement;
  measurement.raw_measurement.association_key = 15u;
  measurement.raw_measurement.has_cartesian_position = true;
  measurement.raw_measurement.position = Eigen::Vector3f(100.0f, 0.0f, 0.0f);
  measurement.raw_measurement.measurement_covariance =
      Eigen::Matrix3f::Identity();
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

  const std::vector<const common::TrackState *> active_tracks =
      manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  EXPECT_NEAR(active_tracks[0]->position(0), 110.0f, 1e-3f);
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
  transition_probability << 0.95f, 0.05f,
                            0.05f, 0.95f;
  Eigen::VectorXf initial_weights(2);
  initial_weights << 0.5f, 0.5f;

  signal::tracking::TrackLifecycleManager manager(
      pool, config, {&pred_1, &pred_2}, {&upd_1, &upd_2},
      transition_probability, initial_weights);

  signal::tracking::TrackMeasurement measurement;
  measurement.raw_measurement.association_key = 42;
  measurement.raw_measurement.matched_existing_track = false;
  measurement.raw_measurement.has_cartesian_position = true;
  measurement.raw_measurement.position = Eigen::Vector3f(100.0f, 0.0f, 0.0f);
  measurement.filtered_feature.velocity = Eigen::Vector3f(10.0f, 0.0f, 0.0f);
  measurement.raw_measurement.measurement_covariance =
      Eigen::Matrix3f::Identity();

  signal::tracking::CycleContext cycle_1;
  cycle_1.cycle_index = 1;
  cycle_1.batch_id = 3001;
  manager.Update(cycle_1, {measurement});

  auto active_tracks = manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  EXPECT_EQ(active_tracks[0]->status, common::TrackStatus::kConfirmed);
  EXPECT_FLOAT_EQ(active_tracks[0]->position(0), 100.0f);

  measurement.raw_measurement.matched_existing_track = true;
  measurement.raw_measurement.position =
      Eigen::Vector3f(110.0f, 0.0f, 0.0f);

  signal::tracking::CycleContext cycle_2;
  cycle_2.cycle_index = 2;
  cycle_2.batch_id = 3002;
  manager.Update(cycle_2, {measurement});

  active_tracks = manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  EXPECT_NEAR(active_tracks[0]->position(0), 110.0f, 2.0f);
  const float covariance_before_miss =
      active_tracks[0]->gaussian_state.covariance(0, 0);

  signal::tracking::CycleContext cycle_3;
  cycle_3.cycle_index = 3;
  cycle_3.batch_id = 3003;
  manager.Update(cycle_3, {});

  active_tracks = manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  EXPECT_GE(active_tracks[0]->position(0), 109.0f);
  EXPECT_GT(active_tracks[0]->gaussian_state.covariance(0, 0),
            covariance_before_miss);

  const common::TargetFeatureList snapshot = manager.BuildFeatureSnapshot();
  ASSERT_EQ(snapshot.size(), 1u);
  EXPECT_FLOAT_EQ(snapshot[0].position_x, active_tracks[0]->position(0));
}

TEST(TrackLifecycleManagerTest,
     ConfirmedOnlyImmFallsBackToSingleModelBeforeImmCreation) {
  signal::tracking::BoostTrackPool pool(4, 16);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 1;
  config.max_miss_before_lost = 1;
  config.max_lost_cycles = 3;

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
  transition_probability << 0.95f, 0.05f,
                            0.05f, 0.95f;
  Eigen::VectorXf initial_weights(2);
  initial_weights << 0.5f, 0.5f;

  signal::tracking::TrackLifecycleManager manager(
      pool, config, {&pred_1, &pred_2}, {&upd_1, &upd_2},
      transition_probability, initial_weights);

  signal::tracking::TrackMeasurement measurement;
  measurement.raw_measurement.association_key = 52u;
  measurement.raw_measurement.matched_existing_track = false;
  measurement.raw_measurement.has_cartesian_position = true;
  measurement.raw_measurement.position = Eigen::Vector3f(100.0f, 0.0f, 0.0f);
  measurement.filtered_feature.velocity = Eigen::Vector3f(10.0f, 0.0f, 0.0f);
  measurement.raw_measurement.measurement_covariance =
      Eigen::Matrix3f::Identity();

  signal::tracking::CycleContext cycle_1;
  cycle_1.cycle_index = 1u;
  cycle_1.batch_id = 5201u;
  manager.Update(cycle_1, {measurement});

  signal::tracking::CycleContext cycle_2;
  cycle_2.cycle_index = 2u;
  cycle_2.batch_id = 5202u;
  manager.Update(cycle_2, {});

  const std::vector<const common::TrackState *> active_tracks =
      manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);

  signal::tracking::GaussianTrackState expected_initial;
  expected_initial.mean << 100.0f, 10.0f, 0.0f, 0.0f, 0.0f, 0.0f;
  expected_initial.covariance =
      signal::tracking::StateCovariance::Identity() * 100.0f;
  const signal::tracking::GaussianTrackState expected_predicted =
      pred_1.Predict(expected_initial, 1.0f);

  EXPECT_NEAR(active_tracks[0]->position(0), 110.0f, 1e-3f);
  EXPECT_NEAR(active_tracks[0]->gaussian_state.mean(0),
              expected_predicted.mean(0), 1e-3f);
  EXPECT_NEAR(active_tracks[0]->gaussian_state.covariance(0, 0),
              expected_predicted.covariance(0, 0), 1e-3f);
}

TEST(TrackLifecycleManagerTest,
     ConfirmedOnlyImmCreatesAndUsesFilterOnFirstConfirmedRehit) {
  signal::tracking::BoostTrackPool pool_confirmed(4, 16);
  signal::tracking::BoostTrackPool pool_all_tracks(4, 16);

  signal::tracking::LifecycleConfig confirmed_only_config;
  confirmed_only_config.confirm_hits = 1;
  confirmed_only_config.max_miss_before_lost = 1;
  confirmed_only_config.max_lost_cycles = 3;

  signal::tracking::LifecycleConfig all_tracks_config = confirmed_only_config;
  all_tracks_config.imm_activation_policy =
      signal::tracking::ImmActivationPolicy::kAllTracks;

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
  transition_probability << 0.95f, 0.05f,
                            0.05f, 0.95f;
  Eigen::VectorXf initial_weights(2);
  initial_weights << 0.5f, 0.5f;

  signal::tracking::TrackLifecycleManager confirmed_only_manager(
      pool_confirmed, confirmed_only_config, {&pred_1, &pred_2},
      {&upd_1, &upd_2}, transition_probability, initial_weights);
  signal::tracking::TrackLifecycleManager all_tracks_manager(
      pool_all_tracks, all_tracks_config, {&pred_1, &pred_2},
      {&upd_1, &upd_2}, transition_probability, initial_weights);

  signal::tracking::TrackMeasurement first;
  first.raw_measurement.association_key = 62u;
  first.raw_measurement.matched_existing_track = false;
  first.raw_measurement.has_cartesian_position = true;
  first.raw_measurement.position = Eigen::Vector3f(100.0f, 0.0f, 0.0f);
  first.filtered_feature.velocity = Eigen::Vector3f(10.0f, 0.0f, 0.0f);
  first.raw_measurement.measurement_covariance = Eigen::Matrix3f::Identity();

  signal::tracking::CycleContext cycle_1;
  cycle_1.cycle_index = 1u;
  cycle_1.batch_id = 6201u;
  confirmed_only_manager.Update(cycle_1, {first});
  all_tracks_manager.Update(cycle_1, {first});

  signal::tracking::TrackMeasurement second = first;
  second.raw_measurement.matched_existing_track = true;
  second.raw_measurement.position = Eigen::Vector3f(150.0f, 0.0f, 0.0f);

  signal::tracking::CycleContext cycle_2;
  cycle_2.cycle_index = 2u;
  cycle_2.batch_id = 6202u;
  confirmed_only_manager.Update(cycle_2, {second});
  all_tracks_manager.Update(cycle_2, {second});

  const std::vector<const common::TrackState *> confirmed_only_tracks =
      confirmed_only_manager.GetActiveTracks();
  const std::vector<const common::TrackState *> all_tracks =
      all_tracks_manager.GetActiveTracks();
  ASSERT_EQ(confirmed_only_tracks.size(), 1u);
  ASSERT_EQ(all_tracks.size(), 1u);

  EXPECT_NEAR(confirmed_only_tracks[0]->gaussian_state.mean(0),
              all_tracks[0]->gaussian_state.mean(0), 1e-3f);
  EXPECT_NEAR(confirmed_only_tracks[0]->gaussian_state.covariance(0, 0),
              all_tracks[0]->gaussian_state.covariance(0, 0), 1e-3f);
}

TEST(TrackLifecycleManagerTest,
     GlobalLockTrackPoolModePreservesLifecycleResults) {
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
  signal::tracking::TrackLifecycleManager locked_manager(
      wrapped_locked_pool, locked_config, &predictor, &updater);

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

  const std::vector<const common::TrackState *> single_thread_tracks =
      single_thread_manager.GetActiveTracks();
  const std::vector<const common::TrackState *> locked_tracks =
      locked_manager.GetActiveTracks();
  ASSERT_EQ(single_thread_tracks.size(), 1u);
  ASSERT_EQ(locked_tracks.size(), 1u);

  EXPECT_EQ(single_thread_tracks[0]->status, locked_tracks[0]->status);
  EXPECT_NEAR(single_thread_tracks[0]->position(0),
              locked_tracks[0]->position(0), 1e-4f);
  EXPECT_NEAR(single_thread_tracks[0]->velocity(0),
              locked_tracks[0]->velocity(0), 1e-4f);
  EXPECT_NEAR(single_thread_tracks[0]->gaussian_state.mean(0),
              locked_tracks[0]->gaussian_state.mean(0), 1e-4f);
  EXPECT_NEAR(single_thread_tracks[0]->gaussian_state.covariance(0, 0),
              locked_tracks[0]->gaussian_state.covariance(0, 0), 1e-4f);
  EXPECT_EQ(single_thread_pool.acquire_calls(), locked_pool.acquire_calls());
  EXPECT_EQ(single_thread_pool.release_calls(), locked_pool.release_calls());
}

TEST(TrackLifecycleManagerTest,
     MixedLifecycleCycleOnlyAcquiresNewTracksAndRecyclesInRecyclePhase) {
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

  const std::vector<const common::TrackState *> after_mixed_cycle =
      manager.GetActiveTracks();
  ASSERT_EQ(after_mixed_cycle.size(), 3u);

  manager.Update(MakeCycle(3u, 8103u), {});

  EXPECT_EQ(pool.acquire_calls(), 3u);
  EXPECT_EQ(pool.release_calls(), 1u);
  EXPECT_EQ(pool.InUseCount(), 2u);

  const std::vector<const common::TrackState *> after_recycle_cycle =
      manager.GetActiveTracks();
  EXPECT_EQ(after_recycle_cycle.size(), 2u);
}

TEST(TrackLifecycleManagerTest,
     GlobalLockTrackPoolModePreservesAcquireReleaseCounts) {
  CountingTrackPool pool(8);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 1;
  config.max_miss_before_lost = 0;
  config.max_lost_cycles = 0;
  config.track_pool_thread_safety_mode =
      signal::tracking::TrackPoolThreadSafetyMode::kMultiThreadGlobalLock;

  signal::tracking::SynchronizedTrackPool wrapped_pool(pool);
  signal::tracking::TrackLifecycleManager manager(wrapped_pool, config);

  manager.Update(MakeCycle(1u, 8201u),
                 {MakeCartesianMeasurement(91u, 10.0f, 1.0f, false),
                  MakeCartesianMeasurement(92u, 20.0f, 2.0f, false)});
  EXPECT_EQ(pool.acquire_calls(), 2u);
  EXPECT_EQ(pool.release_calls(), 0u);

  manager.Update(MakeCycle(2u, 8202u),
                 {MakeCartesianMeasurement(91u, 11.0f, 1.0f, true),
                  MakeCartesianMeasurement(93u, 30.0f, 3.0f, false)});
  EXPECT_EQ(pool.acquire_calls(), 3u);
  EXPECT_EQ(pool.release_calls(), 0u);

  manager.Update(MakeCycle(3u, 8203u), {});
  EXPECT_EQ(pool.acquire_calls(), 3u);
  EXPECT_EQ(pool.release_calls(), 1u);
  EXPECT_EQ(pool.InUseCount(), 2u);
}

TEST(TrackLifecycleManagerTest,
     BuildAssociationSeedsExportsPositionAndGaussianState) {
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
  measurement.raw_measurement.has_cartesian_position = true;
  measurement.raw_measurement.position = Eigen::Vector3f(30.0f, 2.0f, -1.0f);
  measurement.filtered_feature.velocity = Eigen::Vector3f(5.0f, 0.0f, 0.0f);
  measurement.raw_measurement.measurement_covariance =
      Eigen::Matrix3f::Identity();

  signal::tracking::CycleContext cycle;
  cycle.cycle_index = 1;
  cycle.batch_id = 4001;
  manager.Update(cycle, {measurement});

  const std::vector<signal::tracking::AssociationTrackSeed> seeds =
      manager.BuildAssociationSeeds();
  ASSERT_EQ(seeds.size(), 1u);
  EXPECT_EQ(seeds[0].association_key, 99u);
  EXPECT_TRUE(seeds[0].has_position);
  EXPECT_EQ(seeds[0].position, measurement.raw_measurement.position);
  EXPECT_TRUE(seeds[0].has_gaussian_state);
  EXPECT_FLOAT_EQ(seeds[0].gaussian_state.mean(0),
                  measurement.raw_measurement.position(0));
  EXPECT_FLOAT_EQ(seeds[0].gaussian_state.mean(1),
                  measurement.filtered_feature.velocity(0));
  EXPECT_FLOAT_EQ(seeds[0].gaussian_state.mean(2),
                  measurement.raw_measurement.position(1));
  EXPECT_FLOAT_EQ(seeds[0].gaussian_state.mean(4),
                  measurement.raw_measurement.position(2));
}

TEST(TrackLifecycleManagerTest,
     FilterWritebackUpdatesAccelerationFromVelocityDelta) {
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
  first.raw_measurement.has_cartesian_position = true;
  first.raw_measurement.position = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  first.filtered_feature.velocity = Eigen::Vector3f(1.0f, 2.0f, 0.0f);
  first.raw_measurement.measurement_covariance = Eigen::Matrix3f::Identity();

  signal::tracking::CycleContext cycle_1;
  cycle_1.cycle_index = 1u;
  cycle_1.batch_id = 9001u;
  manager.Update(cycle_1, {first});

  signal::tracking::TrackMeasurement second = first;
  second.raw_measurement.matched_existing_track = true;
  second.raw_measurement.position = Eigen::Vector3f(1.0f, 2.0f, 0.0f);
  second.filtered_feature.velocity = Eigen::Vector3f(3.0f, 5.0f, 0.0f);

  signal::tracking::CycleContext cycle_2;
  cycle_2.cycle_index = 2u;
  cycle_2.batch_id = 9002u;
  manager.Update(cycle_2, {second});

  const std::vector<const common::TrackState *> active = manager.GetActiveTracks();
  ASSERT_EQ(active.size(), 1u);
  EXPECT_GT(active[0]->acceleration.norm(), 0.0f);

  const common::TargetFeatureList snapshot = manager.BuildFeatureSnapshot();
  ASSERT_EQ(snapshot.size(), 1u);
  EXPECT_NEAR(snapshot[0].current_track_speed,
              std::sqrt(snapshot[0].current_track_velocity_x * snapshot[0].current_track_velocity_x +
                        snapshot[0].current_track_velocity_y * snapshot[0].current_track_velocity_y +
                        snapshot[0].current_track_velocity_z * snapshot[0].current_track_velocity_z),
              1e-4f);
}

} } // namespace airborne_radar::tests
