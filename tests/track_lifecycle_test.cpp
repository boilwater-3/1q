// Copyright 2026. All Rights Reserved.
//
// Description: 验证轨迹对象池与生命周期管理的基础状态机行为。

#include <gtest/gtest.h>

#include "1q/airborne_radar/common/TrackTypes.h"
#include "1q/airborne_radar/signal/tracking/TrackLifecycleManager.h"
#include "airborne_radar/signal/tracking/BoostTrackPool.h"
#include "airborne_radar/signal/tracking/KalmanPredictor.h"
#include "airborne_radar/signal/tracking/KalmanUpdater.h"

namespace airborne_radar { namespace tests {

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
  measurement.filtered_feature.acceleration =
      Eigen::Vector3f(0.0f, 2.0f, 0.0f);
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
  EXPECT_FLOAT_EQ(snapshot[0].current_track_acceleration, 2.0f);
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
  first.filtered_feature.acceleration = Eigen::Vector3f::Zero();
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
  EXPECT_NEAR(snapshot[0].current_track_acceleration,
              std::sqrt(snapshot[0].current_track_acceleration_x * snapshot[0].current_track_acceleration_x +
                        snapshot[0].current_track_acceleration_y * snapshot[0].current_track_acceleration_y +
                        snapshot[0].current_track_acceleration_z * snapshot[0].current_track_acceleration_z),
              1e-4f);
}

} } // namespace airborne_radar::tests
