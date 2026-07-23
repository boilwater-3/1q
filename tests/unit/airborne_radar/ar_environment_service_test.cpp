// Copyright 2026. All Rights Reserved.
//
// @file ar_environment_service_test.cpp
// @brief 验证自然环境场景冻结、派生与运行态恢复。

#include <gtest/gtest.h>

#include <cmath>

#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/environment/SceneManager.h"

namespace airborne_radar {
namespace tests {

TEST(EnvironmentServiceTest, InitialSnapshotContainsFiniteNaturalEffects) {
  environment::EnvironmentService service;

  const session::EnvironmentSnapshot snapshot = service.SampleEnvironment();
  EXPECT_TRUE(std::isfinite(snapshot.propagation_loss_db));
  EXPECT_TRUE(std::isfinite(snapshot.atmospheric_physics_loss_db));
  EXPECT_TRUE(std::isfinite(snapshot.clutter_power_db));
  EXPECT_EQ(snapshot.cycle_index, 0U);
  EXPECT_FLOAT_EQ(snapshot.cycle_dt_sec, 0.0f);
}

TEST(EnvironmentServiceTest, DerivesAtmosphericInputsFromObservationAndTime) {
  config::EnvironmentScenarioConfig scenario;
  scenario.atmospheric_physics.enable_physical_model = true;
  scenario.atmospheric_physics.pressure_hpa = 950.0f;
  scenario.atmospheric_physics.temperature_k = 300.0f;
  scenario.atmospheric_physics.relative_humidity = 0.9f;
  scenario.atmospheric_context.has_simulation_unix_seconds = true;
  scenario.atmospheric_context.simulation_unix_seconds = 1704067200;

  environment::EnvironmentService service(scenario);
  const session::EnvironmentSnapshot snapshot = service.SampleEnvironment();

  EXPECT_GT(snapshot.atmospheric_physics_loss_db, 0.0f);
  EXPECT_GT(snapshot.effective_k_factor, 1.0f);
  EXPECT_LT(snapshot.effective_k_factor, 2.0f);
  EXPECT_EQ(snapshot.effective_day_of_year, 1);
}

TEST(EnvironmentServiceTest, VegetationPhysicsIncreasesClutter) {
  config::EnvironmentScenarioConfig baseline;
  config::EnvironmentScenarioConfig forest = baseline;
  forest.vegetation_scatter_physics.enable_physical_model = true;
  forest.vegetation_scatter_physics.cover_profile =
      config::VegetationCoverProfile::kConiferousForest;

  environment::EnvironmentService baseline_service(baseline);
  environment::EnvironmentService forest_service(forest);

  EXPECT_GT(forest_service.SampleEnvironment().clutter_power_db,
            baseline_service.SampleEnvironment().clutter_power_db);
}

TEST(EnvironmentServiceTest, PendingSceneBecomesVisibleAtBeginCycle) {
  environment::EnvironmentService service;
  const session::EnvironmentSnapshot initial = service.SampleEnvironment();

  session::EnvironmentSceneState pending;
  pending.atmospheric_physics.enable_physical_model = true;
  pending.atmospheric_physics.relative_humidity = 0.95f;
  pending.atmospheric_context.has_day_of_year = true;
  pending.atmospheric_context.day_of_year = 220;
  pending.vegetation_scatter_physics.enable_physical_model = true;
  pending.vegetation_scatter_physics.cover_profile =
      config::VegetationCoverProfile::kTropicalDense;
  service.UpdateSceneState(pending);

  EXPECT_FLOAT_EQ(service.SampleEnvironment().propagation_loss_db,
                  initial.propagation_loss_db);

  session::EnvironmentCycleContext cycle;
  cycle.cycle_index = 7U;
  cycle.dt_sec = 0.25f;
  service.BeginCycle(cycle);
  const session::EnvironmentSnapshot committed = service.SampleEnvironment();

  EXPECT_EQ(committed.cycle_index, 7U);
  EXPECT_FLOAT_EQ(committed.cycle_dt_sec, 0.25f);
  EXPECT_TRUE(committed.atmospheric_physics.enable_physical_model);
  EXPECT_EQ(committed.effective_day_of_year, 220);
  EXPECT_GT(committed.clutter_power_db, initial.clutter_power_db);
}

TEST(EnvironmentServiceTest, RuntimeStateRestoresActiveAndPendingScenes) {
  environment::EnvironmentService service;

  session::EnvironmentSceneState active;
  active.atmospheric_context.has_day_of_year = true;
  active.atmospheric_context.day_of_year = 30;
  service.UpdateSceneState(active);
  session::EnvironmentCycleContext cycle;
  cycle.cycle_index = 3U;
  cycle.dt_sec = 0.5f;
  service.BeginCycle(cycle);

  session::EnvironmentSceneState pending = active;
  pending.atmospheric_context.day_of_year = 200;
  service.UpdateSceneState(pending);
  const environment::EnvironmentServiceRuntimeState saved =
      service.CaptureRuntimeState();

  session::EnvironmentSceneState replacement;
  replacement.atmospheric_context.has_day_of_year = true;
  replacement.atmospheric_context.day_of_year = 300;
  service.UpdateSceneState(replacement);
  service.BeginCycle(session::EnvironmentCycleContext{9U, 1.0f});

  service.RestoreRuntimeState(saved);
  const session::EnvironmentSnapshot restored = service.SampleEnvironment();
  EXPECT_EQ(restored.cycle_index, 3U);
  EXPECT_FLOAT_EQ(restored.cycle_dt_sec, 0.5f);
  EXPECT_EQ(restored.effective_day_of_year, 30);
  EXPECT_EQ(service.GetPendingSceneState().atmospheric_context.day_of_year,
            200);
}

TEST(SceneManagerTest, CommitsPendingNaturalSceneOnlyAtCycleBoundary) {
  session::EnvironmentSceneState initial;
  initial.atmospheric_physics.temperature_k = 280.0f;
  environment::SceneManager scene_manager(initial);

  session::EnvironmentSceneState pending = initial;
  pending.atmospheric_physics.temperature_k = 310.0f;
  scene_manager.UpdatePendingScene(pending);

  EXPECT_FLOAT_EQ(scene_manager.GetActiveScene().atmospheric_physics.temperature_k,
                  280.0f);
  EXPECT_FLOAT_EQ(scene_manager.GetPendingScene().atmospheric_physics.temperature_k,
                  310.0f);

  scene_manager.CommitPendingScene(
      session::EnvironmentCycleContext{11U, 0.1f});
  EXPECT_FLOAT_EQ(scene_manager.GetActiveScene().atmospheric_physics.temperature_k,
                  310.0f);
  EXPECT_EQ(scene_manager.GetActiveCycleContext().cycle_index, 11U);
}

}  // namespace tests
}  // namespace airborne_radar
