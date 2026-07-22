// Copyright 2026. All Rights Reserved.
//
// @file signal_scan_schedule_test.cpp
// @brief 验证机载雷达扫描调度语义与逐周期行为。

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/airborne_radar/config/ArSessionConfigBuilder.h"
#include "airborne_radar/config/InternalExecutionConfig.h"
#include "airborne_radar/config/mapping/RuntimePatchMapper.h"
#include "airborne_radar/config/mapping/SessionToExecutionMapper.h"
#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/signal/detection/BeamControlResolver.h"
#include "airborne_radar/signal/detection/BeamwidthResolution.h"
#include "airborne_radar/signal/detection/SignalDetector.h"
#include "airborne_radar/signal/detection/TargetLookResolver.h"
#include "airborne_radar/signal/pipeline/CycleExecutor.h"
#include "airborne_radar/signal/pipeline/RuntimeAssemblySupport.h"
#include "airborne_radar/signal/pipeline/ScanScheduleResolver.h"
#include "airborne_radar/signal/pipeline/SignalPipeline.h"

namespace airborne_radar {
namespace tests {
namespace {

using ExecutionConfig = config::execution::InternalExecutionConfig;

config::ArSessionConfig MakeDetectionFocusedConfig() {
  return config::ArSessionConfigBuilder()
      .Detection()
      .WithDetectionIntentProfile(config::profiles::DetectionIntentProfile::kDetectionPriority)
      .End()
      .Tracking()
      .WithTrackingPolicyProfile(config::profiles::TrackingPolicyProfile::kFastAssociation)
      .End()
      .Lifecycle()
      .WithLifecyclePolicyProfile(config::profiles::LifecyclePolicyProfile::kFastConfirm)
      .End()
      .Build();
}

bool AlmostSamePoint(const config::AzimuthElevationDeg& lhs,
                     const config::AzimuthElevationDeg& rhs) {
  return std::fabs(lhs.az_deg - rhs.az_deg) <= 1.0e-4f &&
         std::fabs(lhs.el_deg - rhs.el_deg) <= 1.0e-4f;
}

std::size_t CountUniqueScheduledPoints(const config::ArOrientationConfig& orientation,
                                       const signal::detection::EffectiveBeamwidthDeg& beamwidth,
                                       std::uint32_t cycle_count) {
  std::vector<config::AzimuthElevationDeg> unique_points;
  for (std::uint32_t cycle = 1U; cycle <= cycle_count; ++cycle) {
    const config::AzimuthElevationDeg pointing =
        signal::pipeline::ResolveScheduledBeamPointing(orientation, beamwidth, cycle);
    bool exists = false;
    for (std::size_t i = 0; i < unique_points.size(); ++i) {
      if (AlmostSamePoint(unique_points[i], pointing)) {
        exists = true;
        break;
      }
    }
    if (!exists) {
      unique_points.push_back(pointing);
    }
  }
  return unique_points.size();
}

session::EnvironmentCycleContext MakeEnvironmentCycle(std::uint32_t cycle_index) {
  session::EnvironmentCycleContext cycle;
  cycle.cycle_index = cycle_index;
  cycle.dt_sec = 1.0f;
  return cycle;
}

session::EnvironmentSnapshot MakeEnvironmentSnapshot(std::uint32_t cycle_index) {
  session::EnvironmentSnapshot snapshot;
  snapshot.cycle_dt_sec = MakeEnvironmentCycle(cycle_index).dt_sec;
  return snapshot;
}

session::ArSceneTarget ToSceneTarget(const session::ArSceneTarget& target) {
  session::ArSceneTarget out;
  out.external_target_id = target.external_target_id;
  out.velocity_x = target.velocity_x;
  out.velocity_y = target.velocity_y;
  out.velocity_z = target.velocity_z;
  out.rcs = target.rcs;
  out.range_m = target.range_m;
  out.position_x = target.position_x;
  out.position_y = target.position_y;
  out.position_z = target.position_z;
  out.target_swerling_type = target.target_swerling_type;
  return out;
}

session::ArSceneTargetList ToSceneTargets(const session::ArSceneTargetList& targets) {
  session::ArSceneTargetList out;
  out.reserve(targets.size());
  for (std::size_t i = 0; i < targets.size(); ++i) {
    out.push_back(ToSceneTarget(targets[i]));
  }
  return out;
}

template <typename PipelineType>
session::SignalCycleResult RunPipelineCycle(PipelineType* pipeline,
                                            const session::ArSceneTargetList& input_state,
                                            environment::EnvironmentService* environment_service,
                                            std::uint32_t cycle_index) {
  environment_service->BeginCycle(MakeEnvironmentCycle(cycle_index));
  return pipeline->RunCycle(ToSceneTargets(input_state), *environment_service);
}

class NonAutoLifecycleManager final : public signal::tracking::ITrackLifecycleManager {
 public:
  void Update(const signal::tracking::CycleContext&,
              const std::vector<signal::tracking::TrackMeasurement>&) override {}

  session::ArSceneTargetList BuildSceneTargetSnapshot() const override {
    return session::ArSceneTargetList();
  }

  session::TrackStateSnapshotList BuildTrackStateSnapshots() const override {
    return session::TrackStateSnapshotList();
  }

  std::vector<signal::tracking::AssociationTrackSeed> BuildAssociationSeeds() const override {
    return std::vector<signal::tracking::AssociationTrackSeed>();
  }
};

signal::pipeline::CycleExecutionRuntime BuildMinimalValidRuntime(
    const ExecutionConfig& exec_config, const session::ArControlProfile& control_profile,
    signal::association::DataAssociationEngine* association_engine,
    signal::tracking::TrackFilter* track_filter,
    signal::tracking::ITrackLifecycleManager* lifecycle_manager,
    signal::detection::SignalDetector* signal_detector = nullptr) {
  static const std::vector<signal::tracking::AssociationTrackSeed> kEmptyAssociationSeeds;
  return signal::pipeline::CycleExecutionRuntime(exec_config, control_profile, *association_engine,
                                                 *track_filter, *lifecycle_manager, signal_detector,
                                                 kEmptyAssociationSeeds, false);
}

signal::pipeline::CycleExecutionContext BuildContext(
    const session::ArSceneTargetList& input_state,
    const session::EnvironmentSnapshot& environment_snapshot, std::uint32_t cycle_index,
    std::uint64_t batch_id, const signal::pipeline::CycleExecutionRuntime& runtime,
    float platform_altitude_m = 0.0f) {
  const signal::pipeline::ResolvedRuntimePipelineConfig resolved =
      signal::pipeline::ResolveRuntimePipelineConfig(runtime.base_config, runtime.control_profile);
  ExecutionConfig runtime_config = resolved.config;
  signal::pipeline::ApplyScanScheduleToRuntimeConfig(cycle_index, &runtime_config);
  return signal::pipeline::CycleExecutionContext(input_state, environment_snapshot, cycle_index,
                                                 batch_id, std::move(runtime_config),
                                                 platform_altitude_m);
}

TEST(ScanScheduleResolverTest, StartPositionControlsFirstBeamQuadrant) {
  config::AzimuthElevationLimitsDeg limits;
  limits.az_min_deg = -10.0f;
  limits.az_max_deg = 10.0f;
  limits.el_min_deg = -5.0f;
  limits.el_max_deg = 5.0f;

  const std::vector<config::AzimuthElevationDeg> left_top_pattern =
      signal::pipeline::BuildScheduledScanPattern(limits, 10.0f, 5.0f,
                                                  oneq::foundation::ScanStartPosition::kLeftTop,
                                                  oneq::foundation::ScanSequence::kAzimuthFirst);
  const std::vector<config::AzimuthElevationDeg> right_top_pattern =
      signal::pipeline::BuildScheduledScanPattern(limits, 10.0f, 5.0f,
                                                  oneq::foundation::ScanStartPosition::kRightTop,
                                                  oneq::foundation::ScanSequence::kAzimuthFirst);
  const std::vector<config::AzimuthElevationDeg> right_bottom_pattern =
      signal::pipeline::BuildScheduledScanPattern(limits, 10.0f, 5.0f,
                                                  oneq::foundation::ScanStartPosition::kRightBottom,
                                                  oneq::foundation::ScanSequence::kAzimuthFirst);
  const std::vector<config::AzimuthElevationDeg> left_bottom_pattern =
      signal::pipeline::BuildScheduledScanPattern(limits, 10.0f, 5.0f,
                                                  oneq::foundation::ScanStartPosition::kLeftBottom,
                                                  oneq::foundation::ScanSequence::kAzimuthFirst);

  ASSERT_FALSE(left_top_pattern.empty());
  ASSERT_FALSE(right_top_pattern.empty());
  ASSERT_FALSE(right_bottom_pattern.empty());
  ASSERT_FALSE(left_bottom_pattern.empty());

  EXPECT_FLOAT_EQ(left_top_pattern.front().az_deg, -10.0f);
  EXPECT_FLOAT_EQ(left_top_pattern.front().el_deg, 5.0f);
  EXPECT_FLOAT_EQ(right_top_pattern.front().az_deg, 10.0f);
  EXPECT_FLOAT_EQ(right_top_pattern.front().el_deg, 5.0f);
  EXPECT_FLOAT_EQ(right_bottom_pattern.front().az_deg, 10.0f);
  EXPECT_FLOAT_EQ(right_bottom_pattern.front().el_deg, -5.0f);
  EXPECT_FLOAT_EQ(left_bottom_pattern.front().az_deg, -10.0f);
  EXPECT_FLOAT_EQ(left_bottom_pattern.front().el_deg, -5.0f);
}

TEST(ScanScheduleResolverTest, ApplyScanScheduleUsesPolicyBeamControlInputs) {
  config::ArSessionConfig session_config = MakeDetectionFocusedConfig();
  session_config.mission.orientation.work_mode = config::ArWorkMode::kTas;
  session_config.mission.orientation.scan_center_deg.az_deg = 3.0f;
  session_config.mission.orientation.scan_center_deg.el_deg = -1.0f;
  session_config.mission.orientation.mechanical_scan_limits_deg.az_min_deg = -10.0f;
  session_config.mission.orientation.mechanical_scan_limits_deg.az_max_deg = 10.0f;
  session_config.mission.orientation.mechanical_scan_limits_deg.el_min_deg = -4.0f;
  session_config.mission.orientation.mechanical_scan_limits_deg.el_max_deg = 4.0f;
  session_config.mission.orientation.electronic_scan_limits_deg =
      session_config.mission.orientation.mechanical_scan_limits_deg;
  session_config.policy.beam_control.pointing.nominal_beamwidth_deg.commanded_az_beamwidth_deg =
      4.0f;
  session_config.policy.beam_control.pointing.nominal_beamwidth_deg.commanded_el_beamwidth_deg =
      2.0f;
  session_config.policy.beam_control.scheduler.azimuth_step_count_hint = 6U;
  session_config.policy.beam_control.scheduler.elevation_step_count_hint = 5U;
  session_config.policy.beam_control.scheduler.prefer_dense_tas_sampling = true;
  ExecutionConfig runtime_config = config::mapping::MapSessionToExecution(session_config);

  signal::pipeline::ApplyScanScheduleToRuntimeConfig(1U, &runtime_config);

  EXPECT_FLOAT_EQ(runtime_config.detection.orientation.scan_center_deg.az_deg, -10.0f);
  EXPECT_FLOAT_EQ(runtime_config.detection.orientation.scan_center_deg.el_deg, 4.0f);
}

TEST(CycleExecutorTest, ValidRuntimeProducesInputAlignedStageBuffers) {
  const ExecutionConfig exec_config =
      config::mapping::MapSessionToExecution(MakeDetectionFocusedConfig());
  const session::ArControlProfile control_profile{};

  signal::association::DataAssociationEngine association_engine;
  signal::tracking::TrackFilter track_filter;
  std::unique_ptr<signal::tracking::ITrackLifecycleManager> lifecycle_manager =
      signal::pipeline::CreateAutoLifecycleManagerForRuntimeConfig(exec_config);
  ASSERT_TRUE(lifecycle_manager != nullptr);
  const signal::pipeline::CycleExecutionRuntime runtime = BuildMinimalValidRuntime(
      exec_config, control_profile, &association_engine, &track_filter, lifecycle_manager.get());

  session::ArSceneTarget target_a(1000.0f, 0.0f, 0.0f, 2.0f);

  target_a.position_x = 1000.0f;
  target_a.range_m = 1000.0f;
  session::ArSceneTarget target_b(1500.0f, 0.0f, 0.0f, 3.0f);

  target_b.position_x = 1500.0f;
  target_b.range_m = 1500.0f;
  const session::ArSceneTargetList input_state{target_a, target_b};

  environment::EnvironmentService environment_service;
  signal::pipeline::CycleExecutionScratch scratch;
  auto context = BuildContext(input_state, MakeEnvironmentSnapshot(3U), 3U, 9U, runtime);
  EXPECT_TRUE(signal::pipeline::ExecuteCycle(context, runtime, scratch));

  EXPECT_EQ(scratch.output_state.size(), input_state.size());
  EXPECT_EQ(scratch.signal_term_db.size(), input_state.size());
  EXPECT_EQ(scratch.speed_penalty_db.size(), input_state.size());
  EXPECT_EQ(scratch.detection_margin_db.size(), input_state.size());
  EXPECT_EQ(scratch.detection_succeeded.size(), input_state.size());
  EXPECT_EQ(scratch.association_keys.size(), input_state.size());
  EXPECT_EQ(scratch.measurement_covariances.size(), input_state.size());
  EXPECT_EQ(scratch.measurement_slots.size(), input_state.size());
  EXPECT_EQ(scratch.target_geometry.size(), input_state.size());
  EXPECT_EQ(scratch.decision_frame.cycle_index, 3U);
  EXPECT_EQ(scratch.decision_frame.batch_id, 9U);
}

TEST(CycleExecutorTest, EmptyInputKeepsWorkspaceOutputsEmpty) {
  ExecutionConfig exec_config;
  const session::ArControlProfile control_profile{};

  signal::association::DataAssociationEngine association_engine;
  signal::tracking::TrackFilter track_filter;
  std::unique_ptr<signal::tracking::ITrackLifecycleManager> lifecycle_manager =
      signal::pipeline::CreateAutoLifecycleManagerForRuntimeConfig(exec_config);
  ASSERT_TRUE(lifecycle_manager != nullptr);
  const signal::pipeline::CycleExecutionRuntime runtime = BuildMinimalValidRuntime(
      exec_config, control_profile, &association_engine, &track_filter, lifecycle_manager.get());

  const session::ArSceneTargetList input_state;
  signal::pipeline::CycleExecutionScratch scratch;
  auto context = BuildContext(input_state, MakeEnvironmentSnapshot(1U), 1U, 1U, runtime);
  EXPECT_TRUE(signal::pipeline::ExecuteCycle(context, runtime, scratch));

  EXPECT_TRUE(scratch.output_state.empty());
  EXPECT_TRUE(scratch.track_measurements.empty());
  EXPECT_TRUE(scratch.detection_succeeded.empty());
  EXPECT_TRUE(scratch.association_keys.empty());
  EXPECT_TRUE(scratch.measurement_covariances.empty());
}

TEST(CycleExecutorTest, PhysicalAtmosphereUsesPlatformAbsoluteAltitude) {
  ExecutionConfig exec_config;
  exec_config.detection.engineering.min_detection_margin_db = -300.0f;
  exec_config.detection.orientation.scan_center_deg.az_deg = 0.0f;
  exec_config.detection.orientation.scan_center_deg.el_deg = 0.0f;
  exec_config.detection.orientation.mechanical_scan_limits_deg.az_min_deg = 0.0f;
  exec_config.detection.orientation.mechanical_scan_limits_deg.az_max_deg = 0.0f;
  exec_config.detection.orientation.mechanical_scan_limits_deg.el_min_deg = 0.0f;
  exec_config.detection.orientation.mechanical_scan_limits_deg.el_max_deg = 0.0f;
  exec_config.detection.orientation.electronic_scan_limits_deg =
      exec_config.detection.orientation.mechanical_scan_limits_deg;

  session::EnvironmentSnapshot environment_snapshot = MakeEnvironmentSnapshot(1U);
  environment_snapshot.atmospheric_physics.enable_physical_model = true;
  environment_snapshot.atmospheric_physics.pressure_hpa = 1013.25f;
  environment_snapshot.atmospheric_physics.temperature_k = 288.15f;
  environment_snapshot.atmospheric_physics.relative_humidity = 0.5f;
  environment_snapshot.effective_k_factor = 4.0f / 3.0f;
  environment_snapshot.effective_day_of_year = 172;

  session::ArSceneTarget target(220.0f, 0.0f, 0.0f, 1.0e6f);
  target.position_x = 50000.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 50000.0f;
  const session::ArSceneTargetList input_state{target};

  const session::ArControlProfile control_profile{};
  signal::association::DataAssociationEngine association_engine;
  signal::tracking::TrackFilter track_filter;
  std::unique_ptr<signal::tracking::ITrackLifecycleManager> lifecycle_manager =
      signal::pipeline::CreateAutoLifecycleManagerForRuntimeConfig(exec_config);
  ASSERT_TRUE(lifecycle_manager != nullptr);
  signal::detection::SignalDetector signal_detector(exec_config.detection.engineering);
  const signal::pipeline::CycleExecutionRuntime runtime =
      BuildMinimalValidRuntime(exec_config, control_profile, &association_engine, &track_filter,
                               lifecycle_manager.get(), &signal_detector);

  signal::pipeline::CycleExecutionScratch sea_level_scratch;
  auto sea_level_context = BuildContext(input_state, environment_snapshot, 1U, 1U, runtime, 1.0f);
  ASSERT_TRUE(signal::pipeline::ExecuteCycle(sea_level_context, runtime, sea_level_scratch));
  ASSERT_EQ(sea_level_scratch.signal_term_db.size(), 1U);

  signal::pipeline::CycleExecutionScratch elevated_scratch;
  auto elevated_context = BuildContext(input_state, environment_snapshot, 1U, 2U, runtime, 1000.0f);
  ASSERT_TRUE(signal::pipeline::ExecuteCycle(elevated_context, runtime, elevated_scratch));
  ASSERT_EQ(elevated_scratch.signal_term_db.size(), 1U);

  EXPECT_GT(elevated_scratch.signal_term_db[0], sea_level_scratch.signal_term_db[0]);
}

TEST(CycleExecutorTest, PhysicalDetectionTreatsClutterDbAsThermalRelativeNoise) {
  const config::ArSessionConfig session_config =
      config::ArSessionConfigBuilder()
          .Detection()
          .WithHardwareProfile(config::profiles::ArHardwareProfile::kLongRangeHighPower)
          .WithDetectionIntentProfile(config::profiles::DetectionIntentProfile::kDetectionPriority)
          .WithAntennaPatternProfile(config::profiles::AntennaPatternProfile::kStandard)
          .End()
          .Build();
  ExecutionConfig exec_config = config::mapping::MapSessionToExecution(session_config);
  exec_config.detection.orientation.scan_center_deg.az_deg = 0.0f;
  exec_config.detection.orientation.scan_center_deg.el_deg = 0.0f;
  exec_config.detection.orientation.mechanical_scan_limits_deg.az_min_deg = 0.0f;
  exec_config.detection.orientation.mechanical_scan_limits_deg.az_max_deg = 0.0f;
  exec_config.detection.orientation.mechanical_scan_limits_deg.el_min_deg = 0.0f;
  exec_config.detection.orientation.mechanical_scan_limits_deg.el_max_deg = 0.0f;
  exec_config.detection.orientation.electronic_scan_limits_deg =
      exec_config.detection.orientation.mechanical_scan_limits_deg;

  session::EnvironmentSnapshot environment_snapshot = MakeEnvironmentSnapshot(1U);
  environment_snapshot.propagation_loss_db = 6.5f;
  environment_snapshot.clutter_power_db = 3.0f;

  session::ArSceneTarget target(0.0f, 0.0f, 0.0f, 10.0f, 80000.0f, 0, 80001U);
  target.position_x = 80000.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  const session::ArSceneTargetList input_state{target};

  const session::ArControlProfile control_profile{};
  signal::association::DataAssociationEngine association_engine;
  signal::tracking::TrackFilter track_filter;
  std::unique_ptr<signal::tracking::ITrackLifecycleManager> lifecycle_manager =
      signal::pipeline::CreateAutoLifecycleManagerForRuntimeConfig(exec_config);
  ASSERT_TRUE(lifecycle_manager != nullptr);
  signal::detection::SignalDetector signal_detector(exec_config.detection.engineering);
  const signal::pipeline::CycleExecutionRuntime runtime =
      BuildMinimalValidRuntime(exec_config, control_profile, &association_engine, &track_filter,
                               lifecycle_manager.get(), &signal_detector);

  signal::pipeline::CycleExecutionScratch scratch;
  auto context = BuildContext(input_state, environment_snapshot, 1U, 1U, runtime, 1000.0f);
  ASSERT_TRUE(signal::pipeline::ExecuteCycle(context, runtime, scratch));
  ASSERT_EQ(scratch.signal_term_db.size(), 1U);
  ASSERT_EQ(scratch.detection_succeeded.size(), 1U);

  EXPECT_GT(scratch.signal_term_db[0],
            exec_config.detection.engineering.detection_policy.min_snr_db);
  EXPECT_GT(scratch.signal_term_db[0], 5.0f);
  EXPECT_EQ(scratch.detection_succeeded[0], 1U);
}

TEST(CycleExecutorTest, NonAutoLifecycleManagerCausesRuntimeSyncFailure) {
  ExecutionConfig exec_config;
  const session::ArControlProfile control_profile{};

  signal::association::DataAssociationEngine association_engine;
  signal::tracking::TrackFilter track_filter;
  NonAutoLifecycleManager lifecycle_manager;
  const signal::pipeline::CycleExecutionRuntime runtime = BuildMinimalValidRuntime(
      exec_config, control_profile, &association_engine, &track_filter, &lifecycle_manager);

  session::ArSceneTarget target(1000.0f, 0.0f, 0.0f, 2.0f);

  target.position_x = 1000.0f;
  target.range_m = 1000.0f;

  signal::pipeline::CycleExecutionScratch scratch;
  auto context = BuildContext(session::ArSceneTargetList{target}, MakeEnvironmentSnapshot(1U), 1U,
                              1U, runtime);
  EXPECT_FALSE(signal::pipeline::ExecuteCycle(context, runtime, scratch));
  EXPECT_TRUE(scratch.track_measurements.empty());
  EXPECT_EQ(scratch.decision_frame.cycle_index, 0U);
}

TEST(ScanScheduleResolverTest, SequenceControlsFastScanAxisWithSerpentine) {
  config::AzimuthElevationLimitsDeg limits;
  limits.az_min_deg = -10.0f;
  limits.az_max_deg = 10.0f;
  limits.el_min_deg = -5.0f;
  limits.el_max_deg = 5.0f;

  const std::vector<config::AzimuthElevationDeg> azimuth_first_pattern =
      signal::pipeline::BuildScheduledScanPattern(limits, 10.0f, 5.0f,
                                                  oneq::foundation::ScanStartPosition::kLeftTop,
                                                  oneq::foundation::ScanSequence::kAzimuthFirst);
  const std::vector<config::AzimuthElevationDeg> elevation_first_pattern =
      signal::pipeline::BuildScheduledScanPattern(limits, 10.0f, 5.0f,
                                                  oneq::foundation::ScanStartPosition::kLeftTop,
                                                  oneq::foundation::ScanSequence::kElevationFirst);

  ASSERT_GE(azimuth_first_pattern.size(), 4U);
  ASSERT_GE(elevation_first_pattern.size(), 4U);

  EXPECT_FLOAT_EQ(azimuth_first_pattern[0].az_deg, -10.0f);
  EXPECT_FLOAT_EQ(azimuth_first_pattern[0].el_deg, 5.0f);
  EXPECT_FLOAT_EQ(azimuth_first_pattern[1].az_deg, 0.0f);
  EXPECT_FLOAT_EQ(azimuth_first_pattern[1].el_deg, 5.0f);
  EXPECT_FLOAT_EQ(azimuth_first_pattern[3].az_deg, 10.0f);
  EXPECT_FLOAT_EQ(azimuth_first_pattern[3].el_deg, 0.0f);

  EXPECT_FLOAT_EQ(elevation_first_pattern[0].az_deg, -10.0f);
  EXPECT_FLOAT_EQ(elevation_first_pattern[0].el_deg, 5.0f);
  EXPECT_FLOAT_EQ(elevation_first_pattern[1].az_deg, -10.0f);
  EXPECT_FLOAT_EQ(elevation_first_pattern[1].el_deg, 0.0f);
  EXPECT_FLOAT_EQ(elevation_first_pattern[3].az_deg, 0.0f);
  EXPECT_FLOAT_EQ(elevation_first_pattern[3].el_deg, -5.0f);
}

TEST(ScanScheduleResolverTest, InvalidStepFallsBackToClampedScanCenter) {
  config::ArOrientationConfig orientation;
  orientation.scan_center_deg.az_deg = 80.0f;
  orientation.scan_center_deg.el_deg = 40.0f;
  orientation.mechanical_scan_limits_deg.az_min_deg = -30.0f;
  orientation.mechanical_scan_limits_deg.az_max_deg = 30.0f;
  orientation.mechanical_scan_limits_deg.el_min_deg = -10.0f;
  orientation.mechanical_scan_limits_deg.el_max_deg = 10.0f;
  orientation.electronic_scan_limits_deg = orientation.mechanical_scan_limits_deg;

  signal::detection::EffectiveBeamwidthDeg invalid_beamwidth;
  invalid_beamwidth.az_beamwidth_deg = 0.0f;
  invalid_beamwidth.el_beamwidth_deg = 5.0f;

  const config::AzimuthElevationDeg pointing =
      signal::pipeline::ResolveScheduledBeamPointing(orientation, invalid_beamwidth, 3U);
  EXPECT_FLOAT_EQ(pointing.az_deg, 30.0f);
  EXPECT_FLOAT_EQ(pointing.el_deg, 10.0f);

  const config::AzimuthElevationDeg dwell_center =
      signal::pipeline::ResolveScheduledDwellCenter(orientation, invalid_beamwidth, 3U);
  EXPECT_FLOAT_EQ(dwell_center.az_deg, -50.0f);
  EXPECT_FLOAT_EQ(dwell_center.el_deg, -30.0f);
}

TEST(ScanScheduleResolverTest, FirstCycleMapsToFirstBeamIndex) {
  config::ArOrientationConfig orientation;
  orientation.scan_center_deg.az_deg = 0.0f;
  orientation.scan_center_deg.el_deg = 0.0f;
  orientation.mechanical_scan_limits_deg.az_min_deg = -60.0f;
  orientation.mechanical_scan_limits_deg.az_max_deg = 60.0f;
  orientation.mechanical_scan_limits_deg.el_min_deg = 0.0f;
  orientation.mechanical_scan_limits_deg.el_max_deg = 0.0f;
  orientation.electronic_scan_limits_deg = orientation.mechanical_scan_limits_deg;
  orientation.scan_start_position = oneq::foundation::ScanStartPosition::kLeftTop;
  orientation.scan_sequence = oneq::foundation::ScanSequence::kAzimuthFirst;

  signal::detection::EffectiveBeamwidthDeg beamwidth;
  beamwidth.az_beamwidth_deg = 120.0f;
  beamwidth.el_beamwidth_deg = 10.0f;

  const config::AzimuthElevationDeg cycle_1 =
      signal::pipeline::ResolveScheduledBeamPointing(orientation, beamwidth, 1U);
  const config::AzimuthElevationDeg cycle_2 =
      signal::pipeline::ResolveScheduledBeamPointing(orientation, beamwidth, 2U);
  EXPECT_FLOAT_EQ(cycle_1.az_deg, -60.0f);
  EXPECT_FLOAT_EQ(cycle_2.az_deg, 60.0f);
}

TEST(ScanScheduleResolverTest, StbyParksAtClampedBoresightWithoutCycleAdvance) {
  config::ArOrientationConfig orientation;
  orientation.work_mode = config::ArWorkMode::kStby;
  orientation.scan_center_deg.az_deg = 20.0f;
  orientation.scan_center_deg.el_deg = -8.0f;
  orientation.mechanical_scan_limits_deg.az_min_deg = 5.0f;
  orientation.mechanical_scan_limits_deg.az_max_deg = 30.0f;
  orientation.mechanical_scan_limits_deg.el_min_deg = -3.0f;
  orientation.mechanical_scan_limits_deg.el_max_deg = 4.0f;
  orientation.electronic_scan_limits_deg = orientation.mechanical_scan_limits_deg;

  signal::detection::EffectiveBeamwidthDeg beamwidth;
  beamwidth.az_beamwidth_deg = 8.0f;
  beamwidth.el_beamwidth_deg = 6.0f;

  const config::AzimuthElevationDeg cycle_1 =
      signal::pipeline::ResolveScheduledBeamPointing(orientation, beamwidth, 1U);
  const config::AzimuthElevationDeg cycle_9 =
      signal::pipeline::ResolveScheduledBeamPointing(orientation, beamwidth, 9U);
  EXPECT_TRUE(AlmostSamePoint(cycle_1, cycle_9));
  EXPECT_FLOAT_EQ(cycle_1.az_deg, 5.0f);
  EXPECT_FLOAT_EQ(cycle_1.el_deg, 0.0f);

  const config::AzimuthElevationDeg dwell =
      signal::pipeline::ResolveScheduledDwellCenter(orientation, beamwidth, 5U);
  EXPECT_FLOAT_EQ(dwell.az_deg, cycle_1.az_deg - orientation.scan_center_deg.az_deg);
  EXPECT_FLOAT_EQ(dwell.el_deg, cycle_1.el_deg - orientation.scan_center_deg.el_deg);
}

TEST(ScanScheduleResolverTest, SttFixesAtScanCenterAndKeepsZeroDwell) {
  config::ArOrientationConfig orientation;
  orientation.work_mode = config::ArWorkMode::kStt;
  orientation.scan_center_deg.az_deg = 12.5f;
  orientation.scan_center_deg.el_deg = -4.5f;
  orientation.mechanical_scan_limits_deg.az_min_deg = -60.0f;
  orientation.mechanical_scan_limits_deg.az_max_deg = 60.0f;
  orientation.mechanical_scan_limits_deg.el_min_deg = -30.0f;
  orientation.mechanical_scan_limits_deg.el_max_deg = 30.0f;
  orientation.electronic_scan_limits_deg = orientation.mechanical_scan_limits_deg;

  signal::detection::EffectiveBeamwidthDeg beamwidth;
  beamwidth.az_beamwidth_deg = 10.0f;
  beamwidth.el_beamwidth_deg = 10.0f;

  const config::AzimuthElevationDeg cycle_1 =
      signal::pipeline::ResolveScheduledBeamPointing(orientation, beamwidth, 1U);
  const config::AzimuthElevationDeg cycle_7 =
      signal::pipeline::ResolveScheduledBeamPointing(orientation, beamwidth, 7U);
  EXPECT_TRUE(AlmostSamePoint(cycle_1, cycle_7));
  EXPECT_FLOAT_EQ(cycle_1.az_deg, orientation.scan_center_deg.az_deg);
  EXPECT_FLOAT_EQ(cycle_1.el_deg, orientation.scan_center_deg.el_deg);

  const config::AzimuthElevationDeg dwell =
      signal::pipeline::ResolveScheduledDwellCenter(orientation, beamwidth, 7U);
  EXPECT_FLOAT_EQ(dwell.az_deg, 0.0f);
  EXPECT_FLOAT_EQ(dwell.el_deg, 0.0f);
}

TEST(ScanScheduleResolverTest, ResolveScanStepScaleDefinesDirectCallFallbacks) {
  EXPECT_FLOAT_EQ(signal::pipeline::ResolveScanStepScale(config::ArWorkMode::kTas), 0.5f);
  EXPECT_FLOAT_EQ(signal::pipeline::ResolveScanStepScale(config::ArWorkMode::kTws), 1.0f);
  EXPECT_FLOAT_EQ(signal::pipeline::ResolveScanStepScale(config::ArWorkMode::kStby), 1.0f);
  EXPECT_FLOAT_EQ(signal::pipeline::ResolveScanStepScale(config::ArWorkMode::kStt), 1.0f);
  EXPECT_FLOAT_EQ(signal::pipeline::ResolveScanStepScale(static_cast<config::ArWorkMode>(999)),
                  1.0f);
}

TEST(ScanScheduleResolverTest, TasIsDenserThanTwsAndKeepsSerpentineSemantics) {
  config::ArOrientationConfig tws_orientation;
  tws_orientation.work_mode = config::ArWorkMode::kTws;
  tws_orientation.scan_start_position = oneq::foundation::ScanStartPosition::kLeftTop;
  tws_orientation.scan_sequence = oneq::foundation::ScanSequence::kAzimuthFirst;
  tws_orientation.mechanical_scan_limits_deg.az_min_deg = -20.0f;
  tws_orientation.mechanical_scan_limits_deg.az_max_deg = 20.0f;
  tws_orientation.mechanical_scan_limits_deg.el_min_deg = -10.0f;
  tws_orientation.mechanical_scan_limits_deg.el_max_deg = 10.0f;
  tws_orientation.electronic_scan_limits_deg = tws_orientation.mechanical_scan_limits_deg;
  config::ArOrientationConfig tas_orientation = tws_orientation;
  tas_orientation.work_mode = config::ArWorkMode::kTas;

  signal::detection::EffectiveBeamwidthDeg beamwidth;
  beamwidth.az_beamwidth_deg = 20.0f;
  beamwidth.el_beamwidth_deg = 10.0f;

  const config::AzimuthElevationDeg tws_first =
      signal::pipeline::ResolveScheduledBeamPointing(tws_orientation, beamwidth, 1U);
  const config::AzimuthElevationDeg tws_second =
      signal::pipeline::ResolveScheduledBeamPointing(tws_orientation, beamwidth, 2U);
  const config::AzimuthElevationDeg tas_first =
      signal::pipeline::ResolveScheduledBeamPointing(tas_orientation, beamwidth, 1U);
  const config::AzimuthElevationDeg tas_second =
      signal::pipeline::ResolveScheduledBeamPointing(tas_orientation, beamwidth, 2U);
  EXPECT_FLOAT_EQ(tws_first.az_deg, -20.0f);
  EXPECT_FLOAT_EQ(tws_first.el_deg, 10.0f);
  EXPECT_FLOAT_EQ(tas_first.az_deg, -20.0f);
  EXPECT_FLOAT_EQ(tas_first.el_deg, 10.0f);
  EXPECT_FLOAT_EQ(tws_first.el_deg, tws_second.el_deg);
  EXPECT_FLOAT_EQ(tas_first.el_deg, tas_second.el_deg);
  EXPECT_GT(tws_second.az_deg, tws_first.az_deg);
  EXPECT_GT(tas_second.az_deg, tas_first.az_deg);

  const std::size_t tws_unique = CountUniqueScheduledPoints(tws_orientation, beamwidth, 128U);
  const std::size_t tas_unique = CountUniqueScheduledPoints(tas_orientation, beamwidth, 128U);
  EXPECT_GT(tas_unique, tws_unique);
}

TEST(SignalPipelineScanScheduleTest, RunCycleAdvancesBeamAndChangesDetectionOutcome) {
  config::ArSessionConfig session_config;
  session_config.policy.detection.pulse_count = 16;
  session_config.policy.detection.pfa = 2.0e-6f;
  session_config.policy.detection.minimum_snr_db = -12.0f;
  session_config.policy.detection.minimum_detection_margin_db = -100.0f;
  session_config.hardware.antenna.pattern.model_type =
      config::AntennaPatternModelType::kParabolicMainLobe;
  session_config.hardware.antenna.pattern.max_sidelobe_level_db = -18.0f;
  session_config.hardware.antenna.pattern.max_scan_loss_db = 8.0f;

  ExecutionConfig exec_config = config::mapping::MapSessionToExecution(session_config);
  exec_config.detection.engineering.pulse_count = 4096;
  exec_config.detection.engineering.detection_policy.cfar_pfa = 0.999999f;
  exec_config.detection.engineering.detection_policy.min_snr_db = -50.0f;
  exec_config.detection.engineering.antenna.nominal_az_beamwidth_deg = 120.0f;
  exec_config.detection.engineering.antenna.nominal_el_beamwidth_deg = 10.0f;
  exec_config.detection.engineering.antenna.enable_directional_pattern = true;
  exec_config.detection.engineering.antenna.pattern.max_sidelobe_level_db = -80.0f;
  exec_config.detection.engineering.antenna.pattern.backlobe_level_db = -80.0f;

  config::ArOrientationConfig& orientation = exec_config.detection.orientation;
  orientation.scan_center_deg.az_deg = 0.0f;
  orientation.scan_center_deg.el_deg = 0.0f;
  orientation.mechanical_scan_limits_deg.az_min_deg = -60.0f;
  orientation.mechanical_scan_limits_deg.az_max_deg = 60.0f;
  orientation.mechanical_scan_limits_deg.el_min_deg = 0.0f;
  orientation.mechanical_scan_limits_deg.el_max_deg = 0.0f;
  orientation.electronic_scan_limits_deg = orientation.mechanical_scan_limits_deg;
  orientation.scan_start_position = oneq::foundation::ScanStartPosition::kLeftTop;
  orientation.scan_sequence = oneq::foundation::ScanSequence::kAzimuthFirst;

  session_config = config::mapping::MapExecutionToSession(exec_config);
  signal::pipeline::SignalPipeline signal_pipeline(session_config);
  config::EnvironmentScenarioConfig environment_config;
  environment::EnvironmentService environment_service(environment_config);

  session::ArSceneTarget target(0.0f, 0.0f, 0.0f, 10000.0f, 120.0f, 0, 2026U);

  target.position_x = 120.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  const session::ArSceneTargetList targets(1U, target);

  const signal::detection::TargetLookAnglesDeg look_angles =
      signal::detection::TargetLookResolver::Resolve(target);
  ASSERT_TRUE(look_angles.has_look_angles);
  const signal::detection::EffectiveBeamwidthDeg effective_beamwidth =
      signal::detection::ResolveEffectiveBeamwidth(exec_config.detection.engineering.antenna,
                                                   orientation);

  const config::AzimuthElevationDeg cycle_1_dwell_center =
      signal::pipeline::ResolveScheduledDwellCenter(orientation, effective_beamwidth, 1U);
  const config::AzimuthElevationDeg cycle_2_dwell_center =
      signal::pipeline::ResolveScheduledDwellCenter(orientation, effective_beamwidth, 2U);
  const config::PlatformAttitudeDeg platform_attitude_deg{};
  const signal::detection::ResolvedBeamState cycle_1_beam =
      signal::detection::BeamControlResolver::Resolve(exec_config.detection.engineering.antenna,
                                                      orientation, platform_attitude_deg,
                                                      look_angles, cycle_1_dwell_center);
  const signal::detection::ResolvedBeamState cycle_2_beam =
      signal::detection::BeamControlResolver::Resolve(exec_config.detection.engineering.antenna,
                                                      orientation, platform_attitude_deg,
                                                      look_angles, cycle_2_dwell_center);

  signal::detection::TargetReturn target_return;
  target_return.rcs_m2 = target.rcs;
  target_return.range_m = target.range_m;
  target_return.swerling_type =
      static_cast<config::profiles::SwerlingModel>(target.target_swerling_type);
  signal::detection::EnvironmentState environment_state;
  signal::detection::SignalDetector detector(exec_config.detection.engineering);
  const signal::detection::DetectionResult cycle_1_detection =
      detector.Detect(target_return, environment_state, cycle_1_beam.one_way_antenna_gain_db,
                      exec_config.detection.engineering.pulse_count);
  const signal::detection::DetectionResult cycle_2_detection =
      detector.Detect(target_return, environment_state, cycle_2_beam.one_way_antenna_gain_db,
                      exec_config.detection.engineering.pulse_count);
  ASSERT_GE(cycle_2_detection.snr_db, cycle_1_detection.snr_db);
  ASSERT_GE(cycle_2_detection.detection_prob, cycle_1_detection.detection_prob);

  std::size_t aligned_detected = 0U;
  std::size_t misaligned_detected = 0U;
  const std::size_t kCycleCount = 64U;
  for (std::size_t i = 0; i < kCycleCount; ++i) {
    const session::SignalCycleResult cycle_result = RunPipelineCycle(
        &signal_pipeline, targets, &environment_service, static_cast<std::uint32_t>(i + 1U));
    if ((i % 2U) == 0U) {
      misaligned_detected += cycle_result.association_quality_metrics.detection_count;
    } else {
      aligned_detected += cycle_result.association_quality_metrics.detection_count;
    }
  }

  EXPECT_GE(aligned_detected, misaligned_detected);
}

TEST(SignalPipelineScanScheduleTest, WorkModeSttReducesSweepCoverageComparedToTws) {
  config::ArSessionConfig tws_session;
  tws_session.policy.detection.pulse_count = 16;
  tws_session.policy.detection.pfa = 2.0e-6f;
  tws_session.policy.detection.minimum_snr_db = -12.0f;
  tws_session.policy.detection.minimum_detection_margin_db = -100.0f;
  tws_session.hardware.antenna.pattern.model_type =
      config::AntennaPatternModelType::kParabolicMainLobe;
  tws_session.hardware.antenna.pattern.max_sidelobe_level_db = -18.0f;
  tws_session.hardware.antenna.pattern.max_scan_loss_db = 8.0f;
  tws_session.mission.orientation.work_mode = config::ArWorkMode::kTws;
  tws_session.mission.orientation.scan_center_deg.az_deg = -60.0f;
  tws_session.mission.orientation.scan_center_deg.el_deg = 0.0f;
  tws_session.mission.orientation.mechanical_scan_limits_deg.az_min_deg = -60.0f;
  tws_session.mission.orientation.mechanical_scan_limits_deg.az_max_deg = 60.0f;
  tws_session.mission.orientation.mechanical_scan_limits_deg.el_min_deg = 0.0f;
  tws_session.mission.orientation.mechanical_scan_limits_deg.el_max_deg = 0.0f;
  tws_session.mission.orientation.electronic_scan_limits_deg =
      tws_session.mission.orientation.mechanical_scan_limits_deg;
  tws_session.mission.orientation.scan_start_position =
      oneq::foundation::ScanStartPosition::kLeftTop;
  tws_session.mission.orientation.scan_sequence = oneq::foundation::ScanSequence::kAzimuthFirst;

  config::ArSessionConfig stt_session = tws_session;
  stt_session.mission.orientation.work_mode = config::ArWorkMode::kStt;

  signal::pipeline::SignalPipeline tws_pipeline(tws_session);
  signal::pipeline::SignalPipeline stt_pipeline(stt_session);

  config::EnvironmentScenarioConfig environment_config;
  environment::EnvironmentService environment_service(environment_config);

  session::ArSceneTarget target(0.0f, 0.0f, 0.0f, 10000.0f, 120.0f, 0, 2026U);

  target.position_x = 120.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  const session::ArSceneTargetList targets(1U, target);

  std::size_t tws_detected = 0U;
  std::size_t stt_detected = 0U;
  const std::size_t kCycleCount = 64U;
  for (std::size_t i = 0; i < kCycleCount; ++i) {
    tws_detected += RunPipelineCycle(&tws_pipeline, targets, &environment_service,
                                     static_cast<std::uint32_t>(i + 1U))
                        .association_quality_metrics.detection_count;
    stt_detected += RunPipelineCycle(&stt_pipeline, targets, &environment_service,
                                     static_cast<std::uint32_t>(i + 1U))
                        .association_quality_metrics.detection_count;
  }

  EXPECT_GE(tws_detected, stt_detected);
}

// ===========================================================================
// BuildScheduledScanPattern / ResolveFiniteScanCenter / Apply 边界校验
// ===========================================================================

TEST(ScanScheduleResolverTest, BuildPatternRejectsAzMinGreaterThanAzMax) {
  config::AzimuthElevationLimitsDeg limits;
  limits.az_min_deg = 10.0f;
  limits.az_max_deg = -10.0f;
  limits.el_min_deg = -5.0f;
  limits.el_max_deg = 5.0f;
  const auto pattern = signal::pipeline::BuildScheduledScanPattern(
      limits, 2.0f, 2.0f, oneq::foundation::ScanStartPosition::kLeftTop,
      oneq::foundation::ScanSequence::kAzimuthFirst);
  EXPECT_TRUE(pattern.empty());
}

TEST(ScanScheduleResolverTest, BuildPatternRejectsElMinGreaterThanElMax) {
  config::AzimuthElevationLimitsDeg limits;
  limits.az_min_deg = -10.0f;
  limits.az_max_deg = 10.0f;
  limits.el_min_deg = 5.0f;
  limits.el_max_deg = -5.0f;
  const auto pattern = signal::pipeline::BuildScheduledScanPattern(
      limits, 2.0f, 2.0f, oneq::foundation::ScanStartPosition::kLeftTop,
      oneq::foundation::ScanSequence::kAzimuthFirst);
  EXPECT_TRUE(pattern.empty());
}

TEST(ScanScheduleResolverTest, BuildPatternRejectsNonFiniteLimits) {
  config::AzimuthElevationLimitsDeg limits;
  limits.az_min_deg = std::numeric_limits<float>::quiet_NaN();
  limits.az_max_deg = 10.0f;
  limits.el_min_deg = -5.0f;
  limits.el_max_deg = 5.0f;
  const auto pattern = signal::pipeline::BuildScheduledScanPattern(
      limits, 2.0f, 2.0f, oneq::foundation::ScanStartPosition::kLeftTop,
      oneq::foundation::ScanSequence::kAzimuthFirst);
  EXPECT_TRUE(pattern.empty());
}

TEST(ScanScheduleResolverTest, BuildPatternRejectsNonPositiveStep) {
  config::AzimuthElevationLimitsDeg limits;
  limits.az_min_deg = -10.0f;
  limits.az_max_deg = 10.0f;
  limits.el_min_deg = -5.0f;
  limits.el_max_deg = 5.0f;
  EXPECT_TRUE(signal::pipeline::BuildScheduledScanPattern(
                  limits, 0.0f, 2.0f, oneq::foundation::ScanStartPosition::kLeftTop,
                  oneq::foundation::ScanSequence::kAzimuthFirst)
                  .empty());
  EXPECT_TRUE(signal::pipeline::BuildScheduledScanPattern(
                  limits, 2.0f, -1.0f, oneq::foundation::ScanStartPosition::kLeftTop,
                  oneq::foundation::ScanSequence::kAzimuthFirst)
                  .empty());
}

TEST(ScanScheduleResolverTest, ResolveFiniteScanCenterReturnsZeroForNan) {
  config::ArOrientationConfig orientation;
  orientation.scan_center_deg.az_deg = std::numeric_limits<float>::quiet_NaN();
  orientation.scan_center_deg.el_deg = std::numeric_limits<float>::quiet_NaN();
  const config::AzimuthElevationDeg center = signal::pipeline::ResolveFiniteScanCenter(orientation);
  EXPECT_FLOAT_EQ(center.az_deg, 0.0f);
  EXPECT_FLOAT_EQ(center.el_deg, 0.0f);
}

TEST(ScanScheduleResolverTest, ResolveFiniteScanCenterReturnsValidCenter) {
  config::ArOrientationConfig orientation;
  orientation.scan_center_deg.az_deg = 15.0f;
  orientation.scan_center_deg.el_deg = -5.0f;
  const config::AzimuthElevationDeg center = signal::pipeline::ResolveFiniteScanCenter(orientation);
  EXPECT_FLOAT_EQ(center.az_deg, 15.0f);
  EXPECT_FLOAT_EQ(center.el_deg, -5.0f);
}

TEST(ScanScheduleResolverTest, ApplyScanScheduleHandlesNullConfig) {
  signal::pipeline::ApplyScanScheduleToRuntimeConfig(1U, nullptr);
  SUCCEED();
}

TEST(ScanScheduleResolverTest, ApplyScanScheduleUsesOrientationBeamwidthFallback) {
  ExecutionConfig runtime_config =
      config::mapping::MapSessionToExecution(MakeDetectionFocusedConfig());
  runtime_config.detection.beam_control.pointing.nominal_beamwidth_deg.commanded_az_beamwidth_deg =
      0.0f;
  runtime_config.detection.beam_control.pointing.nominal_beamwidth_deg.commanded_el_beamwidth_deg =
      0.0f;
  runtime_config.detection.orientation.commanded_beamwidth_enabled = true;
  runtime_config.detection.orientation.commanded_beamwidth_deg.commanded_az_beamwidth_deg = 4.0f;
  runtime_config.detection.orientation.commanded_beamwidth_deg.commanded_el_beamwidth_deg = 2.0f;
  signal::pipeline::ApplyScanScheduleToRuntimeConfig(1U, &runtime_config);
  SUCCEED();
}

TEST(ScanScheduleResolverTest, ApplyScanScheduleUsesAntennaFallback) {
  ExecutionConfig runtime_config =
      config::mapping::MapSessionToExecution(MakeDetectionFocusedConfig());
  runtime_config.detection.beam_control.pointing.nominal_beamwidth_deg.commanded_az_beamwidth_deg =
      0.0f;
  runtime_config.detection.beam_control.pointing.nominal_beamwidth_deg.commanded_el_beamwidth_deg =
      0.0f;
  runtime_config.detection.orientation.commanded_beamwidth_enabled = false;
  runtime_config.detection.engineering.antenna.nominal_az_beamwidth_deg = 3.0f;
  runtime_config.detection.engineering.antenna.nominal_el_beamwidth_deg = 1.5f;
  signal::pipeline::ApplyScanScheduleToRuntimeConfig(1U, &runtime_config);
  SUCCEED();
}

}  // namespace
}  // namespace tests
}  // namespace airborne_radar
