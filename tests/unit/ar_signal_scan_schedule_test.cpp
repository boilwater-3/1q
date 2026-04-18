// Copyright 2026. All Rights Reserved.
//
// @file signal_scan_schedule_test.cpp
// @brief 验证机载雷达扫描调度语义与逐周期行为。

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include "1q/airborne_radar/model/TargetFeatureUtils.h"
#include "1q/airborne_radar/config/PipelineConfig.h"
#include "1q/airborne_radar/config/presets/PipelineConfigPresets.h"
#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/signal/detection/BeamControlResolver.h"
#include "airborne_radar/signal/detection/BeamwidthResolution.h"
#include "airborne_radar/signal/detection/SignalDetector.h"
#include "airborne_radar/signal/detection/TargetLookResolver.h"
#include "airborne_radar/signal/pipeline/assembly/RuntimeAssemblySupport.h"
#include "airborne_radar/signal/pipeline/config/InternalPipelineConfig.h"
#include "airborne_radar/signal/pipeline/core/CycleExecutor.h"
#include "airborne_radar/signal/pipeline/core/SignalPipeline.h"
#include "airborne_radar/signal/pipeline/core/ScanScheduleResolver.h"

namespace airborne_radar {
namespace tests {
namespace {

bool AlmostSamePoint(const model::AzimuthElevationDeg& lhs,
                     const model::AzimuthElevationDeg& rhs) {
  return std::fabs(lhs.az_deg - rhs.az_deg) <= 1.0e-4f &&
         std::fabs(lhs.el_deg - rhs.el_deg) <= 1.0e-4f;
}

std::size_t CountUniqueScheduledPoints(const model::RadarOrientationConfig& orientation,
                                       const signal::detection::EffectiveBeamwidthDeg& beamwidth,
                                       std::uint32_t cycle_count) {
  std::vector<model::AzimuthElevationDeg> unique_points;
  for (std::uint32_t cycle = 1U; cycle <= cycle_count; ++cycle) {
    const model::AzimuthElevationDeg pointing =
        signal::pipeline::core::internal::ResolveScheduledBeamPointing(orientation, beamwidth, cycle);
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

environment::EnvironmentCycleContext MakeEnvironmentCycle(std::uint32_t cycle_index) {
  environment::EnvironmentCycleContext cycle;
  cycle.cycle_index = cycle_index;
  cycle.dt_sec = 1.0f;
  return cycle;
}

environment::EnvironmentSnapshot MakeEnvironmentSnapshot(std::uint32_t cycle_index) {
  environment::EnvironmentSnapshot snapshot;
  snapshot.cycle_dt_sec = MakeEnvironmentCycle(cycle_index).dt_sec;
  return snapshot;
}

template <typename PipelineType>
extension::SignalCycleResult RunPipelineCycle(PipelineType* pipeline,
                                              const model::TargetFeatureList& input_state,
                                              environment::EnvironmentService* environment_service,
                                              std::uint32_t cycle_index) {
  environment_service->BeginCycle(MakeEnvironmentCycle(cycle_index));
  return pipeline->RunCycle(input_state, *environment_service);
}

class NonAutoLifecycleManager final : public signal::tracking::ITrackLifecycleManager {
 public:
  void Update(const signal::tracking::CycleContext&,
              const std::vector<signal::tracking::TrackMeasurement>&) override {}

  model::TargetFeatureList BuildFeatureSnapshot() const override { return model::TargetFeatureList(); }

  model::DecisionTrackSnapshotList BuildDecisionSnapshot() const override {
    return model::DecisionTrackSnapshotList();
  }

  model::DecisionInputFrame BuildDecisionFrame(std::uint32_t, std::uint64_t,
                                               bool) const override {
    return model::DecisionInputFrame();
  }

  std::vector<signal::tracking::AssociationTrackSeed> BuildAssociationSeeds() const override {
    return std::vector<signal::tracking::AssociationTrackSeed>();
  }
};

signal::pipeline::internal::CycleExecutionRuntime BuildMinimalValidRuntime(
    const config::PipelineConfig& base_config,
    const signal::pipeline::internal::InternalPipelineConfig& internal_config,
    const extension::control::RadarControlProfile& control_profile,
    signal::association::DataAssociationEngine* association_engine,
    signal::tracking::TrackFilter* track_filter,
    signal::tracking::ITrackLifecycleManager* lifecycle_manager) {
  static const std::vector<signal::tracking::AssociationTrackSeed> kEmptyAssociationSeeds;
  return signal::pipeline::internal::CycleExecutionRuntime(
      base_config, internal_config, control_profile, *association_engine, *track_filter,
      *lifecycle_manager, nullptr, kEmptyAssociationSeeds, false);
}

TEST(ScanScheduleResolverTest, StartPositionControlsFirstBeamQuadrant) {
  model::AzimuthElevationLimitsDeg limits;
  limits.az_min_deg = -10.0f;
  limits.az_max_deg = 10.0f;
  limits.el_min_deg = -5.0f;
  limits.el_max_deg = 5.0f;

  const std::vector<model::AzimuthElevationDeg> left_top_pattern =
      signal::pipeline::core::internal::BuildScheduledScanPattern(
          limits, 10.0f, 5.0f, oneq::foundation::ScanStartPosition::kLeftTop,
          oneq::foundation::ScanSequence::kAzimuthFirst);
  const std::vector<model::AzimuthElevationDeg> right_top_pattern =
      signal::pipeline::core::internal::BuildScheduledScanPattern(
          limits, 10.0f, 5.0f, oneq::foundation::ScanStartPosition::kRightTop,
          oneq::foundation::ScanSequence::kAzimuthFirst);
  const std::vector<model::AzimuthElevationDeg> right_bottom_pattern =
      signal::pipeline::core::internal::BuildScheduledScanPattern(
          limits, 10.0f, 5.0f, oneq::foundation::ScanStartPosition::kRightBottom,
          oneq::foundation::ScanSequence::kAzimuthFirst);
  const std::vector<model::AzimuthElevationDeg> left_bottom_pattern =
      signal::pipeline::core::internal::BuildScheduledScanPattern(
          limits, 10.0f, 5.0f, oneq::foundation::ScanStartPosition::kLeftBottom,
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

TEST(CycleExecutorTest, ValidRuntimeProducesInputAlignedStageBuffers) {
  config::PipelineConfig base_config;
  base_config = config::presets::MakeDetectionMissionPipelineConfig();
  const signal::pipeline::internal::InternalPipelineConfig internal_config =
      signal::pipeline::internal::BuildInternalPipelineConfig(base_config);
  const extension::control::RadarControlProfile control_profile{};

  signal::association::DataAssociationEngine association_engine;
  signal::tracking::TrackFilter track_filter;
  std::unique_ptr<signal::tracking::ITrackLifecycleManager> lifecycle_manager =
      signal::pipeline::assembly::internal::CreateAutoLifecycleManagerForRuntimeConfig(
          base_config, internal_config);
  ASSERT_TRUE(lifecycle_manager != nullptr);
  const signal::pipeline::internal::CycleExecutionRuntime runtime = BuildMinimalValidRuntime(
      base_config, internal_config, control_profile, &association_engine, &track_filter,
      lifecycle_manager.get());

  model::TargetFeature target_a(1000.0f, 0.0f, 0.0f, 2.0f);
  target_a.has_cartesian_position = true;
  target_a.position_x = 1000.0f;
  target_a.range_m = 1000.0f;
  model::TargetFeature target_b(1500.0f, 0.0f, 0.0f, 3.0f);
  target_b.has_cartesian_position = true;
  target_b.position_x = 1500.0f;
  target_b.range_m = 1500.0f;
  const model::TargetFeatureList input_state{target_a, target_b};

  environment::EnvironmentService environment_service;
  signal::pipeline::internal::CycleExecutionScratch scratch;
  EXPECT_TRUE(signal::pipeline::internal::ExecuteCycle(input_state, MakeEnvironmentSnapshot(3U),
                                                       3U, 9U, runtime, scratch));

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
  config::PipelineConfig base_config;
  const signal::pipeline::internal::InternalPipelineConfig internal_config =
      signal::pipeline::internal::BuildInternalPipelineConfig(base_config);
  const extension::control::RadarControlProfile control_profile{};

  signal::association::DataAssociationEngine association_engine;
  signal::tracking::TrackFilter track_filter;
  std::unique_ptr<signal::tracking::ITrackLifecycleManager> lifecycle_manager =
      signal::pipeline::assembly::internal::CreateAutoLifecycleManagerForRuntimeConfig(
          base_config, internal_config);
  ASSERT_TRUE(lifecycle_manager != nullptr);
  const signal::pipeline::internal::CycleExecutionRuntime runtime = BuildMinimalValidRuntime(
      base_config, internal_config, control_profile, &association_engine, &track_filter,
      lifecycle_manager.get());

  const model::TargetFeatureList input_state;
  signal::pipeline::internal::CycleExecutionScratch scratch;
  EXPECT_TRUE(signal::pipeline::internal::ExecuteCycle(input_state, MakeEnvironmentSnapshot(1U),
                                                       1U, 1U, runtime, scratch));

  EXPECT_TRUE(scratch.output_state.empty());
  EXPECT_TRUE(scratch.track_measurements.empty());
  EXPECT_TRUE(scratch.detection_succeeded.empty());
  EXPECT_TRUE(scratch.association_keys.empty());
  EXPECT_TRUE(scratch.measurement_covariances.empty());
}

TEST(CycleExecutorTest, NonAutoLifecycleManagerCausesRuntimeSyncFailure) {
  config::PipelineConfig base_config;
  const signal::pipeline::internal::InternalPipelineConfig internal_config =
      signal::pipeline::internal::BuildInternalPipelineConfig(base_config);
  const extension::control::RadarControlProfile control_profile{};

  signal::association::DataAssociationEngine association_engine;
  signal::tracking::TrackFilter track_filter;
  NonAutoLifecycleManager lifecycle_manager;
  const signal::pipeline::internal::CycleExecutionRuntime runtime = BuildMinimalValidRuntime(
      base_config, internal_config, control_profile, &association_engine, &track_filter,
      &lifecycle_manager);

  model::TargetFeature target(1000.0f, 0.0f, 0.0f, 2.0f);
  target.has_cartesian_position = true;
  target.position_x = 1000.0f;
  target.range_m = 1000.0f;

  signal::pipeline::internal::CycleExecutionScratch scratch;
  EXPECT_FALSE(signal::pipeline::internal::ExecuteCycle(
      model::TargetFeatureList{target}, MakeEnvironmentSnapshot(1U), 1U, 1U, runtime, scratch));
  EXPECT_TRUE(scratch.track_measurements.empty());
  EXPECT_EQ(scratch.decision_frame.cycle_index, 0U);
}

TEST(ScanScheduleResolverTest, SequenceControlsFastScanAxisWithSerpentine) {
  model::AzimuthElevationLimitsDeg limits;
  limits.az_min_deg = -10.0f;
  limits.az_max_deg = 10.0f;
  limits.el_min_deg = -5.0f;
  limits.el_max_deg = 5.0f;

  const std::vector<model::AzimuthElevationDeg> azimuth_first_pattern =
      signal::pipeline::core::internal::BuildScheduledScanPattern(
          limits, 10.0f, 5.0f, oneq::foundation::ScanStartPosition::kLeftTop,
          oneq::foundation::ScanSequence::kAzimuthFirst);
  const std::vector<model::AzimuthElevationDeg> elevation_first_pattern =
      signal::pipeline::core::internal::BuildScheduledScanPattern(
          limits, 10.0f, 5.0f, oneq::foundation::ScanStartPosition::kLeftTop,
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
  model::RadarOrientationConfig orientation;
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

  const model::AzimuthElevationDeg pointing =
      signal::pipeline::core::internal::ResolveScheduledBeamPointing(orientation, invalid_beamwidth, 3U);
  EXPECT_FLOAT_EQ(pointing.az_deg, 30.0f);
  EXPECT_FLOAT_EQ(pointing.el_deg, 10.0f);

  const model::AzimuthElevationDeg dwell_center =
      signal::pipeline::core::internal::ResolveScheduledDwellCenter(orientation, invalid_beamwidth, 3U);
  EXPECT_FLOAT_EQ(dwell_center.az_deg, -50.0f);
  EXPECT_FLOAT_EQ(dwell_center.el_deg, -30.0f);
}

TEST(ScanScheduleResolverTest, FirstCycleMapsToFirstBeamIndex) {
  model::RadarOrientationConfig orientation;
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

  const model::AzimuthElevationDeg cycle_1 =
      signal::pipeline::core::internal::ResolveScheduledBeamPointing(orientation, beamwidth, 1U);
  const model::AzimuthElevationDeg cycle_2 =
      signal::pipeline::core::internal::ResolveScheduledBeamPointing(orientation, beamwidth, 2U);
  EXPECT_FLOAT_EQ(cycle_1.az_deg, -60.0f);
  EXPECT_FLOAT_EQ(cycle_2.az_deg, 60.0f);
}

TEST(ScanScheduleResolverTest, StbyParksAtClampedBoresightWithoutCycleAdvance) {
  model::RadarOrientationConfig orientation;
  orientation.work_sub_mode = model::RadarWorkSubMode::kStby;
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

  const model::AzimuthElevationDeg cycle_1 =
      signal::pipeline::core::internal::ResolveScheduledBeamPointing(orientation, beamwidth, 1U);
  const model::AzimuthElevationDeg cycle_9 =
      signal::pipeline::core::internal::ResolveScheduledBeamPointing(orientation, beamwidth, 9U);
  EXPECT_TRUE(AlmostSamePoint(cycle_1, cycle_9));
  EXPECT_FLOAT_EQ(cycle_1.az_deg, 5.0f);
  EXPECT_FLOAT_EQ(cycle_1.el_deg, 0.0f);

  const model::AzimuthElevationDeg dwell =
      signal::pipeline::core::internal::ResolveScheduledDwellCenter(orientation, beamwidth, 5U);
  EXPECT_FLOAT_EQ(dwell.az_deg, cycle_1.az_deg - orientation.scan_center_deg.az_deg);
  EXPECT_FLOAT_EQ(dwell.el_deg, cycle_1.el_deg - orientation.scan_center_deg.el_deg);
}

TEST(ScanScheduleResolverTest, SttFixesAtScanCenterAndKeepsZeroDwell) {
  model::RadarOrientationConfig orientation;
  orientation.work_sub_mode = model::RadarWorkSubMode::kStt;
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

  const model::AzimuthElevationDeg cycle_1 =
      signal::pipeline::core::internal::ResolveScheduledBeamPointing(orientation, beamwidth, 1U);
  const model::AzimuthElevationDeg cycle_7 =
      signal::pipeline::core::internal::ResolveScheduledBeamPointing(orientation, beamwidth, 7U);
  EXPECT_TRUE(AlmostSamePoint(cycle_1, cycle_7));
  EXPECT_FLOAT_EQ(cycle_1.az_deg, orientation.scan_center_deg.az_deg);
  EXPECT_FLOAT_EQ(cycle_1.el_deg, orientation.scan_center_deg.el_deg);

  const model::AzimuthElevationDeg dwell =
      signal::pipeline::core::internal::ResolveScheduledDwellCenter(orientation, beamwidth, 7U);
  EXPECT_FLOAT_EQ(dwell.az_deg, 0.0f);
  EXPECT_FLOAT_EQ(dwell.el_deg, 0.0f);
}

TEST(ScanScheduleResolverTest, TasIsDenserThanTwsAndKeepsSerpentineSemantics) {
  model::RadarOrientationConfig tws_orientation;
  tws_orientation.work_sub_mode = model::RadarWorkSubMode::kTws;
  tws_orientation.scan_start_position = oneq::foundation::ScanStartPosition::kLeftTop;
  tws_orientation.scan_sequence = oneq::foundation::ScanSequence::kAzimuthFirst;
  tws_orientation.mechanical_scan_limits_deg.az_min_deg = -20.0f;
  tws_orientation.mechanical_scan_limits_deg.az_max_deg = 20.0f;
  tws_orientation.mechanical_scan_limits_deg.el_min_deg = -10.0f;
  tws_orientation.mechanical_scan_limits_deg.el_max_deg = 10.0f;
  tws_orientation.electronic_scan_limits_deg = tws_orientation.mechanical_scan_limits_deg;
  model::RadarOrientationConfig tas_orientation = tws_orientation;
  tas_orientation.work_sub_mode = model::RadarWorkSubMode::kTas;

  signal::detection::EffectiveBeamwidthDeg beamwidth;
  beamwidth.az_beamwidth_deg = 20.0f;
  beamwidth.el_beamwidth_deg = 10.0f;

  const model::AzimuthElevationDeg tws_first =
      signal::pipeline::core::internal::ResolveScheduledBeamPointing(tws_orientation, beamwidth, 1U);
  const model::AzimuthElevationDeg tws_second =
      signal::pipeline::core::internal::ResolveScheduledBeamPointing(tws_orientation, beamwidth, 2U);
  const model::AzimuthElevationDeg tas_first =
      signal::pipeline::core::internal::ResolveScheduledBeamPointing(tas_orientation, beamwidth, 1U);
  const model::AzimuthElevationDeg tas_second =
      signal::pipeline::core::internal::ResolveScheduledBeamPointing(tas_orientation, beamwidth, 2U);
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
  config::PipelineConfig config;
  config.expert.detection.enable_physics_detection = true;
  config.expert.detection.pulse_count = 16;
  config.expert.detection.detection_policy.cfar_pfa = 2.0e-6f;
  config.expert.detection.detection_policy.min_snr_db = -12.0f;
  config.expert.detection.min_detection_margin_db = -100.0f;
  config.expert.detection.antenna.pattern.model_type =
      config::expert::AntennaPatternModelType::kParabolicMainLobe;
  config.expert.detection.antenna.pattern.max_sidelobe_level_db = -18.0f;
  config.expert.detection.antenna.pattern.max_scan_loss_db = 8.0f;

    config::engineering::DetectionConfig engineering_detection =
      config::internal::ResolveDetectionEngineering(config.expert.detection);
  engineering_detection.pulse_count = 4096;
  engineering_detection.detection_policy.cfar_pfa = 0.999999f;
  engineering_detection.detection_policy.min_snr_db = -50.0f;
  engineering_detection.antenna.nominal_az_beamwidth_deg = 120.0f;
  engineering_detection.antenna.nominal_el_beamwidth_deg = 10.0f;
  engineering_detection.antenna.enable_directional_pattern = true;
  engineering_detection.antenna.pattern.max_sidelobe_level_db = -80.0f;
  engineering_detection.antenna.pattern.backlobe_level_db = -80.0f;

  model::RadarOrientationConfig& orientation = config.orientation;
  orientation.scan_center_deg.az_deg = 0.0f;
  orientation.scan_center_deg.el_deg = 0.0f;
  orientation.mechanical_scan_limits_deg.az_min_deg = -60.0f;
  orientation.mechanical_scan_limits_deg.az_max_deg = 60.0f;
  orientation.mechanical_scan_limits_deg.el_min_deg = 0.0f;
  orientation.mechanical_scan_limits_deg.el_max_deg = 0.0f;
  orientation.electronic_scan_limits_deg = orientation.mechanical_scan_limits_deg;
  orientation.scan_start_position = oneq::foundation::ScanStartPosition::kLeftTop;
  orientation.scan_sequence = oneq::foundation::ScanSequence::kAzimuthFirst;

  signal::pipeline::SignalPipeline signal_pipeline(
      config);
  environment::EnvironmentModelConfig environment_config;
  environment::EnvironmentService environment_service(environment_config);

  model::TargetFeature target =
      model::MakeAirTarget(2026U, 120.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10000.0f);
  const model::TargetFeatureList targets(1U, target);

  const signal::detection::TargetLookAnglesDeg look_angles =
      signal::detection::TargetLookResolver::Resolve(target);
  ASSERT_TRUE(look_angles.has_look_angles);
  const signal::detection::EffectiveBeamwidthDeg effective_beamwidth =
      signal::detection::ResolveEffectiveBeamwidth(engineering_detection.antenna,
                                                   orientation);

    const model::AzimuthElevationDeg cycle_1_dwell_center =
      signal::pipeline::core::internal::ResolveScheduledDwellCenter(orientation,
                                     effective_beamwidth, 1U);
    const model::AzimuthElevationDeg cycle_2_dwell_center =
      signal::pipeline::core::internal::ResolveScheduledDwellCenter(orientation,
                                     effective_beamwidth, 2U);
  const model::PlatformAttitudeDeg platform_attitude_deg{};
  const signal::detection::ResolvedBeamState cycle_1_beam =
      signal::detection::BeamControlResolver::Resolve(
        engineering_detection.antenna, orientation, platform_attitude_deg, look_angles,
        cycle_1_dwell_center);
  const signal::detection::ResolvedBeamState cycle_2_beam =
      signal::detection::BeamControlResolver::Resolve(
        engineering_detection.antenna, orientation, platform_attitude_deg, look_angles,
        cycle_2_dwell_center);

  signal::detection::TargetReturn target_return;
  target_return.rcs_m2 = target.current_track_rcs;
  target_return.range_m = target.range_m;
  target_return.swerling_type =
      static_cast<config::semantic::SwerlingModel>(target.target_swerling_type);
  signal::detection::EnvironmentState environment_state;
  signal::detection::SignalDetector detector(engineering_detection);
  const signal::detection::DetectionResult cycle_1_detection =
      detector.Detect(target_return, environment_state, cycle_1_beam.one_way_antenna_gain_db,
                      engineering_detection.pulse_count);
  const signal::detection::DetectionResult cycle_2_detection =
      detector.Detect(target_return, environment_state, cycle_2_beam.one_way_antenna_gain_db,
                      engineering_detection.pulse_count);
  ASSERT_GE(cycle_2_detection.snr_db, cycle_1_detection.snr_db);
  ASSERT_GE(cycle_2_detection.detection_prob, cycle_1_detection.detection_prob);

  std::size_t aligned_detected = 0U;
  std::size_t misaligned_detected = 0U;
  const std::size_t kCycleCount = 64U;
  for (std::size_t i = 0; i < kCycleCount; ++i) {
    const extension::SignalCycleResult cycle_result =
        RunPipelineCycle(&signal_pipeline, targets, &environment_service,
                         static_cast<std::uint32_t>(i + 1U));
    if ((i % 2U) == 0U) {
      misaligned_detected += cycle_result.association_quality_metrics.detection_count;
    } else {
      aligned_detected += cycle_result.association_quality_metrics.detection_count;
    }
  }

  EXPECT_GE(aligned_detected, misaligned_detected);
}

TEST(SignalPipelineScanScheduleTest, WorkSubModeSttReducesSweepCoverageComparedToTws) {
  config::PipelineConfig tws_config;
  tws_config.expert.detection.enable_physics_detection = true;
  tws_config.expert.detection.pulse_count = 16;
  tws_config.expert.detection.detection_policy.cfar_pfa = 2.0e-6f;
  tws_config.expert.detection.detection_policy.min_snr_db = -12.0f;
  tws_config.expert.detection.min_detection_margin_db = -100.0f;
  tws_config.expert.detection.antenna.pattern.model_type =
      config::expert::AntennaPatternModelType::kParabolicMainLobe;
  tws_config.expert.detection.antenna.pattern.max_sidelobe_level_db = -18.0f;
  tws_config.expert.detection.antenna.pattern.max_scan_loss_db = 8.0f;
  model::RadarOrientationConfig& tws_orientation = tws_config.orientation;
  tws_orientation.work_sub_mode = model::RadarWorkSubMode::kTws;
  tws_orientation.scan_center_deg.az_deg = -60.0f;
  tws_orientation.scan_center_deg.el_deg = 0.0f;
  tws_orientation.mechanical_scan_limits_deg.az_min_deg = -60.0f;
  tws_orientation.mechanical_scan_limits_deg.az_max_deg = 60.0f;
  tws_orientation.mechanical_scan_limits_deg.el_min_deg = 0.0f;
  tws_orientation.mechanical_scan_limits_deg.el_max_deg = 0.0f;
  tws_orientation.electronic_scan_limits_deg = tws_orientation.mechanical_scan_limits_deg;
  tws_orientation.scan_start_position = oneq::foundation::ScanStartPosition::kLeftTop;
  tws_orientation.scan_sequence = oneq::foundation::ScanSequence::kAzimuthFirst;

  config::PipelineConfig stt_config = tws_config;
  stt_config.orientation.work_sub_mode = model::RadarWorkSubMode::kStt;

  signal::pipeline::SignalPipeline tws_pipeline(
      tws_config);
  signal::pipeline::SignalPipeline stt_pipeline(
      stt_config);

  environment::EnvironmentModelConfig environment_config;
  environment::EnvironmentService environment_service(environment_config);

  model::TargetFeature target =
      model::MakeAirTarget(2026U, 120.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10000.0f);
  const model::TargetFeatureList targets(1U, target);

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

}  // namespace
}  // namespace tests
}  // namespace airborne_radar
