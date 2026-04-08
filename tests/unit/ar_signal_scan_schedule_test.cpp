// Copyright 2026. All Rights Reserved.
//
// @file signal_scan_schedule_test.cpp
// @brief 验证机载雷达扫描调度语义与逐周期行为。

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "1q/airborne_radar/model/TargetFeatureUtils.h"
#include "1q/airborne_radar/config/SignalPipelineConfig.h"
#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/signal/detection/BeamControlResolver.h"
#include "airborne_radar/signal/detection/BeamwidthResolution.h"
#include "airborne_radar/signal/detection/SignalDetector.h"
#include "airborne_radar/signal/detection/TargetLookResolver.h"
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

signal::pipeline::internal::CycleExecutionRuntime BuildMinimalValidRuntime(
    const config::SignalPipelineConfig& base_config,
    const signal::pipeline::internal::InternalSignalPipelineConfig& internal_config,
    const extension::control::RadarControlProfile& control_profile,
    signal::association::DataAssociationEngine* association_engine,
    signal::tracking::TrackFilter* track_filter) {
  signal::pipeline::internal::CycleExecutionRuntime runtime;
  runtime.base_config = &base_config;
  runtime.base_internal_config = &internal_config;
  runtime.control_profile = &control_profile;
  runtime.association_engine = association_engine;
  runtime.track_filter = track_filter;
  runtime.signal_detector = nullptr;
  runtime.auto_lifecycle_manager = nullptr;
  runtime.manual_association_seeds = nullptr;
  runtime.has_manual_association_seeds = false;
  return runtime;
}

TEST(ScanScheduleResolverTest, StartPositionControlsFirstBeamQuadrant) {
  model::AzimuthElevationLimitsDeg limits;
  limits.az_min_deg = -10.0f;
  limits.az_max_deg = 10.0f;
  limits.el_min_deg = -5.0f;
  limits.el_max_deg = 5.0f;

  const std::vector<model::AzimuthElevationDeg> left_top_pattern =
      signal::pipeline::core::internal::BuildScheduledScanPattern(
          limits, 10.0f, 5.0f, oneq::common::ScanStartPosition::kLeftTop,
          oneq::common::ScanSequence::kAzimuthFirst);
  const std::vector<model::AzimuthElevationDeg> right_top_pattern =
      signal::pipeline::core::internal::BuildScheduledScanPattern(
          limits, 10.0f, 5.0f, oneq::common::ScanStartPosition::kRightTop,
          oneq::common::ScanSequence::kAzimuthFirst);
  const std::vector<model::AzimuthElevationDeg> right_bottom_pattern =
      signal::pipeline::core::internal::BuildScheduledScanPattern(
          limits, 10.0f, 5.0f, oneq::common::ScanStartPosition::kRightBottom,
          oneq::common::ScanSequence::kAzimuthFirst);
  const std::vector<model::AzimuthElevationDeg> left_bottom_pattern =
      signal::pipeline::core::internal::BuildScheduledScanPattern(
          limits, 10.0f, 5.0f, oneq::common::ScanStartPosition::kLeftBottom,
          oneq::common::ScanSequence::kAzimuthFirst);

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

TEST(CycleExecutorTest, InvalidRuntimeAndNullScratchReturnSafely) {
  const model::TargetFeatureList input_state;
  environment::EnvironmentService environment_service;
  signal::pipeline::internal::CycleExecutionRuntime runtime;

  signal::pipeline::internal::ExecuteCycle(input_state, environment_service, 1U, 1U, runtime, nullptr);

  signal::pipeline::internal::CycleExecutionScratch scratch;
  signal::pipeline::internal::ExecuteCycle(input_state, environment_service, 1U, 1U, runtime,
                                           &scratch);

  SUCCEED();
}

TEST(CycleExecutorTest, ValidRuntimeProducesInputAlignedStageBuffers) {
  config::SignalPipelineConfig base_config;
  base_config.detection.min_detection_margin_db = -100.0f;
  const signal::pipeline::internal::InternalSignalPipelineConfig internal_config =
      signal::pipeline::internal::BuildInternalSignalPipelineConfig(base_config);
  const extension::control::RadarControlProfile control_profile{};

  signal::association::DataAssociationEngine association_engine;
  signal::tracking::TrackFilter track_filter;
  const signal::pipeline::internal::CycleExecutionRuntime runtime = BuildMinimalValidRuntime(
      base_config, internal_config, control_profile, &association_engine, &track_filter);

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
  signal::pipeline::internal::ExecuteCycle(input_state, environment_service, 3U, 9U, runtime,
                                           &scratch);

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
  config::SignalPipelineConfig base_config;
  const signal::pipeline::internal::InternalSignalPipelineConfig internal_config =
      signal::pipeline::internal::BuildInternalSignalPipelineConfig(base_config);
  const extension::control::RadarControlProfile control_profile{};

  signal::association::DataAssociationEngine association_engine;
  signal::tracking::TrackFilter track_filter;
  const signal::pipeline::internal::CycleExecutionRuntime runtime = BuildMinimalValidRuntime(
      base_config, internal_config, control_profile, &association_engine, &track_filter);

  const model::TargetFeatureList input_state;
  environment::EnvironmentService environment_service;
  signal::pipeline::internal::CycleExecutionScratch scratch;
  signal::pipeline::internal::ExecuteCycle(input_state, environment_service, 1U, 1U, runtime,
                                           &scratch);

  EXPECT_TRUE(scratch.output_state.empty());
  EXPECT_TRUE(scratch.track_measurements.empty());
  EXPECT_TRUE(scratch.detection_succeeded.empty());
  EXPECT_TRUE(scratch.association_keys.empty());
  EXPECT_TRUE(scratch.measurement_covariances.empty());
}

TEST(ScanScheduleResolverTest, SequenceControlsFastScanAxisWithSerpentine) {
  model::AzimuthElevationLimitsDeg limits;
  limits.az_min_deg = -10.0f;
  limits.az_max_deg = 10.0f;
  limits.el_min_deg = -5.0f;
  limits.el_max_deg = 5.0f;

  const std::vector<model::AzimuthElevationDeg> azimuth_first_pattern =
      signal::pipeline::core::internal::BuildScheduledScanPattern(
          limits, 10.0f, 5.0f, oneq::common::ScanStartPosition::kLeftTop,
          oneq::common::ScanSequence::kAzimuthFirst);
  const std::vector<model::AzimuthElevationDeg> elevation_first_pattern =
      signal::pipeline::core::internal::BuildScheduledScanPattern(
          limits, 10.0f, 5.0f, oneq::common::ScanStartPosition::kLeftTop,
          oneq::common::ScanSequence::kElevationFirst);

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
  orientation.scan_start_position = oneq::common::ScanStartPosition::kLeftTop;
  orientation.scan_sequence = oneq::common::ScanSequence::kAzimuthFirst;

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
  tws_orientation.scan_start_position = oneq::common::ScanStartPosition::kLeftTop;
  tws_orientation.scan_sequence = oneq::common::ScanSequence::kAzimuthFirst;
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
  config::SignalPipelineConfig config;
  config.detection.enable_physics_detection = true;
  config.detection.pulse_count = 4096;
  config.detection.detection_policy.cfar_pfa = 0.999999f;
  config.detection.detection_policy.min_snr_db = -50.0f;
  config.detection.antenna.nominal_az_beamwidth_deg = 120.0f;
  config.detection.antenna.nominal_el_beamwidth_deg = 10.0f;
  config.detection.antenna.enable_directional_pattern = true;
  config.detection.antenna.pattern.max_sidelobe_level_db = -80.0f;
  config.detection.antenna.pattern.backlobe_level_db = -80.0f;

  model::RadarOrientationConfig& orientation = config.beam_control.radar_orientation;
  orientation.scan_center_deg.az_deg = 0.0f;
  orientation.scan_center_deg.el_deg = 0.0f;
  orientation.mechanical_scan_limits_deg.az_min_deg = -60.0f;
  orientation.mechanical_scan_limits_deg.az_max_deg = 60.0f;
  orientation.mechanical_scan_limits_deg.el_min_deg = 0.0f;
  orientation.mechanical_scan_limits_deg.el_max_deg = 0.0f;
  orientation.electronic_scan_limits_deg = orientation.mechanical_scan_limits_deg;
  orientation.scan_start_position = oneq::common::ScanStartPosition::kLeftTop;
  orientation.scan_sequence = oneq::common::ScanSequence::kAzimuthFirst;

  signal::pipeline::SignalPipeline signal_pipeline(
      config);
  environment::EnvironmentModelConfig environment_config;
  environment_config.base_propagation_loss_db = 0.0f;
  environment_config.atmospheric_attenuation_db = 0.0f;
  environment_config.terrain_reflection_db = 0.0f;
  environment_config.clutter_power_db = 0.0f;
  environment::EnvironmentService environment_service(environment_config);

  model::TargetFeature target =
      model::MakeAirTarget(2026U, 500.0f, 866.025f, 0.0f, 0.0f, 0.0f, 0.0f, 100.0f);
  const model::TargetFeatureList targets(1U, target);

  const signal::detection::TargetLookAnglesDeg look_angles =
      signal::detection::TargetLookResolver::Resolve(target);
  ASSERT_TRUE(look_angles.has_look_angles);
  const signal::detection::EffectiveBeamwidthDeg effective_beamwidth =
      signal::detection::ResolveEffectiveBeamwidth(config.detection.antenna,
                                                   orientation);

  model::RadarOrientationConfig cycle_1_orientation = orientation;
  model::RadarOrientationConfig cycle_2_orientation = orientation;
  cycle_1_orientation.dwell_center_deg = signal::pipeline::core::internal::ResolveScheduledDwellCenter(
      cycle_1_orientation, effective_beamwidth, 1U);
  cycle_2_orientation.dwell_center_deg = signal::pipeline::core::internal::ResolveScheduledDwellCenter(
      cycle_2_orientation, effective_beamwidth, 2U);
  const signal::detection::ResolvedBeamState cycle_1_beam =
      signal::detection::BeamControlResolver::Resolve(
          config.detection.antenna, cycle_1_orientation,
          config.beam_control.platform_attitude_deg, look_angles);
  const signal::detection::ResolvedBeamState cycle_2_beam =
      signal::detection::BeamControlResolver::Resolve(
          config.detection.antenna, cycle_2_orientation,
          config.beam_control.platform_attitude_deg, look_angles);

  signal::detection::TargetReturn target_return;
  target_return.rcs_m2 = target.current_track_rcs;
  target_return.range_m = target.range_m;
  target_return.swerling_type =
      static_cast<config::SwerlingModel>(target.target_swerling_type);
  signal::detection::EnvironmentState environment_state;
  signal::detection::SignalDetector detector(config.detection);
  const signal::detection::DetectionResult cycle_1_detection =
      detector.Detect(target_return, environment_state, cycle_1_beam.one_way_antenna_gain_db,
                      config.detection.pulse_count);
  const signal::detection::DetectionResult cycle_2_detection =
      detector.Detect(target_return, environment_state, cycle_2_beam.one_way_antenna_gain_db,
                      config.detection.pulse_count);
  ASSERT_GT(cycle_2_detection.snr_db, cycle_1_detection.snr_db);
  ASSERT_GT(cycle_2_detection.detection_prob, cycle_1_detection.detection_prob);

  std::size_t aligned_detected = 0U;
  std::size_t misaligned_detected = 0U;
  const std::size_t kCycleCount = 64U;
  for (std::size_t i = 0; i < kCycleCount; ++i) {
    const extension::SignalCycleResult cycle_result =
        signal_pipeline.RunCycle(targets, environment_service);
    if ((i % 2U) == 0U) {
      misaligned_detected += cycle_result.association_quality_metrics.detection_count;
    } else {
      aligned_detected += cycle_result.association_quality_metrics.detection_count;
    }
  }

  EXPECT_GT(aligned_detected, 0U);
  EXPECT_GT(aligned_detected, misaligned_detected);
}

TEST(SignalPipelineScanScheduleTest, WorkSubModeSttReducesSweepCoverageComparedToTws) {
  config::SignalPipelineConfig tws_config;
  tws_config.detection.enable_physics_detection = true;
  tws_config.detection.pulse_count = 4096;
  tws_config.detection.detection_policy.cfar_pfa = 0.999999f;
  tws_config.detection.detection_policy.min_snr_db = -50.0f;
  tws_config.detection.antenna.nominal_az_beamwidth_deg = 120.0f;
  tws_config.detection.antenna.nominal_el_beamwidth_deg = 10.0f;
  tws_config.detection.antenna.enable_directional_pattern = true;
  tws_config.detection.antenna.pattern.max_sidelobe_level_db = -80.0f;
  tws_config.detection.antenna.pattern.backlobe_level_db = -80.0f;
  model::RadarOrientationConfig& tws_orientation =
      tws_config.beam_control.radar_orientation;
  tws_orientation.work_sub_mode = model::RadarWorkSubMode::kTws;
  tws_orientation.scan_center_deg.az_deg = -60.0f;
  tws_orientation.scan_center_deg.el_deg = 0.0f;
  tws_orientation.mechanical_scan_limits_deg.az_min_deg = -60.0f;
  tws_orientation.mechanical_scan_limits_deg.az_max_deg = 60.0f;
  tws_orientation.mechanical_scan_limits_deg.el_min_deg = 0.0f;
  tws_orientation.mechanical_scan_limits_deg.el_max_deg = 0.0f;
  tws_orientation.electronic_scan_limits_deg = tws_orientation.mechanical_scan_limits_deg;
  tws_orientation.scan_start_position = oneq::common::ScanStartPosition::kLeftTop;
  tws_orientation.scan_sequence = oneq::common::ScanSequence::kAzimuthFirst;

  config::SignalPipelineConfig stt_config = tws_config;
  stt_config.beam_control.radar_orientation.work_sub_mode = model::RadarWorkSubMode::kStt;

  signal::pipeline::SignalPipeline tws_pipeline(
      tws_config);
  signal::pipeline::SignalPipeline stt_pipeline(
      stt_config);

  environment::EnvironmentModelConfig environment_config;
  environment_config.base_propagation_loss_db = 0.0f;
  environment_config.atmospheric_attenuation_db = 0.0f;
  environment_config.terrain_reflection_db = 0.0f;
  environment_config.clutter_power_db = 0.0f;
  environment::EnvironmentService environment_service(environment_config);

  model::TargetFeature target =
      model::MakeAirTarget(2026U, 500.0f, 866.025f, 0.0f, 0.0f, 0.0f, 0.0f, 100.0f);
  const model::TargetFeatureList targets(1U, target);

  std::size_t tws_detected = 0U;
  std::size_t stt_detected = 0U;
  const std::size_t kCycleCount = 64U;
  for (std::size_t i = 0; i < kCycleCount; ++i) {
    tws_detected += tws_pipeline.RunCycle(targets, environment_service)
                        .association_quality_metrics.detection_count;
    stt_detected += stt_pipeline.RunCycle(targets, environment_service)
                        .association_quality_metrics.detection_count;
  }

  EXPECT_GT(tws_detected, stt_detected);
}

}  // namespace
}  // namespace tests
}  // namespace airborne_radar
