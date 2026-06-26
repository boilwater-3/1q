/**
 * @file esr_cycle_output_builder_test.cpp
 * @brief 验证 EsrCycleOutputBuilder 将内部 ESR 输出转换为外部 ECEF 方位线。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "1q/coordinate/position_transform.h"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfigBuilder.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInputBuilder.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleOutputBuilder.h"
#include "1q/electronic_surveillance_radar/session/EsrEmitterLifecycleRecorder.h"
#include "1q/electronic_surveillance_radar/session/EsrOutputDebugView.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"
#include "1q/electronic_surveillance_radar/session/EsrSessionFactory.h"

namespace {

namespace esr_config = ::electronic_surveillance_radar::config;
namespace esr_session = ::electronic_surveillance_radar::session;

constexpr double kPi = 3.14159265358979323846;

esr_session::EsrExternalPoseInput MakePlatformInput() {
  oneq::coordinate::LlaPositionDegM platform_lla;
  platform_lla.latitude_deg = 30.0;
  platform_lla.longitude_deg = 120.0;
  platform_lla.altitude_m = 6000.0;
  oneq::coordinate::EcefPositionM platform_ecef;
  EXPECT_TRUE(oneq::coordinate::TryLlaToEcef(platform_lla, &platform_ecef));

  esr_session::EsrExternalPoseInput platform;
  platform.platform_position_ecef_m = platform_ecef;
  platform.platform_attitude_deg.yaw_deg = 0.0;
  platform.platform_attitude_deg.pitch_deg = 0.0;
  platform.platform_attitude_deg.roll_deg = 0.0;
  return platform;
}

std::vector<esr_session::EsrExternalEmitterInput> MakeEmitters(std::size_t count) {
  std::vector<esr_session::EsrExternalEmitterInput> emitters;
  emitters.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    oneq::coordinate::LlaPositionDegM lla;
    lla.latitude_deg = 30.0 + static_cast<double>(i + 1U) * 0.0002;
    lla.longitude_deg = 120.0 + static_cast<double>(i % 4U) * 0.00015;
    lla.altitude_m = 6000.0 + static_cast<double>(i % 3U) * 20.0;
    oneq::coordinate::EcefPositionM ecef;
    EXPECT_TRUE(oneq::coordinate::TryLlaToEcef(lla, &ecef));

    esr_session::EsrExternalEmitterInput emitter;
    emitter.emitter_id = 1000U + static_cast<std::uint64_t>(i);
    emitter.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
    emitter.kinematics.position_ecef_m = ecef;
    emitter.kinematics.velocity_mps.x_mps = 8.0 + static_cast<double>(i % 3U);
    emitter.kinematics.velocity_mps.y_mps = -3.0 + static_cast<double>(i % 2U);
    emitter.carrier_hz = 9.0e9 + static_cast<double>(i) * 1.0e7;
    emitter.bandwidth_hz = 2.0e6;
    emitter.tx_power_w = 5.0e8;
    emitter.pulse_width_s = 1.0e-6;
    emitter.pri_s = 1.0e-4;
    emitter.is_emitting = true;
    emitters.push_back(emitter);
  }
  return emitters;
}

void AdvanceEmitters(double dt_sec, std::vector<esr_session::EsrExternalEmitterInput>* emitters) {
  ASSERT_NE(emitters, nullptr);
  for (std::size_t i = 0; i < emitters->size(); ++i) {
    esr_session::EsrExternalEmitterInput& emitter = (*emitters)[i];
    emitter.kinematics.position_ecef_m.x_m += emitter.kinematics.velocity_mps.x_mps * dt_sec;
    emitter.kinematics.position_ecef_m.y_m += emitter.kinematics.velocity_mps.y_mps * dt_sec;
    emitter.kinematics.position_ecef_m.z_m += emitter.kinematics.velocity_mps.z_mps * dt_sec;
  }
}

esr_config::EsrSessionConfig MakeConfig() {
  esr_config::EsrSessionConfig config =
      esr_config::EsrSessionConfigBuilder()
          .Detection()
          .WithMinDetectSnrDb(3.0f)
          .End()
          .Environment()
          .WithEnvironmentPreset(esr_config::EsrEnvironmentPreset::kStandard)
          .End()
          .Build();
  config.policy.detection.minimum_snr_db = -20.0f;
  config.policy.detection.enable_statistical_detection = false;
  config.mission.scan.use_explicit_scan_bounds = true;
  config.mission.scan.scan_start_az_deg = -180.0f;
  config.mission.scan.scan_end_az_deg = 180.0f;
  config.mission.scan.scan_start_el_deg = -90.0f;
  config.mission.scan.scan_end_el_deg = 90.0f;
  config.hardware.beam_az_width_deg = 180.0f;
  config.hardware.beam_el_width_deg = 120.0f;
  return config;
}

oneq::coordinate::Vector3d TruthBearingUnit(const esr_session::EsrExternalPoseInput& platform,
                                            const esr_session::EsrExternalEmitterInput& emitter) {
  oneq::coordinate::Vector3d bearing;
  bearing.x = emitter.kinematics.position_ecef_m.x_m - platform.platform_position_ecef_m.x_m;
  bearing.y = emitter.kinematics.position_ecef_m.y_m - platform.platform_position_ecef_m.y_m;
  bearing.z = emitter.kinematics.position_ecef_m.z_m - platform.platform_position_ecef_m.z_m;
  const double norm =
      std::sqrt(bearing.x * bearing.x + bearing.y * bearing.y + bearing.z * bearing.z);
  bearing.x /= norm;
  bearing.y /= norm;
  bearing.z /= norm;
  return bearing;
}

const esr_session::EsrExternalObservation* FindObservation(
    const esr_session::EsrExternalOutputFrame& frame, std::uint64_t observation_id) {
  for (std::size_t i = 0; i < frame.observations.size(); ++i) {
    if (frame.observations[i].observation_id == observation_id) {
      return &frame.observations[i];
    }
  }
  return nullptr;
}

const esr_session::EsrExternalEmitterInput* FindEmitter(
    const std::vector<esr_session::EsrExternalEmitterInput>& emitters, std::uint64_t id) {
  for (std::size_t i = 0; i < emitters.size(); ++i) {
    if (emitters[i].emitter_id == id) {
      return &emitters[i];
    }
  }
  return nullptr;
}

double Dot(const oneq::coordinate::Vector3d& lhs, const oneq::coordinate::Vector3d& rhs) {
  return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

}  // namespace

TEST(EsrCycleOutputBuilderTest, MultiCycleMovingEmittersKeepExternalBearingsNearTruth) {
  const esr_session::EsrExternalPoseInput platform = MakePlatformInput();
  std::vector<esr_session::EsrExternalEmitterInput> emitters = MakeEmitters(6U);
  esr_session::EsrSession session = esr_session::EsrSessionFactory::Create(MakeConfig());

  const std::size_t cycle_count = 30U;
  const float dt_sec = 1.0f;
  const double min_cosine = std::cos(8.0 * kPi / 180.0);
  for (std::size_t cycle = 0; cycle < cycle_count; ++cycle) {
    esr_session::EsrCycleInput input;
    ASSERT_TRUE(esr_session::EsrCycleInputBuilder::Build(platform, emitters, dt_sec, &input))
        << "cycle=" << cycle;
    input.cycle_index = static_cast<std::uint32_t>(cycle);

    const esr_session::EsrCycleResult result = session.StepWithResult(input);
    ASSERT_FALSE(result.has_validation_error) << "cycle=" << cycle;

    esr_session::EsrExternalOutputFrame external_frame;
    ASSERT_TRUE(
        esr_session::EsrCycleOutputBuilder::Build(platform, result.output_frame, &external_frame))
        << "cycle=" << cycle;

    for (std::size_t i = 0; i < result.output_frame.truth_evaluation_output.associations.size();
         ++i) {
      const ::electronic_surveillance_radar::session::TruthAssociationRecord& association =
          result.output_frame.truth_evaluation_output.associations[i];
      if (!association.matched) {
        continue;
      }
      const esr_session::EsrExternalObservation* observation =
          FindObservation(external_frame, association.observation_id);
      ASSERT_NE(observation, nullptr) << "cycle=" << cycle;
      const esr_session::EsrExternalEmitterInput* emitter =
          FindEmitter(emitters, association.truth_emitter_id);
      ASSERT_NE(emitter, nullptr) << "cycle=" << cycle;
      const oneq::coordinate::Vector3d truth = TruthBearingUnit(platform, *emitter);
      EXPECT_GT(Dot(observation->bearing_unit_ecef, truth), min_cosine)
          << "cycle=" << cycle << " emitter=" << association.truth_emitter_id;
    }

    AdvanceEmitters(dt_sec, &emitters);
  }
}

TEST(EsrCycleOutputBuilderTest, DebugViewMapsTruthAssociationsBackToNamedEmitters) {
  esr_session::EsrCycleInput input;
  input.cycle_index = 12U;
  esr_session::EsrSceneEmitter observed;
  observed.emitter_id = 101U;
  observed.emitter_name = "observed-emitter";
  observed.is_emitting = true;
  esr_session::EsrSceneEmitter silent;
  silent.emitter_id = 102U;
  silent.emitter_name = "silent-emitter";
  silent.is_emitting = false;
  esr_session::EsrSceneEmitter missed;
  missed.emitter_id = 103U;
  missed.emitter_name = "missed-emitter";
  missed.is_emitting = true;
  input.scene = {observed, silent, missed};

  esr_session::EsrCycleResult result;
  result.input_cycle_index = input.cycle_index;
  result.executed_this_cycle = true;
  result.output_frame.cycle_index = input.cycle_index;
  ::electronic_surveillance_radar::session::TruthAssociationRecord association;
  association.observation_id = 9001U;
  association.truth_emitter_id = 101U;
  association.matched = true;
  association.confidence = 0.75f;
  result.output_frame.truth_evaluation_output.associations.push_back(association);

  const esr_session::EsrOutputDebugView view =
      esr_session::EsrOutputDebugViewBuilder::Build(input, result);
  ASSERT_EQ(view.emitters.size(), 3U);
  EXPECT_EQ(view.emitters[0].status, esr_session::EsrDebugEmitterStatus::kObserved);
  EXPECT_EQ(view.emitters[0].emitter_name, "observed-emitter");
  EXPECT_EQ(view.emitters[0].observation_id, 9001U);
  EXPECT_EQ(view.emitters[1].status, esr_session::EsrDebugEmitterStatus::kNotEmitting);
  EXPECT_EQ(view.emitters[2].status, esr_session::EsrDebugEmitterStatus::kNotObserved);
}

TEST(EsrCycleOutputBuilderTest, LifecycleRecorderTracksObservedLostAndOptionalNotObserved) {
  esr_session::EsrCycleInput input;
  input.cycle_index = 30U;
  esr_session::EsrSceneEmitter tracked;
  tracked.emitter_id = 201U;
  tracked.emitter_name = "tracked-emitter";
  tracked.is_emitting = true;
  esr_session::EsrSceneEmitter missed;
  missed.emitter_id = 202U;
  missed.emitter_name = "missed-emitter";
  missed.is_emitting = true;
  input.scene = {tracked, missed};

  esr_session::EsrCycleResult first_result;
  first_result.input_cycle_index = input.cycle_index;
  first_result.executed_this_cycle = true;
  first_result.output_frame.cycle_index = input.cycle_index;
  ::electronic_surveillance_radar::session::TruthAssociationRecord association;
  association.observation_id = 8001U;
  association.truth_emitter_id = 201U;
  association.matched = true;
  association.confidence = 0.9f;
  first_result.output_frame.truth_evaluation_output.associations.push_back(association);

  esr_session::EsrEmitterLifecycleRecorder recorder;
  std::vector<esr_session::EsrEmitterLifecycleEvent> events = recorder.Update(input, first_result);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events.front().kind, esr_session::EsrEmitterLifecycleEventKind::kFirstObserved);
  EXPECT_EQ(events.front().emitter_name, "tracked-emitter");
  EXPECT_EQ(events.front().observation_id, 8001U);

  esr_session::EsrCycleResult second_result;
  second_result.input_cycle_index = 31U;
  second_result.executed_this_cycle = true;
  second_result.output_frame.cycle_index = 31U;
  events = recorder.Update(input, second_result);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events.front().kind, esr_session::EsrEmitterLifecycleEventKind::kLost);
  EXPECT_EQ(events.front().reason, esr_session::EsrEmitterLifecycleReason::kNoMatchedObservation);

  esr_session::EsrEmitterLifecycleRecorder diagnose_recorder(
      esr_session::EsrEmitterLifecycleRecorderConfig{true});
  events = diagnose_recorder.Update(input, second_result);
  ASSERT_EQ(events.size(), 2U);
  EXPECT_EQ(events[0].kind, esr_session::EsrEmitterLifecycleEventKind::kNotObserved);
  EXPECT_EQ(events[0].reason, esr_session::EsrEmitterLifecycleReason::kNoMatchedObservation);
  EXPECT_EQ(events[1].emitter_name, "missed-emitter");
}
