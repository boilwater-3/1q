#include <gtest/gtest.h>

#include "flatbuffers/flatbuffers.h"
#include "1q/electromagnetics/RfScene.h"
#include "electronic_surveillance_radar/session/EsrReplayFlatbufferCodec.h"
#include "electronic_surveillance_radar/session/generated/esr_replay_generated.h"
#include "electronic_surveillance_radar/session/generated/esr_session_replay_generated.h"

namespace electronic_surveillance_radar {
namespace session {
namespace {

EsrCycleInput MakeInput() {
  EsrCycleInput input;
  input.cycle_index = 7U;
  input.cycle_start_time_s = 20.0;
  input.dt_sec = 0.5f;
  input.platform_entity_id = 1U;
  input.has_platform_ecef_kinematics = true;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.rf_emissions.world_cycle_index = input.cycle_index;
  input.rf_emissions.window_start_time_s = input.cycle_start_time_s;
  input.rf_emissions.window_duration_s = input.dt_sec;
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity.platform_id = 2U;
  emission.identity.equipment_id = 3U;
  emission.identity.emission_id = 4U;
  emission.position_ecef_m.x_m = 6378137.0;
  emission.position_ecef_m.y_m = 1000.0;
  emission.antenna.boresight_ecef.y = -1.0;
  EXPECT_TRUE(oneq::electromagnetics::TryCreateRfPulseTrainWaveform(
      20.0, 10.0e9, 1.0e6, 100.0, 1.0e-6, 1.0e-3, 10U, 0.0, 9U, 1U,
      &emission.waveform));
  input.rf_emissions.emissions.push_back(emission);
  return input;
}

TEST(EsrReplayCodecRoundtripTest, CycleInputPreservesRfV2Frame) {
  const EsrCycleInput input = MakeInput();
  EsrCycleInput decoded;
  ASSERT_TRUE(DecodeEsrCycleInput(EncodeEsrCycleInput(input), &decoded));
  EXPECT_EQ(decoded.cycle_index, input.cycle_index);
  EXPECT_DOUBLE_EQ(decoded.cycle_start_time_s, input.cycle_start_time_s);
  ASSERT_EQ(decoded.rf_emissions.emissions.size(), 1U);
  EXPECT_EQ(decoded.rf_emissions.emissions.front().identity.emission_id, 4U);
  EXPECT_EQ(decoded.rf_emissions.emissions.front().waveform.kind,
            oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain);
}

TEST(EsrReplayCodecRoundtripTest, CycleResultPreservesExplicitStatus) {
  EsrCycleResult result;
  result.input_cycle_index = 7U;
  result.status = EsrCycleExecutionStatus::kPoweredOff;
  result.abort_reason = EsrPipelineAbortReason::kSensorPoweredOff;
  EsrCycleResult decoded;
  ASSERT_TRUE(DecodeEsrCycleResult(EncodeEsrCycleResult(result), &decoded));
  EXPECT_EQ(decoded.status, EsrCycleExecutionStatus::kPoweredOff);
  EXPECT_EQ(decoded.abort_reason, EsrPipelineAbortReason::kSensorPoweredOff);
}

TEST(EsrReplayCodecRoundtripTest,
     UnknownWaveformClassRejectsWithoutMutatingDestination) {
  flatbuffers::FlatBufferBuilder builder;
  esr::replay::EmitterObservationBuilder observation_builder(builder);
  observation_builder.add_observation_id(1U);
  observation_builder.add_quality(
      static_cast<std::int32_t>(EsrObservationQuality::kHigh));
  observation_builder.add_waveform_class(999);
  const auto observation = observation_builder.Finish();
  const std::vector<flatbuffers::Offset<esr::replay::EmitterObservation>>
      observations{observation};
  esr::replay::ObservationOutputBuilder output_builder(builder);
  output_builder.add_observations(builder.CreateVector(observations));
  const auto observation_output = output_builder.Finish();
  const auto root = esr::replay::CreateEsrOutputFrame(
      builder, 1U, 2U, observation_output, 0);
  builder.Finish(root);
  const std::string bytes(
      reinterpret_cast<const char*>(builder.GetBufferPointer()),
      builder.GetSize());

  EsrOutputFrame destination;
  destination.cycle_index = 777U;
  EXPECT_FALSE(DecodeEsrOutputFrame(bytes, &destination));
  EXPECT_EQ(destination.cycle_index, 777U);
  EXPECT_TRUE(destination.observation_output.observations.empty());
}

TEST(EsrReplayCodecRoundtripTest,
     UnknownSessionConfigEnumRejectsWithoutMutatingDestination) {
  flatbuffers::FlatBufferBuilder builder;
  const auto mission =
      esr::replay::CreateEsrMissionConfig(builder, true, 999, 0);
  builder.Finish(
      esr::replay::CreateEsrSessionConfig(builder, 0, mission, 0, 0));
  const std::string bytes(
      reinterpret_cast<const char*>(builder.GetBufferPointer()),
      builder.GetSize());

  config::EsrSessionConfig destination;
  destination.mission.power_on = false;
  EXPECT_FALSE(DecodeEsrSessionConfig(bytes, &destination));
  EXPECT_FALSE(destination.mission.power_on);
  EXPECT_EQ(destination.mission.work_mode, config::EsrWorkMode::kEsm);
}

TEST(EsrReplayCodecRoundtripTest,
     UnknownRuntimePatchEnumRejectsWithoutMutatingDestination) {
  flatbuffers::FlatBufferBuilder builder;
  esr::replay::EsrRuntimeConfigPatchBuilder patch_builder(builder);
  patch_builder.add_has_work_mode(true);
  patch_builder.add_work_mode(999);
  builder.Finish(patch_builder.Finish());
  const std::string bytes(
      reinterpret_cast<const char*>(builder.GetBufferPointer()),
      builder.GetSize());

  config::EsrRuntimeConfigPatch destination;
  destination.has_sensor_enabled = true;
  destination.sensor_enabled = false;
  EXPECT_FALSE(DecodeEsrRuntimeConfigPatch(bytes, &destination));
  EXPECT_TRUE(destination.has_sensor_enabled);
  EXPECT_FALSE(destination.sensor_enabled);
  EXPECT_FALSE(destination.has_work_mode);
}

TEST(EsrReplayCodecRoundtripTest,
     RuntimePatchEventPreservesStructuredApplyResult) {
  config::EsrRuntimeConfigPatch patch;
  patch.has_work_mode = true;
  patch.work_mode = config::EsrWorkMode::kRwr;
  EsrRuntimeConfigApplyResult expected;
  expected.status = EsrRuntimeConfigApplyStatus::kApplied;
  expected.has_requested_update = true;
  expected.applied = true;

  config::EsrRuntimeConfigPatch decoded_patch;
  EsrRuntimeConfigApplyResult decoded_result;
  ASSERT_TRUE(DecodeEsrRuntimeConfigPatchEvent(
      EncodeEsrRuntimeConfigPatchEvent(patch, expected), &decoded_patch,
      &decoded_result));
  EXPECT_TRUE(decoded_patch.has_work_mode);
  EXPECT_EQ(decoded_patch.work_mode, config::EsrWorkMode::kRwr);
  EXPECT_EQ(decoded_result.status, expected.status);
  EXPECT_TRUE(decoded_result.has_requested_update);
  EXPECT_TRUE(decoded_result.applied);
}

TEST(EsrReplayCodecRoundtripTest,
     UnknownRuntimeApplyStatusRejectsEventAtomically) {
  const std::string patch_bytes =
      EncodeEsrRuntimeConfigPatch(config::EsrRuntimeConfigPatch{});
  flatbuffers::FlatBufferBuilder builder;
  const auto payload = builder.CreateVector(
      reinterpret_cast<const std::uint8_t*>(patch_bytes.data()),
      patch_bytes.size());
  const auto result =
      esr::replay::CreateEsrRuntimeConfigApplyResult(builder, 999, true, true);
  builder.Finish(esr::replay::CreateEsrRuntimeConfigPatchEvent(
      builder, payload, result));
  const std::string bytes(
      reinterpret_cast<const char*>(builder.GetBufferPointer()),
      builder.GetSize());

  config::EsrRuntimeConfigPatch destination_patch;
  destination_patch.has_sensor_enabled = true;
  EsrRuntimeConfigApplyResult destination_result;
  destination_result.status =
      EsrRuntimeConfigApplyStatus::kRejectedInvalidPolicy;
  EXPECT_FALSE(DecodeEsrRuntimeConfigPatchEvent(
      bytes, &destination_patch, &destination_result));
  EXPECT_TRUE(destination_patch.has_sensor_enabled);
  EXPECT_EQ(destination_result.status,
            EsrRuntimeConfigApplyStatus::kRejectedInvalidPolicy);
}

TEST(EsrReplayCodecRoundtripTest,
     DeceptionClassFieldRoundtripsCorrectly) {
  EsrOutputFrame frame;
  frame.cycle_index = 1U;
  frame.batch_id = 2U;
  EmitterObservation obs;
  obs.observation_id = 10U;
  obs.quality = EsrObservationQuality::kHigh;
  obs.waveform_class = EsrWaveformClass::kPulse;
  obs.deception_class = EsrDeceptionClass::kLikelyFalseTarget;
  frame.observation_output.observations.push_back(obs);
  EsrOutputFrame decoded;
  ASSERT_TRUE(DecodeEsrOutputFrame(EncodeEsrOutputFrame(frame), &decoded));
  ASSERT_EQ(decoded.observation_output.observations.size(), 1U);
  EXPECT_EQ(decoded.observation_output.observations.front().deception_class,
            EsrDeceptionClass::kLikelyFalseTarget);
}

TEST(EsrReplayCodecRoundtripTest,
     UnknownDeceptionClassRejectsWithoutMutatingDestination) {
  flatbuffers::FlatBufferBuilder builder;
  esr::replay::EmitterObservationBuilder observation_builder(builder);
  observation_builder.add_observation_id(1U);
  observation_builder.add_quality(
      static_cast<std::int32_t>(EsrObservationQuality::kHigh));
  observation_builder.add_waveform_class(
      static_cast<std::int32_t>(EsrWaveformClass::kPulse));
  observation_builder.add_deception_class(999);
  const auto observation = observation_builder.Finish();
  const std::vector<flatbuffers::Offset<esr::replay::EmitterObservation>>
      observations{observation};
  esr::replay::ObservationOutputBuilder output_builder(builder);
  output_builder.add_observations(builder.CreateVector(observations));
  const auto observation_output = output_builder.Finish();
  const auto root = esr::replay::CreateEsrOutputFrame(
      builder, 1U, 2U, observation_output, 0);
  builder.Finish(root);
  const std::string bytes(
      reinterpret_cast<const char*>(builder.GetBufferPointer()),
      builder.GetSize());

  EsrOutputFrame destination;
  destination.cycle_index = 777U;
  EXPECT_FALSE(DecodeEsrOutputFrame(bytes, &destination));
  EXPECT_EQ(destination.cycle_index, 777U);
  EXPECT_TRUE(destination.observation_output.observations.empty());
}

// 旧 trace 不含 deception_class 字段时默认解码为 kNone
TEST(EsrReplayCodecRoundtripTest,
     MissingDeceptionClassFieldDefaultsToNone) {
  flatbuffers::FlatBufferBuilder builder;
  esr::replay::EmitterObservationBuilder observation_builder(builder);
  observation_builder.add_observation_id(1U);
  observation_builder.add_quality(
      static_cast<std::int32_t>(EsrObservationQuality::kHigh));
  observation_builder.add_waveform_class(
      static_cast<std::int32_t>(EsrWaveformClass::kPulse));
  // 故意不调用 add_deception_class —— 模拟旧 trace
  const auto observation = observation_builder.Finish();
  const std::vector<flatbuffers::Offset<esr::replay::EmitterObservation>>
      observations{observation};
  esr::replay::ObservationOutputBuilder output_builder(builder);
  output_builder.add_observations(builder.CreateVector(observations));
  const auto observation_output = output_builder.Finish();
  const auto root = esr::replay::CreateEsrOutputFrame(
      builder, 1U, 2U, observation_output, 0);
  builder.Finish(root);
  const std::string bytes(
      reinterpret_cast<const char*>(builder.GetBufferPointer()),
      builder.GetSize());

  EsrOutputFrame decoded;
  ASSERT_TRUE(DecodeEsrOutputFrame(bytes, &decoded));
  ASSERT_EQ(decoded.observation_output.observations.size(), 1U);
  EXPECT_EQ(decoded.observation_output.observations.front().deception_class,
            EsrDeceptionClass::kNone);
}

}  // namespace
}  // namespace session
}  // namespace electronic_surveillance_radar
