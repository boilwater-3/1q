#include <gtest/gtest.h>

#include "1q/electromagnetics/RfScene.h"
#include "electronic_surveillance_radar/session/EsrReplayFlatbufferCodec.h"

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
  input.interference.world_cycle_index = input.cycle_index;
  input.interference.window_start_time_s = input.cycle_start_time_s;
  input.interference.window_duration_s = input.dt_sec;
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
  input.interference.emissions.push_back(emission);
  return input;
}

TEST(EsrReplayCodecRoundtripTest, CycleInputPreservesRfV2Frame) {
  const EsrCycleInput input = MakeInput();
  EsrCycleInput decoded;
  ASSERT_TRUE(DecodeEsrCycleInput(EncodeEsrCycleInput(input), &decoded));
  EXPECT_EQ(decoded.cycle_index, input.cycle_index);
  EXPECT_DOUBLE_EQ(decoded.cycle_start_time_s, input.cycle_start_time_s);
  ASSERT_EQ(decoded.interference.emissions.size(), 1U);
  EXPECT_EQ(decoded.interference.emissions.front().identity.emission_id, 4U);
  EXPECT_EQ(decoded.interference.emissions.front().waveform.kind,
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

}  // namespace
}  // namespace session
}  // namespace electronic_surveillance_radar
