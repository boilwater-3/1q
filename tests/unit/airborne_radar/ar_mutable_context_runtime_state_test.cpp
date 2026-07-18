#include <gtest/gtest.h>

#include <cstdint>

#include "airborne_radar/session/MutableArContext.h"

namespace airborne_radar {
namespace session {
namespace tests {
namespace {

ArCycleInput MakeCycleInput(std::uint32_t cycle_index, std::uint64_t target_id, float altitude_m,
                            float dt_sec, double yaw_deg) {
  ArCycleInput input;
  input.cycle_index = cycle_index;
  input.dt_sec = dt_sec;
  input.platform_altitude_m = altitude_m;
  input.platform_pose.attitude_deg.yaw_deg = yaw_deg;
  input.platform_pose.attitude_deg.pitch_deg = -0.5 * yaw_deg;
  input.platform_pose.attitude_deg.roll_deg = 0.25 * yaw_deg;
  ArSceneTarget target;
  target.external_target_id = target_id;
  target.target_name = "runtime-state-target";
  input.scene.push_back(target);
  return input;
}

void SeedContext(MutableArContext* context, std::uint32_t cycle_index, std::uint64_t target_id,
                 float altitude_m, float dt_sec, double yaw_deg, std::uint64_t profile_version) {
  context->BeginCycle(MakeCycleInput(cycle_index, target_id, altitude_m, dt_sec, yaw_deg));
  context->SubmitControlCommand(ArCommand(ArCommandType::SET_AGILITY_FREQ, ArCommandSource::ECCM));
  ArControlProfile profile;
  profile.version = profile_version;
  profile.enable_agility_frequency = true;
  profile.lpi_power_scale = 0.4f;
  context->UpdateRadarControlProfile(profile);
}

void ExpectContextState(const MutableArContext& context, std::uint32_t cycle_index,
                        std::uint64_t target_id, float altitude_m, float dt_sec, double yaw_deg,
                        std::uint64_t profile_version) {
  ASSERT_EQ(context.GetSceneTargets().size(), 1U);
  EXPECT_EQ(context.GetSceneTargets().front().external_target_id, target_id);
  EXPECT_EQ(context.GetSceneTargets().front().target_name, "runtime-state-target");
  EXPECT_DOUBLE_EQ(context.GetPlatformAttitude().yaw_deg, yaw_deg);
  EXPECT_DOUBLE_EQ(context.GetPlatformAttitude().pitch_deg, -0.5 * yaw_deg);
  EXPECT_DOUBLE_EQ(context.GetPlatformAttitude().roll_deg, 0.25 * yaw_deg);
  EXPECT_FLOAT_EQ(context.GetPlatformAltitudeM(), altitude_m);
  EXPECT_FLOAT_EQ(context.GetCycleDeltaTimeSec(), dt_sec);
  EXPECT_EQ(context.GetCycleIndex(), cycle_index);
  ASSERT_EQ(context.GetSubmittedCommands().size(), 1U);
  EXPECT_EQ(context.GetSubmittedCommands().front().type, ArCommandType::SET_AGILITY_FREQ);
  EXPECT_EQ(context.GetSubmittedCommands().front().source, ArCommandSource::ECCM);
  ASSERT_TRUE(context.HasLatestControlProfile());
  EXPECT_EQ(context.GetLatestControlProfile().version, profile_version);
  EXPECT_TRUE(context.GetLatestControlProfile().enable_agility_frequency);
  EXPECT_FLOAT_EQ(context.GetLatestControlProfile().lpi_power_scale, 0.4f);
}

TEST(MutableArContextRuntimeStateTest, ValidSnapshotRestoresObservableState) {
  MutableArContext context;
  SeedContext(&context, 11U, 101U, 7000.0f, 0.25f, 8.0, 3U);
  const ArContextRuntimeState snapshot = context.CaptureRuntimeState();

  SeedContext(&context, 12U, 202U, 8100.0f, 0.5f, 16.0, 4U);

  ASSERT_TRUE(context.RestoreRuntimeState(snapshot));
  ExpectContextState(context, 11U, 101U, 7000.0f, 0.25f, 8.0, 3U);
}

TEST(MutableArContextRuntimeStateTest, ForeignOwnerIsRejectedWithoutMutation) {
  MutableArContext context;
  MutableArContext foreign_context;
  SeedContext(&context, 21U, 301U, 9000.0f, 0.75f, 24.0, 5U);
  SeedContext(&foreign_context, 22U, 302U, 9100.0f, 1.0f, 32.0, 6U);

  EXPECT_FALSE(context.RestoreRuntimeState(foreign_context.CaptureRuntimeState()));
  ExpectContextState(context, 21U, 301U, 9000.0f, 0.75f, 24.0, 5U);
}

TEST(MutableArContextRuntimeStateTest, BadSchemaIsRejectedWithoutMutation) {
  MutableArContext context;
  SeedContext(&context, 31U, 401U, 10000.0f, 1.25f, 40.0, 7U);
  ArContextRuntimeState invalid_snapshot = context.CaptureRuntimeState();
  ++invalid_snapshot.schema_version;
  SeedContext(&context, 32U, 402U, 10100.0f, 1.5f, 48.0, 8U);

  EXPECT_FALSE(context.RestoreRuntimeState(invalid_snapshot));
  ExpectContextState(context, 32U, 402U, 10100.0f, 1.5f, 48.0, 8U);
}

TEST(MutableArContextRuntimeStateTest, EmptySnapshotIsRejectedWithoutMutation) {
  MutableArContext context;
  SeedContext(&context, 41U, 501U, 11000.0f, 1.75f, 56.0, 9U);
  ArContextRuntimeState invalid_snapshot = context.CaptureRuntimeState();
  invalid_snapshot.snapshot.reset();
  SeedContext(&context, 42U, 502U, 11100.0f, 2.0f, 64.0, 10U);

  EXPECT_FALSE(context.RestoreRuntimeState(invalid_snapshot));
  ExpectContextState(context, 42U, 502U, 11100.0f, 2.0f, 64.0, 10U);
}

}  // namespace
}  // namespace tests
}  // namespace session
}  // namespace airborne_radar
