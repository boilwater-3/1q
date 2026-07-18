#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <new>
#include <type_traits>
#include <utility>

#include "airborne_radar/session/MutableArContext.h"

namespace airborne_radar {
namespace session {
namespace tests {
namespace {

static_assert(!std::is_aggregate<ArContextRuntimeState>::value,
              "runtime state must not expose a recomposable aggregate envelope");
static_assert(!std::is_default_constructible<ArContextRuntimeState>::value,
              "runtime state must only be created by MutableArContext");
static_assert(std::is_copy_constructible<ArContextRuntimeState>::value,
              "snapshot must be copyable");
static_assert(std::is_copy_assignable<ArContextRuntimeState>::value,
              "snapshot must be copy assignable as a whole envelope");
static_assert(std::is_move_constructible<ArContextRuntimeState>::value, "snapshot must be movable");
static_assert(std::is_move_assignable<ArContextRuntimeState>::value,
              "snapshot must be move assignable as a whole envelope");
static_assert(!std::is_copy_constructible<MutableArContext>::value,
              "context identity must not be copied to another instance");
static_assert(!std::is_copy_assignable<MutableArContext>::value,
              "context identity must not be copy assigned to another instance");
static_assert(!std::is_move_constructible<MutableArContext>::value,
              "context identity must not be moved to another instance");
static_assert(!std::is_move_assignable<MutableArContext>::value,
              "context identity must not be move assigned to another instance");

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

TEST(MutableArContextRuntimeStateTest, WholeForeignEnvelopeAssignmentIsRejectedWithoutMutation) {
  MutableArContext context;
  MutableArContext foreign_context;
  SeedContext(&context, 31U, 401U, 10000.0f, 1.25f, 40.0, 7U);
  ArContextRuntimeState assigned_snapshot = context.CaptureRuntimeState();
  SeedContext(&foreign_context, 33U, 403U, 10200.0f, 1.75f, 56.0, 9U);
  assigned_snapshot = foreign_context.CaptureRuntimeState();
  SeedContext(&context, 32U, 402U, 10100.0f, 1.5f, 48.0, 8U);

  EXPECT_FALSE(context.RestoreRuntimeState(assigned_snapshot));
  ExpectContextState(context, 32U, 402U, 10100.0f, 1.5f, 48.0, 8U);
}

TEST(MutableArContextRuntimeStateTest, MovedFromEnvelopeIsRejectedWithoutMutation) {
  MutableArContext context;
  SeedContext(&context, 41U, 501U, 11000.0f, 1.75f, 56.0, 9U);
  ArContextRuntimeState moved_from_snapshot = context.CaptureRuntimeState();
  const ArContextRuntimeState retained_snapshot = std::move(moved_from_snapshot);
  SeedContext(&context, 42U, 502U, 11100.0f, 2.0f, 64.0, 10U);

  EXPECT_FALSE(context.RestoreRuntimeState(moved_from_snapshot));
  ExpectContextState(context, 42U, 502U, 11100.0f, 2.0f, 64.0, 10U);
  EXPECT_TRUE(context.RestoreRuntimeState(retained_snapshot));
  ExpectContextState(context, 41U, 501U, 11000.0f, 1.75f, 56.0, 9U);
}

TEST(MutableArContextRuntimeStateTest, ReusedObjectAddressRejectsPreviousLifetimeEnvelope) {
  using ContextStorage =
      typename std::aligned_storage<sizeof(MutableArContext), alignof(MutableArContext)>::type;
  ContextStorage storage;

  MutableArContext* first_context = new (&storage) MutableArContext();
  SeedContext(first_context, 51U, 601U, 12000.0f, 2.25f, 72.0, 11U);
  const ArContextRuntimeState previous_lifetime_snapshot = first_context->CaptureRuntimeState();
  first_context->~MutableArContext();

  MutableArContext* second_context = new (&storage) MutableArContext();
  SeedContext(second_context, 52U, 602U, 12100.0f, 2.5f, 80.0, 12U);

  EXPECT_FALSE(second_context->RestoreRuntimeState(previous_lifetime_snapshot));
  ExpectContextState(*second_context, 52U, 602U, 12100.0f, 2.5f, 80.0, 12U);
  second_context->~MutableArContext();
}

}  // namespace
}  // namespace tests
}  // namespace session
}  // namespace airborne_radar
