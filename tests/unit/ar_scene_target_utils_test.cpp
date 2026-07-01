#include <gtest/gtest.h>

#include "airborne_radar/session/ArSceneTargetUtils.h"

using namespace airborne_radar::session;

namespace {

TEST(RadarSceneTargetUtilsTest, MakeSceneTargetSetsAllFields) {
  const RadarSceneTarget target = MakeSceneTarget(42, 100.0f, 200.0f, 300.0f, 10.0f, 20.0f, 30.0f,
                                                  5.0f, 3);

  EXPECT_EQ(target.external_target_id, 42u);
  EXPECT_FLOAT_EQ(target.velocity_x, 10.0f);
  EXPECT_FLOAT_EQ(target.velocity_y, 20.0f);
  EXPECT_FLOAT_EQ(target.velocity_z, 30.0f);
  EXPECT_FLOAT_EQ(target.rcs, 5.0f);
  EXPECT_FLOAT_EQ(target.position_x, 100.0f);
  EXPECT_FLOAT_EQ(target.position_y, 200.0f);
  EXPECT_FLOAT_EQ(target.position_z, 300.0f);
  EXPECT_EQ(target.target_swerling_type, 3);
  EXPECT_GT(target.range_m, 0.0f);
}

TEST(RadarSceneTargetUtilsTest, MakeSceneTargetComputesRange) {
  const RadarSceneTarget target = MakeSceneTarget(1, 3.0f, 4.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);

  EXPECT_FLOAT_EQ(target.range_m, 5.0f);
}

TEST(RadarSceneTargetUtilsTest, NormalizeSceneTargetGeometryBackfillsRange) {
  RadarSceneTarget target;
  target.position_x = 3.0f;
  target.position_y = 4.0f;
  target.position_z = 0.0f;
  target.range_m = 0.0f;

  NormalizeSceneTargetGeometry(&target);
  EXPECT_FLOAT_EQ(target.range_m, 5.0f);
}

TEST(RadarSceneTargetUtilsTest, NormalizeSceneTargetGeometrySkipsIfRangeSet) {
  RadarSceneTarget target;
  target.position_x = 3.0f;
  target.position_y = 4.0f;
  target.position_z = 0.0f;
  target.range_m = 99.0f;

  NormalizeSceneTargetGeometry(&target);
  EXPECT_FLOAT_EQ(target.range_m, 99.0f);
}

TEST(RadarSceneTargetUtilsTest, NormalizeSceneTargetGeometryHandlesNullptr) {
  RadarSceneTarget* null_target = nullptr;
  NormalizeSceneTargetGeometry(null_target);
  RadarSceneTargetList* null_list = nullptr;
  NormalizeSceneTargetGeometry(null_list);
}

TEST(RadarSceneTargetUtilsTest, NormalizeSceneTargetGeometryBatch) {
  RadarSceneTargetList targets(2);
  targets[0].position_x = 3.0f;
  targets[0].position_y = 4.0f;
  targets[0].range_m = 0.0f;

  targets[1].position_x = 6.0f;
  targets[1].position_y = 8.0f;
  targets[1].range_m = 0.0f;

  NormalizeSceneTargetGeometry(&targets);
  EXPECT_FLOAT_EQ(targets[0].range_m, 5.0f);
  EXPECT_FLOAT_EQ(targets[1].range_m, 10.0f);
}

}  // namespace
