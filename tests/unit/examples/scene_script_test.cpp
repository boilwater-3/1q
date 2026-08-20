/**
 * @file scene_script_test.cpp
 * @brief 世界模型目标真值脚本（examples/component_attachment/scene_script.*）
 * 单元测试。
 *
 * 覆盖：目标脚本 → ECEF 状态（id/外观/辐射源频率随真值流转）、变速机动
 * （maneuvers 分段匀速：start_cycle 生效、推进位置闭合）。
 */

#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "1q/coordinate/types.h"
#include "1q/coordinate/velocity_transform.h"
#include "scene_data.h"
#include "scene_script.h"

namespace ca = component_attachment;
namespace demo = component_attachment::demo;

namespace {

/// 构造单目标脚本（含两条机动：cycle 200 转北、cycle 260 停北向速度）。
demo::ScriptedTarget MakeManeuverTarget() {
  demo::ScriptedTarget target;
  target.id = 1001U;
  target.azimuth_deg = 0.0;
  target.range_m = 12000.0;
  target.altitude_m = 400.0;
  target.v_east_mps = 47.0;
  target.v_north_mps = 5.0;
  target.temperature_k = 520.0;
  target.rcs = 2.2;
  target.projected_area_m2 = 18.0;
  target.radiant_intensity_w_per_sr = 3819.864;
  target.emitter_center_frequency_hz = 9.5e9;
  demo::TargetManeuver turn;
  turn.start_cycle = 200U;
  turn.v_east_mps = 47.0;
  turn.v_north_mps = -30.0;  // 转向：向北速 -30 m/s（相对平台快速机动）
  target.maneuvers.push_back(turn);
  demo::TargetManeuver straighten;
  straighten.start_cycle = 260U;
  straighten.v_east_mps = 47.0;
  straighten.v_north_mps = 0.0;
  target.maneuvers.push_back(straighten);
  return target;
}

}  // namespace

TEST(SceneScriptTest, StateCarriesIdentityAndAppearance) {
  const oneq::coordinate::LlaPositionDegM origin{30.0, 120.0, 0.0};
  const std::vector<demo::TargetEcefState> states =
      demo::MakeTargetStates({MakeManeuverTarget()}, origin);

  ASSERT_EQ(states.size(), 1U);
  EXPECT_EQ(states[0].id, 1001U);
  EXPECT_FLOAT_EQ(states[0].temperature_k, 520.0f);
  EXPECT_FLOAT_EQ(states[0].projected_area_m2, 18.0f);
  EXPECT_DOUBLE_EQ(states[0].radiant_intensity_w_per_sr, 3819.864);
  EXPECT_FLOAT_EQ(states[0].rcs, 2.2f);
  EXPECT_DOUBLE_EQ(states[0].emitter_center_frequency_hz, 9.5e9);
  // 初始速度投影到 ECEF 非零（东/北分量）。
  EXPECT_NE(states[0].velocity.x_mps, 0.0);
  ASSERT_EQ(states[0].maneuvers.size(), 2U);
  EXPECT_EQ(states[0].maneuvers[0].start_cycle, 200U);
}

TEST(SceneScriptTest, ManeuverAppliesProjectedVelocityAtStartCycle) {
  const oneq::coordinate::LlaPositionDegM origin{30.0, 120.0, 0.0};
  std::vector<demo::TargetEcefState> states =
      demo::MakeTargetStates({MakeManeuverTarget()}, origin);

  // 机动前推进（cycle 199）：速度保持初始值，位置沿当前速度推进。
  const demo::TargetEcefState before = states[0];
  demo::AdvanceTargetStates(states, 199U, 1.0, origin);
  EXPECT_NEAR(states[0].position.x_m - before.position.x_m,
              before.velocity.x_mps * 1.0, 1e-6);
  EXPECT_NEAR(states[0].position.y_m - before.position.y_m,
              before.velocity.y_mps * 1.0, 1e-6);
  // 初始速度保持（未到机动生效周期）。
  EXPECT_NEAR(states[0].velocity.x_mps, before.velocity.x_mps, 1e-9);

  // cycle 200：机动生效，速度切换为 (47, -30) 的 ENU 精确投影（ECEF 分量是
  // 东/北混合投影，不能按分量直觉断言——以库投影为期望值对比）。
  oneq::coordinate::EnuVelocityMps enu;
  enu.east_mps = 47.0;
  enu.north_mps = -30.0;
  enu.up_mps = 0.0;
  oneq::coordinate::EcefVelocityMps expected;
  oneq::coordinate::TryEnuToEcefVelocity(enu, origin, &expected);
  demo::AdvanceTargetStates(states, 200U, 1.0, origin);
  EXPECT_NEAR(states[0].velocity.x_mps, expected.x_mps, 1e-9);
  EXPECT_NEAR(states[0].velocity.y_mps, expected.y_mps, 1e-9);
  EXPECT_NEAR(states[0].velocity.z_mps, expected.z_mps, 1e-9);
  // 与机动前速度显著不同（北向 5 → -30）。
  EXPECT_GT(std::abs(states[0].velocity.y_mps - before.velocity.y_mps), 1.0);

  // cycle 201（非机动周期）：速度保持 cycle 200 的值，位置继续推进。
  const demo::TargetEcefState post_maneuver = states[0];
  demo::AdvanceTargetStates(states, 201U, 1.0, origin);
  EXPECT_NEAR(states[0].velocity.x_mps, post_maneuver.velocity.x_mps, 1e-9);
  EXPECT_NEAR(states[0].position.x_m - post_maneuver.position.x_m,
              post_maneuver.velocity.x_mps, 1e-6);
  EXPECT_NEAR(states[0].position.y_m - post_maneuver.position.y_m,
              post_maneuver.velocity.y_mps, 1e-6);
}

TEST(SceneScriptTest, LaterManeuverOverridesEarlier) {
  // 多段机动：cycle 260 第二条机动（47, 0）覆盖 cycle 200 的（47, -30）——
  // 分段匀速语义，速度按生效条目切换。
  const oneq::coordinate::LlaPositionDegM origin{30.0, 120.0, 0.0};
  std::vector<demo::TargetEcefState> states =
      demo::MakeTargetStates({MakeManeuverTarget()}, origin);

  oneq::coordinate::EnuVelocityMps enu;
  enu.east_mps = 47.0;
  enu.north_mps = 0.0;
  enu.up_mps = 0.0;
  oneq::coordinate::EcefVelocityMps expected;
  oneq::coordinate::TryEnuToEcefVelocity(enu, origin, &expected);
  demo::AdvanceTargetStates(states, 260U, 1.0, origin);
  EXPECT_NEAR(states[0].velocity.x_mps, expected.x_mps, 1e-9);
  EXPECT_NEAR(states[0].velocity.y_mps, expected.y_mps, 1e-9);
  EXPECT_NEAR(states[0].velocity.z_mps, expected.z_mps, 1e-9);
}
