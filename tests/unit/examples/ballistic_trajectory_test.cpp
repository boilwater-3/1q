/**
 * @file ballistic_trajectory_test.cpp
 * @brief 弹道目标二体椭圆轨道（examples/scenes/ballistic_trajectory.*）单元测试。
 *
 * 覆盖：三约束闭式解对照设计文档锚点（docs/review/
 * rir_ballistic_scene_design_2026-08-28.md §3.1 表，5 目标 e/a/v_bo/时刻）、
 * 顶点/端点构造闭合（顶高 1%、端点 1 km）、助推段占位与速度爬升、
 * 弧段速度剖面（vis-viva）、非法几何拒绝、scene_script 生命周期
 * （MakeTargetStates 初值 + AdvanceTargetStates 绝对时间求值）。
 */

#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/types.h"
#include "scenes/ballistic_trajectory.h"
#include "scenes/scene_data.h"
#include "scenes/scene_script.h"

namespace ca = component_attachment;
namespace app = component_attachment::app;

namespace {

constexpr double kMeanEarthRadiusM = 6371000.0;
constexpr double kEarthMuM3PerS2 = 3.986004418e14;

/// 设计文档 §2/§3.1 清洗后目标集（客户句柄号；顶高 m、顶高时刻 s）。
struct DesignTarget {
  std::uint32_t id;
  oneq::coordinate::LlaPositionDegM start_lla;
  oneq::coordinate::LlaPositionDegM end_lla;
  double max_alt_m;
  double max_alt_time_s;
  double design_e;            /**< §3.1 表 e */
  double design_a_km;         /**< §3.1 表 a（km） */
  double design_v_bo_mps;     /**< §3.1 表拟合关机点速度（m/s） */
  double design_t_apogee_s;   /**< §3.1 表拟合顶高时刻（s） */
  double design_t_flight_s;   /**< §3.1 表拟合落点时刻（s，自发射起算） */
};

const DesignTarget kDesignTargets[] = {
    {208U, {43.195964, 122.728481, 0.0}, {45.311723, -121.221318, 0.0},
     1663185.661, 1247.0, 0.559, 5153.0, 6912.0, 961.0, 1923.0},
    {210U, {39.447564, 119.579139, 0.0}, {46.152929, -122.022365, 0.0},
     1684257.510, 1257.0, 0.541, 5227.0, 6991.0, 987.0, 1974.0},
    {227U, {43.325190, 121.543133, 0.0}, {54.693469, -121.170652, 0.0},
     1561662.676, 1172.0, 0.586, 5002.0, 6741.0, 898.0, 1796.0},
    {231U, {39.462958, 119.052750, 0.0}, {55.468404, -122.139840, 0.0},
     1580833.855, 1181.0, 0.567, 5076.0, 6827.0, 922.0, 1844.0},
    // 249：终点/顶高/极速按 210 簇补齐（§1.2 裁定），与 210 同解。
    {249U, {39.448106, 119.578338, 0.0}, {46.152929, -122.022365, 0.0},
     1684257.510, 1257.0, 0.541, 5227.0, 6991.0, 987.0, 1974.0},
};

double EcefDistance(const oneq::coordinate::EcefPositionM& a,
                    const oneq::coordinate::EcefPositionM& b) {
  const double dx = a.x_m - b.x_m;
  const double dy = a.y_m - b.y_m;
  const double dz = a.z_m - b.z_m;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double EcefNorm(const oneq::coordinate::EcefPositionM& a) {
  return std::sqrt(a.x_m * a.x_m + a.y_m * a.y_m + a.z_m * a.z_m);
}

/// 设计表锚点断言（±2%；实现用 WGS84 实际半径而设计表用 6371 km 球面近似，
/// 实测偏差 ≤ 0.5%，2% 容差覆盖口径差）。
void ExpectMatchesDesignAnchors(const DesignTarget& target,
                                const app::BallisticTrajectory& trajectory) {
  EXPECT_NEAR(trajectory.eccentricity, target.design_e, 0.02 * target.design_e);
  EXPECT_NEAR(trajectory.semi_major_axis_m / 1000.0, target.design_a_km,
              0.02 * target.design_a_km);
  EXPECT_NEAR(trajectory.burnout_velocity_mps, target.design_v_bo_mps,
              0.02 * target.design_v_bo_mps);
  EXPECT_NEAR(trajectory.fit_time_to_apogee_s, target.design_t_apogee_s,
              0.02 * target.design_t_apogee_s);
  EXPECT_NEAR(trajectory.fit_flight_time_s, target.design_t_flight_s,
              0.02 * target.design_t_flight_s);
  // 两段式时间基准：t_bo = 数据顶高时刻 − 拟合顶高时刻（设计表 258–286 s）。
  EXPECT_NEAR(trajectory.burnout_time_s,
              target.max_alt_time_s - trajectory.fit_time_to_apogee_s, 1e-6);
  EXPECT_GT(trajectory.burnout_time_s, 0.0);
  EXPECT_NEAR(trajectory.landing_time_s,
              trajectory.burnout_time_s + trajectory.fit_flight_time_s, 1e-6);
}

}  // namespace

TEST(BallisticTrajectoryTest, SolveMatchesDesignTableAnchors) {
  for (const DesignTarget& target : kDesignTargets) {
    app::BallisticTrajectory trajectory;
    ASSERT_TRUE(app::SolveBallisticTrajectory(target.start_lla, target.end_lla,
                                              target.max_alt_m, target.max_alt_time_s,
                                              &trajectory))
        << "target " << target.id;
    ASSERT_TRUE(trajectory.valid);
    ExpectMatchesDesignAnchors(target, trajectory);
  }
}

TEST(BallisticTrajectoryTest, ApogeeAndEndpointsCloseByConstruction) {
  for (const DesignTarget& target : kDesignTargets) {
    app::BallisticTrajectory trajectory;
    ASSERT_TRUE(app::SolveBallisticTrajectory(target.start_lla, target.end_lla,
                                              target.max_alt_m, target.max_alt_time_s,
                                              &trajectory));

    // 顶点：t = max_alt_time_s 处地心距 = 6371 km + max_alt（<1% 容差，按构造
    // 应为精确闭合），且为邻域极大（±30 s 两侧更低）。
    oneq::coordinate::EcefPositionM position;
    oneq::coordinate::EcefVelocityMps velocity;
    app::PropagateBallistic(trajectory, target.max_alt_time_s, &position, &velocity);
    EXPECT_NEAR(EcefNorm(position), kMeanEarthRadiusM + target.max_alt_m,
                0.01 * target.max_alt_m);
    oneq::coordinate::EcefPositionM before;
    oneq::coordinate::EcefPositionM after;
    app::PropagateBallistic(trajectory, target.max_alt_time_s - 30.0, &before, nullptr);
    app::PropagateBallistic(trajectory, target.max_alt_time_s + 30.0, &after, nullptr);
    EXPECT_LT(EcefNorm(before), EcefNorm(position));
    EXPECT_LT(EcefNorm(after), EcefNorm(position));

    // 顶点大地高（WGS84）与顶高一致（<1%）。
    oneq::coordinate::LlaPositionDegM apogee_lla;
    ASSERT_TRUE(oneq::coordinate::TryEcefToLla(position, &apogee_lla));
    EXPECT_NEAR(apogee_lla.altitude_m, target.max_alt_m, 0.01 * target.max_alt_m);

    // 发射点/落点闭合（<1 km）：弧段起点 t_bo、落点 landing_time_s。
    oneq::coordinate::EcefPositionM start_ecef;
    oneq::coordinate::EcefPositionM end_ecef;
    ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(target.start_lla, &start_ecef));
    ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(target.end_lla, &end_ecef));
    app::PropagateBallistic(trajectory, trajectory.burnout_time_s, &position, nullptr);
    EXPECT_LT(EcefDistance(position, start_ecef), 1000.0);
    app::PropagateBallistic(trajectory, trajectory.landing_time_s, &position, nullptr);
    EXPECT_LT(EcefDistance(position, end_ecef), 1000.0);
  }
}

TEST(BallisticTrajectoryTest, BoostPhaseHoldsLaunchPadWithVelocityRamp) {
  const DesignTarget& target = kDesignTargets[0];  // 208：t_bo ≈ 284 s
  app::BallisticTrajectory trajectory;
  ASSERT_TRUE(app::SolveBallisticTrajectory(target.start_lla, target.end_lla,
                                            target.max_alt_m, target.max_alt_time_s,
                                            &trajectory));
  oneq::coordinate::EcefPositionM start_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(target.start_lla, &start_ecef));

  // 助推段：位置保持发射点，速度按 0→v_bo 线性爬升。
  oneq::coordinate::EcefPositionM position;
  oneq::coordinate::EcefVelocityMps velocity;
  for (double t : {0.0, 1.0, 100.0, trajectory.burnout_time_s - 1.0}) {
    app::PropagateBallistic(trajectory, t, &position, &velocity);
    EXPECT_LT(EcefDistance(position, start_ecef), 1.0) << "t=" << t;
    const double speed = std::sqrt(velocity.x_mps * velocity.x_mps +
                                   velocity.y_mps * velocity.y_mps +
                                   velocity.z_mps * velocity.z_mps);
    EXPECT_NEAR(speed, trajectory.burnout_velocity_mps * t / trajectory.burnout_time_s,
                1.0) << "t=" << t;
  }
  // 关机点边界连续：t = t_bo 处仍在发射点（弧段起点）。
  app::PropagateBallistic(trajectory, trajectory.burnout_time_s, &position, nullptr);
  EXPECT_LT(EcefDistance(position, start_ecef), 1.0);
}

TEST(BallisticTrajectoryTest, ArcSpeedProfileMatchesVisViva) {
  const DesignTarget& target = kDesignTargets[0];
  app::BallisticTrajectory trajectory;
  ASSERT_TRUE(app::SolveBallisticTrajectory(target.start_lla, target.end_lla,
                                            target.max_alt_m, target.max_alt_time_s,
                                            &trajectory));
  auto speed_at = [&](double t) {
    oneq::coordinate::EcefVelocityMps velocity;
    app::PropagateBallistic(trajectory, t, nullptr, &velocity);
    return std::sqrt(velocity.x_mps * velocity.x_mps + velocity.y_mps * velocity.y_mps +
                     velocity.z_mps * velocity.z_mps);
  };
  // 关机点速度 = vis-viva(r0)（求解器已按此闭式给值，推进链不得走样）。
  EXPECT_NEAR(speed_at(trajectory.burnout_time_s), trajectory.burnout_velocity_mps, 1.0);
  // 顶点速度 = vis-viva(r_a)，显著低于关机点速度（弹道中段最慢点）。
  const double vis_viva_apogee = std::sqrt(
      kEarthMuM3PerS2 * (2.0 / trajectory.apogee_radius_m - 1.0 / trajectory.semi_major_axis_m));
  EXPECT_NEAR(speed_at(target.max_alt_time_s), vis_viva_apogee, 1.0);
  EXPECT_LT(speed_at(target.max_alt_time_s), 0.8 * trajectory.burnout_velocity_mps);
}

TEST(BallisticTrajectoryTest, RejectsInconsistentGeometry) {
  app::BallisticTrajectory trajectory;
  // 起落点重合：无唯一轨道面。
  EXPECT_FALSE(app::SolveBallisticTrajectory({30.0, 120.0, 0.0}, {30.0, 120.0, 0.0},
                                             500000.0, 600.0, &trajectory));
  // 对跖点：轨道面退化。
  EXPECT_FALSE(app::SolveBallisticTrajectory({0.0, 0.0, 0.0}, {0.0, 180.0, 0.0},
                                             500000.0, 1800.0, &trajectory));
  // 顶点不高于端点（赤道端点地心半径 6378 km > 6371 km + 1 km）。
  EXPECT_FALSE(app::SolveBallisticTrajectory({0.0, 122.0, 0.0}, {0.5, -121.0, 0.0},
                                             1000.0, 60.0, &trajectory));
  // 顶高/时刻非正。
  EXPECT_FALSE(app::SolveBallisticTrajectory({30.0, 120.0, 0.0}, {31.0, 121.0, 0.0},
                                             0.0, 600.0, &trajectory));
  EXPECT_FALSE(app::SolveBallisticTrajectory({30.0, 120.0, 0.0}, {31.0, 121.0, 0.0},
                                             500000.0, 0.0, &trajectory));
  // 输出指针为空：拒绝（防御）。
  EXPECT_FALSE(app::SolveBallisticTrajectory({30.0, 120.0, 0.0}, {31.0, 121.0, 0.0},
                                             500000.0, 600.0, nullptr));
}

TEST(BallisticTrajectoryTest, SceneScriptBallisticLifecycle) {
  const DesignTarget& target = kDesignTargets[0];  // 208
  app::ScriptedTarget entry;
  entry.id = target.id;
  entry.type = "ballistic";
  entry.is_ballistic = true;
  entry.start_lla = target.start_lla;
  entry.end_lla = target.end_lla;
  entry.max_alt_m = target.max_alt_m;
  entry.max_alt_time_s = target.max_alt_time_s;
  entry.rcs = 0.1;
  entry.temperature_k = 600.0;
  entry.projected_area_m2 = 1.0;
  entry.radiant_intensity_w_per_sr = 300.0;

  const oneq::coordinate::LlaPositionDegM origin{64.2878, -149.1746, 0.0};
  std::vector<app::TargetEcefState> states = app::MakeTargetStates({entry}, origin);
  ASSERT_EQ(states.size(), 1U);
  ASSERT_TRUE(states[0].ballistic.valid);
  // 初值 = 场景时刻 0（助推段：静止于发射点）。
  oneq::coordinate::EcefPositionM start_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(target.start_lla, &start_ecef));
  EXPECT_LT(EcefDistance(states[0].position, start_ecef), 1.0);
  EXPECT_DOUBLE_EQ(states[0].velocity.x_mps, 0.0);

  // 周期推进 = 绝对时间求值：cycle 1 仍助推段在发射点；cycle 1247 顶点；
  // 任意周期重复求值结果一致（无累积漂移）。
  app::AdvanceTargetStates(states, 1U, 1.0, origin);
  EXPECT_LT(EcefDistance(states[0].position, start_ecef), 1.0);
  app::AdvanceTargetStates(states, target.max_alt_time_s, 1.0, origin);
  EXPECT_NEAR(EcefNorm(states[0].position),
              kMeanEarthRadiusM + target.max_alt_m, 0.01 * target.max_alt_m);
  const oneq::coordinate::EcefPositionM at_apogee = states[0].position;
  app::AdvanceTargetStates(states, target.max_alt_time_s, 1.0, origin);
  EXPECT_DOUBLE_EQ(states[0].position.x_m, at_apogee.x_m);
  EXPECT_DOUBLE_EQ(states[0].position.y_m, at_apogee.y_m);
  EXPECT_DOUBLE_EQ(states[0].position.z_m, at_apogee.z_m);
}
