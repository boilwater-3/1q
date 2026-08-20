#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "1q/flight_dynamic/FlightManager.h"
#include "1q/flight_dynamic/config/FlightDynamicConfig.h"
#include "fd_test_helpers.h"

namespace oneq {
namespace flight_dynamic {
namespace {

// ─── 起飞段积分子步进契约的回归锚点 ────────────────────────────────────────
//
// docs/flight_dynamic/algorithms.md「起飞与降落」的步长上限边界与
// FlightManager.h Step(dt) 的 @note 在此锚定：
//   - dt=0.01（10 ms）：起飞稳定完成（权威用法，10-20 ms 区间内）。
//   - dt=0.1（100 ms）：起飞段发散（地面滑跑/起落架等快动态数值崩溃，
//     实测 roll 达 180° 量级）。
// 此前该证据锚定在 examples/flight_dynamic/takeoff_land_csv.cpp——作为库设计
// 的证据不应落在示例上（示例只编译不运行、也不进 ctest），2026-08-10 迁至本单测。

// 完成门 = 0.95 × 目标高度（Maneuver.cpp kRotateAndClimb 的 kComplete 判定）。
constexpr double kTakeoffTargetAltM = 500.0;
constexpr double kTakeoffCompletionAltM = kTakeoffTargetAltM * 0.95;
// 发散特征阈值：稳定起飞全程 |roll| 应远小于 90°（发散实测达 180° 量级）。
constexpr double kMaxRollDegForStableTakeoff = 90.0;
// 仿真预算 300 s：10 ms → 30000 步，100 ms → 3000 步。
constexpr int kMaxTakeoffSteps10ms = 30000;
constexpr int kMaxTakeoffSteps100ms = 3000;

struct TakeoffRunResult {
  bool completed = false;  // FlightManagerState::kCompleted
  bool aborted = false;    // FlightManagerState::kAborted（含坠毁）
  bool nan_seen = false;   // 过程中出现过非有限状态量
  double max_abs_roll_deg = 0.0;
  double final_agl_m = 0.0;
  int steps_used = 0;
};

config::FlightDynamicConfig MakeGroundStartConfig(const std::string& model) {
  // 地面静止启动：与集成契约一致（示例 takeoff_land_csv 同款配置），
  // 不做空中配平（do_trim=false），地面高度尊重 aircraft-specific reset XML。
  config::FlightDynamicConfig config;
  config.aircraft_model = model;
  config.aircraft_root_dir = FD_JSBSIM_ROOT_DIR;
  config.dt_sec = kDt;  // 与 fd_test_helpers.h 一致：10 ms
  config.do_trim = false;
  config.silent_mode = true;
  config.initial_kinematics.position_frame = coordinate::PositionFrame::kLla;
  config.initial_kinematics.position_lla_deg_m.latitude_deg = 0.0;
  config.initial_kinematics.position_lla_deg_m.longitude_deg = 0.0;
  config.initial_kinematics.position_lla_deg_m.altitude_m = 0.0;
  config.initial_kinematics.velocity_mps.x_mps = 0.0;
  config.initial_kinematics.velocity_mps.y_mps = 0.0;
  config.initial_kinematics.velocity_mps.z_mps = 0.0;
  config.initial_kinematics.attitude_deg.roll_deg = 0.0;
  config.initial_kinematics.attitude_deg.pitch_deg = 0.0;
  config.initial_kinematics.attitude_deg.yaw_deg = 0.0;
  return config;
}

void PushTakeoff(FlightManager& fm) {
  ManeuverCommand tko;
  tko.type = guidance::ManeuverType::kTakeoff;
  tko.target.altitude_m = kTakeoffTargetAltM;
  fm.PushManeuver(tko);
}

TakeoffRunResult RunTakeoff(FlightManager& fm, double dt_sec, int max_steps) {
  TakeoffRunResult result;
  for (int i = 0; i < max_steps; ++i) {
    if (!fm.Step(dt_sec)) break;  // kCompleted/kAborted 后返回 false
    const auto& s = fm.GetVehicleState();
    result.max_abs_roll_deg =
        std::max(result.max_abs_roll_deg, std::abs(s.phi_rad) * 180.0 / kPi);
    if (std::isnan(s.altitude_geod_m) || std::isnan(s.phi_rad) ||
        std::isnan(s.theta_rad) || std::isnan(s.u_mps)) {
      result.nan_seen = true;
    }
    result.steps_used = i + 1;
    const auto state = fm.GetState();
    if (state == FlightManagerState::kCompleted) {
      result.completed = true;
      result.final_agl_m = s.altitude_agl_m;
      return result;
    }
    if (state == FlightManagerState::kAborted) {
      result.aborted = true;
      result.final_agl_m = s.altitude_agl_m;
      return result;
    }
  }
  result.final_agl_m = fm.GetVehicleState().altitude_agl_m;
  return result;
}

// ─── 正向回归：dt=0.01（10 ms）起飞稳定完成 ────────────────────────────────

class Takeoff10msSubstepTest : public ::testing::TestWithParam<std::string> {};

/// 10 ms 子步进下起飞必须完成：kCompleted、无 NaN、滚转有界、高度达完成门。
TEST_P(Takeoff10msSubstepTest, TakeoffCompletesAt10msSubstep) {
  const std::string& model = GetParam();
  FlightManager fm(MakeGroundStartConfig(model));
  ASSERT_EQ(fm.GetState(), FlightManagerState::kReady)
      << model << ": ground-start init failed";

  PushTakeoff(fm);
  const TakeoffRunResult result = RunTakeoff(fm, kDt, kMaxTakeoffSteps10ms);

  EXPECT_TRUE(result.completed)
      << model << ": takeoff not completed at 10 ms substep within "
      << kMaxTakeoffSteps10ms << " steps (300 s sim); agl=" << result.final_agl_m
      << " m";
  EXPECT_FALSE(result.aborted) << model << ": aborted/crashed during takeoff";
  EXPECT_FALSE(result.nan_seen) << model << ": NaN during takeoff";
  EXPECT_LT(result.max_abs_roll_deg, kMaxRollDegForStableTakeoff)
      << model << ": roll excursion during takeoff (divergence signature), max="
      << result.max_abs_roll_deg << " deg";
  EXPECT_GE(result.final_agl_m, kTakeoffCompletionAltM)
      << model << ": final agl below takeoff completion gate";
  ExpectNoNaN(fm.GetVehicleState());
}

// 机型覆盖发动机类型（起飞逻辑按类型区分）：piston / turboprop / turbine /
// 重型 turbine / 战斗机。全为固定翼（示例的多旋翼 skip 逻辑不适用）。
INSTANTIATE_TEST_SUITE_P(
    TakeoffModels, Takeoff10msSubstepTest,
    ::testing::Values("c172x", "DHC6", "737", "B747", "f16"));

// ─── 负向回归：dt=0.1（100 ms）起飞段发散 ───────────────────────────────────

class Takeoff100msSubstepTest : public ::testing::TestWithParam<std::string> {};

/// 100 ms 子步进下起飞段必须发散：未在预算内完成，且呈现坠毁/NaN/滚转越界
/// 之一的发散特征。这是 algorithms.md「100 ms 量级在起飞段发散」的负向锚点。
/// 机型集合按 2026-08-10 全量实测探测选定：20 个固定翼机型中仅以下 11 个在
/// 100 ms 下发散（f16 坠毁时 roll 达 179.97°，即文档记录的 180° 量级特征）；
/// 其余机型在 100 ms 下仍能稳定完成（F4N/737/B747/Boeing314）或卡在地面
/// （OV10/B17/c182/L410/XB-70），均不进入本断言——避免把"发散"断言建立在
/// 未实测的行为上。若 JSBSim 版本升级改变任一机型在 100 ms 下的行为，
/// 需重新探测并同步本集合与文档。
TEST_P(Takeoff100msSubstepTest, TakeoffDivergesAt100msSubstep) {
  const std::string& model = GetParam();
  FlightManager fm(MakeGroundStartConfig(model));
  ASSERT_EQ(fm.GetState(), FlightManagerState::kReady)
      << model << ": ground-start init failed";

  PushTakeoff(fm);
  constexpr double kDt100ms = 0.1;
  const TakeoffRunResult result = RunTakeoff(fm, kDt100ms, kMaxTakeoffSteps100ms);

  EXPECT_FALSE(result.completed)
      << model << ": takeoff unexpectedly completed at 100 ms substep within "
      << kMaxTakeoffSteps100ms << " steps (300 s sim)";
  EXPECT_TRUE(result.aborted || result.nan_seen ||
              result.max_abs_roll_deg >= kMaxRollDegForStableTakeoff)
      << model << ": no divergence signature at 100 ms substep"
      << " (aborted=" << result.aborted << " nan=" << result.nan_seen
      << " max_roll=" << result.max_abs_roll_deg
      << " deg agl=" << result.final_agl_m << " m)";
}

INSTANTIATE_TEST_SUITE_P(
    DivergingTakeoffModels, Takeoff100msSubstepTest,
    ::testing::Values("c172p", "c172x", "c310", "DHC6", "C130", "A4", "F80C",
                      "f15", "f16", "Concorde", "MD11"));

}  // namespace
}  // namespace flight_dynamic
}  // namespace oneq
