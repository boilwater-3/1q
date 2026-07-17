// 桥接守护：验证 SarCycleInputAdapter 产出的仅轨迹输入能正确进入 SarSession 内部
// raw echo 路径，而不是被误判为外部完整 IQ 并在 shape 校验处中止。
//
// 历史背景：HasExternalRawIq 曾是 6 个 raw_iq 字段的析取（pulse_count/pulse_states/
// ideal_pulse_states 任一非空即为 true）。SarCycleInputAdapter 只填伴随轨迹、不填 IQ
// 样本，却被该谓词判为"外部完整 IQ"，随后在 external_raw_iq_shape_mismatch 处中止。
// 收紧谓词（以 IQ 样本为充要条件）后，仅轨迹输入落回内部回波路径。本测试守护此契约。

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "1q/sar/config/SarSessionConfig.h"
#include "1q/sar/session/SarCycleInput.h"
#include "1q/sar/session/SarCycleInputAdapter.h"
#include "1q/sar/session/SarExternalInputAdapter.h"
#include "1q/sar/session/SarSession.h"

namespace sar {
namespace {

config::SarSessionConfig MakeSmallRdaConfig() {
  config::SarSessionConfig config;
  config.hardware.carrier_frequency_hz = 1.0e9;
  config.hardware.bandwidth_hz = 25.0e6;
  config.hardware.pulse_width_s = 0.16e-6;
  config.hardware.pulse_repetition_frequency_hz = 20.0;
  config.hardware.sample_rate_hz = 100.0e6;
  config.mission.nominal_slant_range_m = 29.9792458;
  config.mission.scene_center_latitude_deg =
      29.9792458 / 6378137.0 * 180.0 / 3.14159265358979323846;
  config.mission.platform_speed_mps = 2.0;
  config.mission.range_sample_count = 64U;
  config.mission.azimuth_pulse_count = 9U;
  config.policy.enable_l1_rda_imaging = true;
  return config;
}

// 构造位于 scene center 正南侧的 broadside 平台，便于 adapter 坐标转换。
session::SarPlatformState MakePlatformAtSceneCenter() {
  session::SarPlatformState platform;
  platform.latitude_deg = 0.0;
  platform.longitude_deg = 0.0;
  platform.altitude_m = 0.0;
  platform.velocity_north_mps = 0.0;
  platform.velocity_east_mps = 2.0;
  platform.velocity_down_mps = 0.0;
  return platform;
}

// 构造两个合法的 LLA 外部脉冲（仅运动学，无 IQ 样本）——这正是 adapter 历史上
// 会触发误判的输入形状。
std::vector<session::SarExternalPulseInput> MakeTrajectoryOnlyPulses() {
  std::vector<session::SarExternalPulseInput> pulses;
  for (std::uint64_t i = 0U; i < 2U; ++i) {
    session::SarExternalPulseInput pulse;
    pulse.pulse_id = i + 1U;
    pulse.time_s = static_cast<double>(i);
    pulse.kinematics.position_frame = oneq::coordinate::PositionFrame::kLla;
    pulse.kinematics.position_lla_deg_m.latitude_deg = 0.0;
    pulse.kinematics.position_lla_deg_m.longitude_deg = 0.001;
    pulse.kinematics.position_lla_deg_m.altitude_m = 0.0;
    pulses.push_back(pulse);
  }
  return pulses;
}

bool HasAbortCode(const session::SarCycleResult& result, const std::string& code) {
  for (const session::SarDiagnosticIssue& diagnostic : result.diagnostics) {
    if (diagnostic.code == code) {
      return true;
    }
  }
  return false;
}

TEST(SarCycleInputAdapterBridgeTest, TrajectoryOnlyInputRunsInternalEchoNotExternalIq) {
  // 仅轨迹的 adapter 输出必须进入内部 raw echo 路径并完成本周期，而非在外部 IQ
  // shape 校验处中止。
  const auto config = MakeSmallRdaConfig();
  session::SarSession session = session::SarSession::Create(config);

  session::SarCycleInput adapted;
  const auto platform = MakePlatformAtSceneCenter();
  const auto pulses = MakeTrajectoryOnlyPulses();
  ASSERT_TRUE(session::SarCycleInputAdapter::Build(platform, /*targets=*/{}, config.mission,
                                                   /*dt_sec=*/0.1f, pulses, &adapted, nullptr));

  // 适配器确认产出的是仅轨迹输入（无 IQ 样本）。
  ASSERT_NE(adapted.raw_iq.pulse_count, 0U);
  ASSERT_FALSE(adapted.raw_iq.pulse_states.empty());
  ASSERT_EQ(adapted.raw_iq.samples_per_pulse, 0U);
  ASSERT_TRUE(adapted.raw_iq.i_values.empty());
  ASSERT_TRUE(adapted.raw_iq.q_values.empty());

  const session::SarCycleResult result = session.StepWithResult(adapted);

  // 关键断言：不再触发外部 IQ shape 中止，而是走内部回波并完成。
  EXPECT_FALSE(HasAbortCode(result, "external_raw_iq_shape_mismatch"));
  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_TRUE(result.output_frame.has_raw_echo);
}

}  // namespace
}  // namespace sar
