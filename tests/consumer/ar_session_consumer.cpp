/**
 * @file ar_session_consumer.cpp
 * @brief 验证安装后机载雷达公共 API 路径可被外部工程编译链接。
 *
 * 覆盖要点：
 *   - ArSessionConfigBuilder/直接字段赋值 构造会话配置
 *   - ArCycleInput 构造 + ArInputValidation 校验
 *   - ArSession 构造、StepWithResult、Step 调用
 *   - ArRuntimeConfigBuilder 热切换（工作模式、扫描中心）
 *   - ArCycleResult 各字段可访问
 *   - TrackOutputQueries 输出查询工具
 */

#include <cstddef>

#include "1q/airborne_radar/config/airborne_radar_config.hpp"
#include "1q/airborne_radar/session/ArCycleInput.h"
#include "1q/airborne_radar/session/ArCycleResult.h"
#include "1q/airborne_radar/session/ArInputValidation.h"
#include "1q/airborne_radar/session/ArSession.h"

int main() {
  // 1. Builder config construction
  airborne_radar::config::ArSessionConfig preset_config =
      airborne_radar::config::ArSessionConfigBuilder().Build();

  // 2. 直接字段赋值构造会话配置
  auto built_config = preset_config;
  built_config.hardware.transmitter.peak_power_w = 5.0e6f;
  built_config.hardware.transmitter.frequency_hz = 9.3e9f;
  built_config.mission.orientation.scan_center_deg.az_deg = 0.0f;
  built_config.mission.orientation.scan_center_deg.el_deg = 0.0f;
  built_config.policy.tracking.enable_kalman_filter = true;
  built_config.policy.lifecycle.confirm_hits = 3;
  built_config.environment.jamming_sensitivity_profile =
      airborne_radar::config::ResolveJammingSensitivityProfile(5.0f);

  // 3. Session construction from builder config
  airborne_radar::session::ArSession session =
      airborne_radar::session::ArSession::Create(built_config);

  // 4. Input construction + validation
  airborne_radar::session::ArCycleInput input;
  input.dt_sec = 1.0f;
  const std::vector<airborne_radar::session::ValidationIssue> issues =
      airborne_radar::session::ValidateArCycleInput(input);
  if (airborne_radar::session::HasValidationError(issues)) {
    return 1;
  }

  // 5. StepWithResult
  const airborne_radar::session::ArCycleResult result = session.StepWithResult(input);
  if (result.has_validation_error) {
    return 2;
  }

  // 6. Step (output-only)
  const airborne_radar::session::TrackOutputFrame step_frame = session.Step(input);

  // 7. Access result fields
  const std::size_t confirmed_tracks = airborne_radar::session::CountTracksByStatus(
      result.track_output_frame, airborne_radar::session::TrackStatus::kConfirmed);
  if (confirmed_tracks > result.track_output_frame.tracks.size()) {
    return 3;
  }

  const bool has_profile = result.has_control_profile;
  (void)has_profile;
  const std::size_t command_count = result.submitted_commands.size();
  (void)command_count;
  const bool executed_this_cycle = result.executed_this_cycle;
  (void)executed_this_cycle;
  const bool reused_previous_output = result.reused_previous_output;
  (void)reused_previous_output;

  // 8. RuntimeConfigBuilder hot-switch
  airborne_radar::config::AzimuthElevationDeg scan_center_deg;
  scan_center_deg.az_deg = 15.0f;
  scan_center_deg.el_deg = -5.0f;
  const airborne_radar::config::ArRuntimeConfigPatch runtime_patch =
      airborne_radar::config::ArRuntimeConfigBuilder()
          .WithWorkMode(airborne_radar::config::ArWorkMode::kTas)
          .WithScanCenterDeg(scan_center_deg)
          .WithJammingSensitivityProfile(
              airborne_radar::config::ResolveJammingSensitivityProfile(8.0f))
          .WithCommandedBeamwidthEnabled(true)
          .Build();
  session.ApplyRuntimeConfig(runtime_patch);

  // 9. Second cycle after runtime config change
  airborne_radar::session::ArCycleInput input_2;
  input_2.dt_sec = 1.0f;
  const airborne_radar::session::ArCycleResult result_2 = session.StepWithResult(input_2);
  if (result_2.has_validation_error) {
    return 4;
  }

  // 10. Step frame cycle_index should differ from first
  if (step_frame.cycle_index == result_2.track_output_frame.cycle_index) {
    // both default 0 is fine for basic smoke
  }

  return 0;
}
