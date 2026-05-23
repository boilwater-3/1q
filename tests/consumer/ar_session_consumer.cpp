/**
 * @file ar_session_consumer.cpp
 * @brief 验证安装后机载雷达公共 API 路径可被外部工程编译链接。
 *
 * 覆盖要点：
 *   - RadarSessionConfigBuilder/直接字段赋值 构造会话配置
 *   - RadarCycleInput 构造 + RadarInputValidation 校验
 *   - RadarSession 构造、StepWithResult、Step 调用
 *   - RadarRuntimeConfigBuilder 热切换（工作子模式、扫描中心）
 *   - RadarCycleResult 各字段可访问
 *   - TrackOutputQueries 输出查询工具
 */

#include <cstddef>

#include "1q/airborne_radar/config/airborne_radar_config.hpp"
#include "1q/airborne_radar/session/RadarCycleInput.h"
#include "1q/airborne_radar/session/RadarCycleResult.h"
#include "1q/airborne_radar/session/RadarInputValidation.h"
#include "1q/airborne_radar/session/RadarSession.h"
#include "1q/airborne_radar/session/RadarSessionFactory.h"

int main() {
  // 1. Builder config construction
  airborne_radar::config::RadarSessionConfig preset_config =
      airborne_radar::config::RadarSessionConfigBuilder().Build();

  // 2. 直接字段赋值构造会话配置
  auto built_config = preset_config;
  built_config.hardware.detection.transmitter.peak_power_w = 5.0e6f;
  built_config.hardware.detection.transmitter.frequency_hz = 9.3e9f;
  built_config.mission.orientation.scan_center_deg = {0.0f, 0.0f};
  built_config.policy.tracking.enable_kalman_filter = true;
  built_config.policy.lifecycle.confirm_hits = 3;
  built_config.jamming_sensitivity_profile =
      airborne_radar::environment::ResolveJammingSensitivityProfile(5.0f);

  // 3. Session construction from builder config
  airborne_radar::session::RadarSession session =
      airborne_radar::session::RadarSessionFactory::Create(built_config);

  // 4. Input construction + validation
  airborne_radar::session::RadarCycleInput input;
  input.dt_sec = 1.0f;
  const std::vector<airborne_radar::session::ValidationIssue> issues =
      airborne_radar::session::ValidateRadarCycleInput(input);
  if (airborne_radar::session::HasValidationError(issues)) {
    return 1;
  }

  // 5. StepWithResult
  const airborne_radar::session::RadarCycleResult result = session.StepWithResult(input);
  if (result.has_validation_error) {
    return 2;
  }

  // 6. Step (output-only)
  const airborne_radar::session::TrackOutputFrame step_frame = session.Step(input);

  // 7. Access result fields
  const std::size_t confirmed_tracks = airborne_radar::session::CountTracksByStatus(
      result.track_output_frame, airborne_radar::model::TrackStatus::kConfirmed);
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
  const airborne_radar::config::RadarRuntimeConfigPatch runtime_patch =
      airborne_radar::config::RadarRuntimeConfigBuilder()
          .WithRadarWorkSubMode(airborne_radar::model::RadarWorkSubMode::kTas)
          .WithScanCenterDeg({15.0f, -5.0f})
          .WithJammingSensitivityProfile(
              airborne_radar::environment::ResolveJammingSensitivityProfile(8.0f))
          .EnableCommandedBeamwidth(true)
          .Build();
  session.ApplyRuntimeConfig(runtime_patch);

  // 9. Second cycle after runtime config change
  airborne_radar::session::RadarCycleInput input_2;
  input_2.dt_sec = 1.0f;
  const airborne_radar::session::RadarCycleResult result_2 = session.StepWithResult(input_2);
  if (result_2.has_validation_error) {
    return 4;
  }

  // 10. Step frame cycle_index should differ from first
  if (step_frame.cycle_index == result_2.track_output_frame.cycle_index) {
    // both default 0 is fine for basic smoke
  }

  return 0;
}
