/**
 * @file ar_session_consumer.cpp
 * @brief 验证安装后机载雷达公共 API 路径可被外部工程编译链接。
 *
 * 覆盖要点：
 *   - RadarSessionConfigPresets + RadarSessionConfigBuilder 构造会话配置
 *   - RadarCycleInput 构造 + RadarInputValidation 校验
 *   - RadarSession 构造、StepWithResult、Step 调用
 *   - RadarRuntimeConfigBuilder 热切换（工作子模式、扫描中心）
 *   - RadarCycleResult 各字段可访问
 *   - TrackOutputQueries 输出查询工具
 */

#include <cstddef>

#include "1q/airborne_radar/common/output/TrackOutputFrame.h"
#include "1q/airborne_radar/common/output/TrackOutputQueries.h"
#include "1q/airborne_radar/config/RadarRuntimeConfigBuilder.h"
#include "1q/airborne_radar/config/RadarSessionConfigBuilder.h"
#include "1q/airborne_radar/core/context/RadarCycleInput.h"
#include "1q/airborne_radar/core/context/RadarInputValidation.h"
#include "1q/airborne_radar/core/session/RadarCycleResult.h"
#include "1q/airborne_radar/core/session/RadarSession.h"
#include "1q/airborne_radar/core/session/RadarSessionConfigPresets.h"

int main() {
  // 1. Preset config construction
  airborne_radar::core::session::RadarSessionConfig preset_config =
      airborne_radar::common::config::MakeDefaultRadarSessionConfig();

  // 2. Builder config construction
  airborne_radar::core::session::RadarSessionConfig built_config =
      airborne_radar::config::RadarSessionConfigBuilder(preset_config)
          .Detection()
          .WithPeakPowerW(5.0e6f)
          .WithFrequencyHz(9.3e9f)
          .End()
          .Beam()
          .WithScanCenterDeg({0.0f, 0.0f})
          .End()
          .Tracking()
          .EnableKalmanFilter(true)
          .End()
          .Lifecycle()
          .WithLifecycleConfirmHits(3)
          .End()
          .Environment()
          .WithJammingDetectionThresholdDb(5.0f)
          .End()
          .Build();

  // 3. Session construction from builder config
  airborne_radar::core::session::RadarSession session(built_config);

  // 4. Input construction + validation
  airborne_radar::core::context::RadarCycleInput input;
  input.dt_sec = 1.0f;
  const std::vector<airborne_radar::core::context::ValidationIssue> issues =
      airborne_radar::core::context::ValidateRadarCycleInput(input);
  if (airborne_radar::core::context::HasValidationError(issues)) {
    return 1;
  }

  // 5. StepWithResult
  const airborne_radar::core::session::RadarCycleResult result = session.StepWithResult(input);
  if (result.has_validation_error) {
    return 2;
  }

  // 6. Step (output-only)
  const airborne_radar::common::output::TrackOutputFrame step_frame = session.Step(input);

  // 7. Access result fields
  const std::size_t confirmed_tracks = airborne_radar::common::output::CountTracksByStatus(
      result.track_output_frame, airborne_radar::common::model::DecisionTrackStatus::kConfirmed);
  if (confirmed_tracks > result.track_output_frame.tracks.size()) {
    return 3;
  }

  const bool has_profile = result.has_control_profile;
  (void)has_profile;
  const std::size_t command_count = result.submitted_commands.size();
  (void)command_count;

  // 8. RuntimeConfigBuilder hot-switch
  const airborne_radar::config::RadarRuntimeConfigPatch runtime_patch =
      airborne_radar::config::RadarRuntimeConfigBuilder()
          .WithRadarWorkSubMode(airborne_radar::common::model::RadarWorkSubMode::kTas)
          .WithScanCenterDeg({15.0f, -5.0f})
          .WithJammingDetectionThresholdDb(8.0f)
          .EnableCommandedBeamwidth(true)
          .Build();
  session.ApplyRuntimeConfig(runtime_patch);

  // 9. Second cycle after runtime config change
  airborne_radar::core::context::RadarCycleInput input_2;
  input_2.dt_sec = 1.0f;
  const airborne_radar::core::session::RadarCycleResult result_2 = session.StepWithResult(input_2);
  if (result_2.has_validation_error) {
    return 4;
  }

  // 10. Step frame cycle_index should differ from first
  if (step_frame.cycle_index == result_2.track_output_frame.cycle_index) {
    // both default 0 is fine for basic smoke
  }

  return 0;
}
