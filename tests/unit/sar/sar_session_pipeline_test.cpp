#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "1q/sar/config/SarRuntimeConfigPatch.h"
#include "1q/sar/session/SarIssueCodes.h"
#include "1q/sar/session/SarSession.h"
#include "1q/sar/session/SarProductDebugView.h"
#include "1q/sar/session/SarProductLifecycleRecorder.h"
#include "1q/sar/session/SarRecordingSession.h"
#include "sar/session/SarReplayFlatbufferCodec.h"

namespace sar {
namespace {

bool HasIssueContaining(const session::SarCycleResult& result, const std::string& code,
                             const std::string& text) {
  for (const session::SarIssue& diagnostic : result.issues) {
    if (diagnostic.code == code && diagnostic.message.find(text) != std::string::npos) {
      return true;
    }
  }
  return false;
}

// 统一问题列表模型（规则 14）：可推导字段 has_error 已删除，遍历 issues 判定。
bool HasErrorIssue(const session::SarCycleResult& result) {
  for (const session::SarIssue& issue : result.issues) {
    if (issue.severity == session::SarIssueSeverity::kError) {
      return true;
    }
  }
  return false;
}

// Q-2 审查修复：断言存在指定 phase 的 error 级 issue
// （锁定 PhaseForAbortReason 映射，防 kExternalInputRejected 等误标）。
bool HasPhaseError(const session::SarIssueList& issues, session::SarIssuePhase phase) {
  for (const session::SarIssue& issue : issues) {
    if (issue.phase == phase && issue.severity == session::SarIssueSeverity::kError) {
      return true;
    }
  }
  return false;
}

bool HasNonZeroFocusedPixel(const session::SarFocusedImage& image) {
  for (std::size_t index = 0U; index < image.real_values.size(); ++index) {
    if (image.real_values[index] != 0.0 || image.imaginary_values[index] != 0.0) {
      return true;
    }
  }
  return false;
}

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
  config.policy.max_allowed_squint_angle_deg = 89.0;
  return config;
}

config::SarSessionConfig MakeSmallL3BpConfig() {
  config::SarSessionConfig config = MakeSmallRdaConfig();
  config.policy.max_allowed_squint_angle_deg = 89.0;
  config.policy.enable_l1_rda_imaging = false;
  config.policy.enable_l3_bp_imaging = true;
  const double meters_to_degrees = 180.0 / (3.14159265358979323846 * 6378137.0);
  config::SarWaypointConfig start;
  start.time_from_session_start_s = 0.0;
  start.longitude_deg = -0.4 * meters_to_degrees;
  config::SarWaypointConfig turn;
  turn.time_from_session_start_s = 0.2;
  config::SarWaypointConfig end;
  end.time_from_session_start_s = 1.0;
  end.longitude_deg = 1.6 * meters_to_degrees;
  end.latitude_deg = 3.0 * meters_to_degrees;
  config.mission.l3_waypoints = {start, turn, end};
  return config;
}

session::SarCycleInput MakeInput(std::uint32_t cycle_index = 1U) {
  session::SarCycleInput input;
  input.cycle_index = cycle_index;
  input.dt_sec = 0.1f;
  input.platform.latitude_deg = 0.0;
  input.platform.longitude_deg = 0.0;
  input.platform.altitude_m = 0.0;
  input.platform.velocity_east_mps = 2.0;

  session::SarPointTarget target;
  target.latitude_deg = 29.9792458 / 6378137.0 * 180.0 / 3.14159265358979323846;
  target.longitude_deg = 0.0;
  target.altitude_m = 0.0;
  target.radar_cross_section_dbsm = 80.0;
  input.point_targets.push_back(target);
  return input;
}

session::SarCycleInput MakeExternalRawIqInput() {
  session::SarCycleInput input = MakeInput();
  input.raw_iq.samples_per_pulse = 64U;
  input.raw_iq.i_values.assign(9U * 64U, 0.0);
  input.raw_iq.q_values.assign(9U * 64U, 0.0);
  for (std::size_t row = 0U; row < 9U; ++row) {
    input.raw_iq.i_values[row * 64U + 20U] = 1.0;
  }
  return input;
}

session::SarCycleInput MakeExternalRawIqInputWithTrajectory() {
  session::SarCycleInput input = MakeExternalRawIqInput();
  for (std::size_t index = 0U; index < input.raw_iq.i_values.size() / input.raw_iq.samples_per_pulse; ++index) {
    session::SarRawIqFrame::PulseState state;
    state.pulse_id = static_cast<std::uint64_t>(index);
    state.time_s = static_cast<double>(index) / 20.0;
    state.position_x_m = -0.4 + 0.1 * static_cast<double>(index);
    state.position_y_m = -29.9792458;
    state.velocity_x_mps = 2.0;
    input.raw_iq.pulse_states.push_back(state);
  }
  return input;
}

session::SarCycleInput MakeExternalRawIqInputWithDualTrajectory() {
  session::SarCycleInput input = MakeExternalRawIqInputWithTrajectory();
  input.raw_iq.ideal_pulse_states = input.raw_iq.pulse_states;
  for (session::SarRawIqFrame::PulseState& state : input.raw_iq.pulse_states) {
    state.position_y_m += 0.25;
  }
  return input;
}

TEST(SarSessionPipelineTest, StepWithResultRunsRawRangeAndRdaPipeline) {
  session::SarSession session = session::SarSession::Create(MakeSmallRdaConfig());

  const session::SarCycleResult result = session.StepWithResult(MakeInput());

  EXPECT_EQ(result.status, session::SarCycleStatus::kCompleted);
  EXPECT_FALSE(HasErrorIssue(result));
  EXPECT_TRUE(result.product.output_frame.has_raw_echo);
  EXPECT_TRUE(result.product.output_frame.has_range_compressed_echo);
  EXPECT_TRUE(result.product.output_frame.has_l1_image);
  EXPECT_TRUE(result.product.output_frame.has_image_quality_metrics);
  EXPECT_TRUE(result.product.output_frame.image_resolution_m_valid);
  EXPECT_TRUE(result.product.output_frame.phase_reference_applied);
  EXPECT_EQ(result.product.output_frame.phase_reference_mode,
            session::SarPhaseReferenceMode::kCenterBroadside);
  EXPECT_EQ(result.product.output_frame.image_quality_mainlobe_method,
            session::SarMainlobeEstimationMethod::k3dB);
  EXPECT_GT(result.product.output_frame.range_width_3db_bins, 0.0);
  EXPECT_GT(result.product.output_frame.azimuth_width_3db_bins, 0.0);
  EXPECT_GT(result.product.output_frame.range_resolution_3db_m, 0.0);
  EXPECT_GT(result.product.output_frame.azimuth_resolution_3db_m, 0.0);
  EXPECT_GE(result.product.output_frame.image_entropy_nats, 0.0);
  EXPECT_GE(result.product.output_frame.image_contrast, 0.0);
  EXPECT_EQ(result.product.output_frame.completed_stage, session::SarProcessingStage::kL1RdaImage);
  EXPECT_EQ(result.product.output_frame.range_sample_count, 64U);
  EXPECT_EQ(result.product.output_frame.azimuth_pulse_count, 9U);
  EXPECT_EQ(result.product.focused_image.source, session::SarFocusedImageSource::kL1Rda);
  EXPECT_EQ(result.product.focused_image.row_count, 9U);
  EXPECT_EQ(result.product.focused_image.column_count, 64U);
  EXPECT_EQ(result.product.focused_image.real_values.size(), 9U * 64U);
  EXPECT_EQ(result.product.focused_image.imaginary_values.size(), 9U * 64U);
  EXPECT_FALSE(result.product.focused_image.is_placeholder);
  EXPECT_TRUE(HasNonZeroFocusedPixel(result.product.focused_image));
  EXPECT_FALSE(result.issues.empty());
  // 13b 空洞条款：SAR 无逐目标门控排除，cause 恒 kNone（五模块结构同构保留字段）。
  for (const session::SarIssue& diagnostic : result.issues) {
    EXPECT_EQ(diagnostic.cause, session::SarIssueCause::kNone);
  }
  EXPECT_TRUE(HasIssueContaining(result, "sar.rda_peak", "image_entropy_nats="));
  EXPECT_TRUE(HasIssueContaining(result, "sar.rda_peak", "image_contrast="));
  EXPECT_TRUE(HasIssueContaining(result, "sar.rda_peak", "phase_reference_mode="));
  EXPECT_TRUE(HasIssueContaining(result, "sar.rda_peak", "phase_reference_applied=1"));
  EXPECT_TRUE(HasIssueContaining(result, "sar.rda_peak", "range_width_3db_bins="));
  EXPECT_TRUE(HasIssueContaining(result, "sar.rda_peak", "azimuth_width_3db_bins="));
  EXPECT_TRUE(HasIssueContaining(result, "sar.rda_peak", "range_resolution_3db_m="));
  EXPECT_TRUE(HasIssueContaining(result, "sar.rda_peak", "azimuth_resolution_3db_m="));
  EXPECT_TRUE(HasIssueContaining(result, "sar.rda_peak", "azimuth_sample_spacing_m="));
  EXPECT_TRUE(
      HasIssueContaining(result, "sar.rda_peak", "azimuth_phase_curvature_rad_per_pulse2="));
  EXPECT_TRUE(HasIssueContaining(result, "sar.rda_peak", "azimuth_quadratic_phase_span_rad="));
  EXPECT_TRUE(HasIssueContaining(result, "sar.rda_peak", "max_geometric_doppler_hz="));
  EXPECT_TRUE(HasIssueContaining(result, "sar.rda_peak", "doppler_nyquist_margin="));
}

TEST(SarSessionPipelineTest, PoweredOffCycleShortCircuitsWithEmptyFrame) {
  // 电源关闭（COMMON-OQ-4 顶层 sensor_enabled）：管线短路，输出默认空帧，
  // 关机是合法非执行状态（status=kPoweredOff），与执行失败/校验拒绝区分。
  config::SarSessionConfig config = MakeSmallRdaConfig();
  config.sensor_enabled = false;
  session::SarSession session = session::SarSession::Create(config);

  const session::SarCycleResult result = session.StepWithResult(MakeInput());

  EXPECT_EQ(result.status, session::SarCycleStatus::kPoweredOff);
  EXPECT_EQ(result.abort_reason, session::SarPipelineAbortReason::kSensorPoweredOff);
  EXPECT_NE(result.status, session::SarCycleStatus::kCompleted);
  // 严格默认空帧：元数据未写入（controller 在元数据写入前短路）。
  EXPECT_EQ(result.product.output_frame.cycle_index, 0U);
  EXPECT_EQ(result.product.output_frame.range_sample_count, 0U);
  EXPECT_FALSE(result.product.output_frame.has_raw_echo);
  EXPECT_FALSE(result.product.output_frame.has_l1_image);
  EXPECT_EQ(result.product.output_frame.completed_stage, session::SarProcessingStage::kNone);
  // focused_image 保持默认空图像（无来源、无像素；is_placeholder 默认 false，
  // 仅成功聚焦路径置 true）。
  EXPECT_EQ(result.product.focused_image.source, session::SarFocusedImageSource::kNone);
  EXPECT_TRUE(result.product.focused_image.real_values.empty());
  EXPECT_TRUE(result.product.focused_image.imaginary_values.empty());
}

TEST(SarSessionPipelineTest, PowerOffPatchThenReenabledResumesExecution) {
  // 运行期下电补丁 → 关机周期短路；重新上电补丁 → 恢复正常执行
  // （电源叶子唯一控制，COMMON-OQ-4）。
  session::SarSession session = session::SarSession::Create(MakeSmallRdaConfig());

  config::SarRuntimeConfigPatch power_off;
  power_off.has_sensor_enabled = true;
  power_off.sensor_enabled = false;
  ASSERT_TRUE(session.TryApplyRuntimeConfig(power_off));
  const session::SarCycleResult powered_off = session.StepWithResult(MakeInput());
  EXPECT_EQ(powered_off.status, session::SarCycleStatus::kPoweredOff);

  config::SarRuntimeConfigPatch power_on;
  power_on.has_sensor_enabled = true;
  power_on.sensor_enabled = true;
  ASSERT_TRUE(session.TryApplyRuntimeConfig(power_on));
  const session::SarCycleResult resumed = session.StepWithResult(MakeInput());
  EXPECT_EQ(resumed.status, session::SarCycleStatus::kCompleted);
  EXPECT_FALSE(HasErrorIssue(resumed));
  EXPECT_TRUE(resumed.product.output_frame.has_raw_echo);
}

TEST(SarSessionPipelineTest, SquintGateUsesMaximumActualApertureAngle) {
  config::SarSessionConfig config = MakeSmallRdaConfig();
  config.policy.max_allowed_squint_angle_deg = 5.9;
  session::SarCycleInput input = MakeExternalRawIqInputWithTrajectory();
  const double six_degree_offset_m =
      std::tan(6.0 * 3.14159265358979323846 / 180.0) * 29.9792458;
  for (session::SarRawIqFrame::PulseState& state : input.raw_iq.pulse_states) {
    state.position_x_m = -six_degree_offset_m;
  }

  const session::SarCycleResult rejected =
      session::SarSession::Create(config).StepWithResult(input);
  EXPECT_NE(rejected.status, session::SarCycleStatus::kCompleted);
  EXPECT_EQ(rejected.abort_reason, session::SarPipelineAbortReason::kPipelineExecutionFailed);

  config.policy.max_allowed_squint_angle_deg = 6.1;
  const session::SarCycleResult accepted =
      session::SarSession::Create(config).StepWithResult(input);
  EXPECT_EQ(accepted.status, session::SarCycleStatus::kCompleted);
}

TEST(SarSessionPipelineTest, RawEchoOnlySkipsSquintImagingGate) {
  config::SarSessionConfig config = MakeSmallRdaConfig();
  config.policy.enable_l1_rda_imaging = false;
  config.policy.max_allowed_squint_angle_deg = 0.0;
  session::SarCycleInput input = MakeInput();
  const session::SarCycleResult result =
      session::SarSession::Create(config).StepWithResult(input);
  EXPECT_EQ(result.status, session::SarCycleStatus::kCompleted);
}

// 内部生成路径：squint 门控在 raw echo 生成之前执行——被拒周期不产出 raw echo
//（输出帧仅元数据，无 raw echo 标记），接受周期正常产出（覆盖 prebuilt 轨迹
// 复用路径）。几何构造与外部路径测试一致：场景中心沿航迹偏移 tan(6°)×斜距，
// 冷启动孔径（脉冲 x∈[0, 0.8]m）下最大 squint ≈ 7.5°（含平台相对场景中心的
// 初始经度偏移，实测略高于名义 6°）。拒绝侧阈值 5.9 与接受侧阈值 8.0 之间
// 保留足够余量，避免 tan/asin 的 libm 实现差异（如 MSVC v141）导致接受分支
// 误判为超限。
TEST(SarSessionPipelineTest, SquintGatePrecedesEchoGenerationInternalPath) {
  config::SarSessionConfig config = MakeSmallRdaConfig();
  const double six_degree_offset_m =
      std::tan(6.0 * 3.14159265358979323846 / 180.0) * config.mission.nominal_slant_range_m;
  const double meters_to_degrees = 180.0 / (3.14159265358979323846 * 6378137.0);
  config.mission.scene_center_longitude_deg = -six_degree_offset_m * meters_to_degrees;

  session::SarCycleInput input = MakeInput();

  config.policy.max_allowed_squint_angle_deg = 5.9;
  const session::SarCycleResult rejected =
      session::SarSession::Create(config).StepWithResult(input);
  EXPECT_NE(rejected.status, session::SarCycleStatus::kCompleted);
  EXPECT_EQ(rejected.abort_reason, session::SarPipelineAbortReason::kPipelineExecutionFailed);
  EXPECT_TRUE(HasIssueContaining(rejected, session::codes::kSquintAngleExceedsLimit,
                                 "exceeds the configured imaging limit"));
  // 门控前置的可观察契约：被拒周期不生成 raw echo（输出帧仅元数据）。
  EXPECT_FALSE(rejected.product.output_frame.has_raw_echo);
  EXPECT_EQ(rejected.product.output_frame.completed_stage, session::SarProcessingStage::kNone);

  config.policy.max_allowed_squint_angle_deg = 8.0;
  const session::SarCycleResult accepted =
      session::SarSession::Create(config).StepWithResult(input);
  EXPECT_EQ(accepted.status, session::SarCycleStatus::kCompleted);
  EXPECT_TRUE(accepted.product.output_frame.has_raw_echo);
}

// 门控前置后：被拒周期不污染跨周期状态——同一会话下一周期（几何合规）
// 正常生成 echo 并完成成像（缓冲/分数余量由调用方快照恢复）。
TEST(SarSessionPipelineTest, SquintGateRejectionDoesNotPolluteNextCycle) {
  config::SarSessionConfig config = MakeSmallRdaConfig();
  const double six_degree_offset_m =
      std::tan(6.0 * 3.14159265358979323846 / 180.0) * config.mission.nominal_slant_range_m;
  const double meters_to_degrees = 180.0 / (3.14159265358979323846 * 6378137.0);
  config.mission.scene_center_longitude_deg = -six_degree_offset_m * meters_to_degrees;
  config.policy.max_allowed_squint_angle_deg = 5.9;

  session::SarSession session = session::SarSession::Create(config);

  // 周期 1：平台位于原点，孔径最大 squint ≈ 6.0° 超限 → 拒绝。
  const session::SarCycleResult rejected = session.StepWithResult(MakeInput(1U));
  EXPECT_NE(rejected.status, session::SarCycleStatus::kCompleted);
  EXPECT_FALSE(rejected.product.output_frame.has_raw_echo);

  // 周期 2：平台推进到与场景中心同经度（正侧视），最大 squint ≈ 1.5° → 接受。
  session::SarCycleInput input = MakeInput(2U);
  input.platform.longitude_deg = config.mission.scene_center_longitude_deg;
  const session::SarCycleResult accepted = session.StepWithResult(input);
  EXPECT_EQ(accepted.status, session::SarCycleStatus::kCompleted);
  EXPECT_TRUE(accepted.product.output_frame.has_raw_echo);
}

TEST(SarSessionPipelineTest, ProductDebugViewCarriesProductAndTargetLabels) {
  session::SarCycleInput input = MakeInput(44U);
  ASSERT_EQ(input.point_targets.size(), 1U);
  input.point_targets[0].target_id = 701U;
  input.point_targets[0].target_name = "sar-debug-point";

  session::SarCycleResult result;
  result.input_cycle_index = input.cycle_index;
  result.status = session::SarCycleStatus::kCompleted;
  result.product.output_frame.cycle_index = input.cycle_index;
  result.product.output_frame.completed_stage = session::SarProcessingStage::kL1RdaImage;
  result.product.output_frame.has_raw_echo = true;
  result.product.output_frame.has_range_compressed_echo = true;
  result.product.output_frame.has_l1_image = true;
  result.product.output_frame.estimated_snr_db = 18.0;
  result.product.output_frame.range_sample_count = 64U;
  result.product.output_frame.azimuth_pulse_count = 9U;
  result.product.focused_image.real_values.push_back(1.0);
  session::SarIssue issue;
  issue.code = "sar.test";
  issue.message = "debug";
  result.issues.push_back(issue);

  const session::SarProductDebugView view =
      session::SarProductDebugViewBuilder::Build(input, result);
  EXPECT_EQ(view.input_cycle_index, 44U);
  EXPECT_TRUE(view.has_raw_echo);
  EXPECT_TRUE(view.has_range_compressed_echo);
  EXPECT_TRUE(view.has_l1_image);
  EXPECT_TRUE(view.has_focused_pixels);
  EXPECT_DOUBLE_EQ(view.estimated_snr_db, 18.0);
  ASSERT_EQ(view.point_targets.size(), 1U);
  EXPECT_EQ(view.point_targets.front().target_id, 701U);
  EXPECT_EQ(view.point_targets.front().target_name, "sar-debug-point");
  ASSERT_EQ(view.issues.size(), 1U);
  EXPECT_EQ(view.issues.front().code, "sar.test");
}

TEST(SarSessionPipelineTest, ProductLifecycleRecorderTracksProducedUpdatedLostAndFailure) {
  session::SarProductLifecycleRecorder recorder;

  session::SarCycleResult produced;
  produced.input_cycle_index = 1U;
  produced.status = session::SarCycleStatus::kCompleted;
  produced.product.output_frame.cycle_index = 1U;
  produced.product.output_frame.completed_stage = session::SarProcessingStage::kL1RdaImage;
  produced.product.output_frame.has_l1_image = true;
  std::vector<session::SarProductLifecycleEvent> events = recorder.Update(produced);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events.front().kind, session::SarProductLifecycleEventKind::kImageProduced);

  session::SarCycleResult updated = produced;
  updated.input_cycle_index = 2U;
  updated.product.output_frame.cycle_index = 2U;
  events = recorder.Update(updated);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events.front().kind, session::SarProductLifecycleEventKind::kProductSustained);

  session::SarCycleResult lost;
  lost.input_cycle_index = 3U;
  lost.status = session::SarCycleStatus::kCompleted;
  lost.product.output_frame.cycle_index = 3U;
  lost.product.output_frame.completed_stage = session::SarProcessingStage::kRawEcho;
  events = recorder.Update(lost);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events.front().kind, session::SarProductLifecycleEventKind::kProductLost);
  EXPECT_EQ(events.front().reason, session::SarProductLifecycleReason::kNoImageProduct);

  session::SarCycleResult failed;
  failed.input_cycle_index = 4U;
  failed.status = session::SarCycleStatus::kRejectedExecution;
  failed.abort_reason = session::SarPipelineAbortReason::kPipelineExecutionFailed;
  events = recorder.Update(failed);
  EXPECT_TRUE(events.empty());

  updated.input_cycle_index = 5U;
  events = recorder.Update(updated);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events.front().kind, session::SarProductLifecycleEventKind::kImageProduced);

  session::SarProductLifecycleRecorder diagnose_recorder(
      session::SarProductLifecycleRecorderConfig{true});
  events = diagnose_recorder.Update(lost);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events.front().kind, session::SarProductLifecycleEventKind::kNoProduct);
}

TEST(SarSessionPipelineTest, ProductLifecycleRecorderPreservesStateAcrossNonExecutedCycle) {
  session::SarProductLifecycleRecorder recorder;
  session::SarCycleResult produced;
  produced.input_cycle_index = 1U;
  produced.status = session::SarCycleStatus::kCompleted;
  produced.product.output_frame.has_l1_image = true;
  ASSERT_EQ(recorder.Update(produced).front().kind,
            session::SarProductLifecycleEventKind::kImageProduced);
  session::SarCycleResult rejected;
  rejected.input_cycle_index = 2U;
  rejected.status = session::SarCycleStatus::kRejectedInvalidInput;
  EXPECT_TRUE(recorder.Update(rejected).empty());
  produced.input_cycle_index = 3U;
  const std::vector<session::SarProductLifecycleEvent> recovered = recorder.Update(produced);
  ASSERT_EQ(recovered.size(), 1U);
  EXPECT_EQ(recovered.front().kind, session::SarProductLifecycleEventKind::kProductSustained);
}

TEST(SarSessionPipelineTest, RetainFocusedImageFalseProducesPlaceholder) {
  config::SarSessionConfig config = MakeSmallRdaConfig();
  config.policy.retain_focused_image = false;
  session::SarSession session = session::SarSession::Create(config);

  const session::SarCycleResult result = session.StepWithResult(MakeInput());

  EXPECT_EQ(result.status, session::SarCycleStatus::kCompleted);
  EXPECT_FALSE(HasErrorIssue(result));
  EXPECT_TRUE(result.product.output_frame.has_l1_image);
  // 元数据仍完整，但像素数据被跳过。
  EXPECT_EQ(result.product.focused_image.source, session::SarFocusedImageSource::kL1Rda);
  EXPECT_EQ(result.product.focused_image.row_count, 9U);
  EXPECT_EQ(result.product.focused_image.column_count, 64U);
  EXPECT_TRUE(result.product.focused_image.is_placeholder);
  EXPECT_TRUE(result.product.focused_image.real_values.empty());
  EXPECT_TRUE(result.product.focused_image.imaginary_values.empty());
}

TEST(SarSessionPipelineTest, RetainFocusedImageFalseAppliesToL3Bp) {
  config::SarSessionConfig config = MakeSmallL3BpConfig();
  config.policy.retain_focused_image = false;
  session::SarSession session = session::SarSession::Create(config);

  const session::SarCycleResult result = session.StepWithResult(MakeInput());

  EXPECT_EQ(result.status, session::SarCycleStatus::kCompleted);
  EXPECT_FALSE(HasErrorIssue(result));
  EXPECT_TRUE(result.product.output_frame.has_l3_bp_image);
  EXPECT_EQ(result.product.focused_image.source, session::SarFocusedImageSource::kL3Bp);
  EXPECT_TRUE(result.product.focused_image.is_placeholder);
  EXPECT_TRUE(result.product.focused_image.real_values.empty());
}

TEST(SarSessionPipelineTest, DiagnosticsDisabledSuppressesNonErrorDiagnostics) {
  config::SarSessionConfig config = MakeSmallRdaConfig();
  config.policy.enable_diagnostics = false;
  session::SarSession session = session::SarSession::Create(config);

  const session::SarCycleResult result = session.StepWithResult(MakeInput());

  EXPECT_EQ(result.status, session::SarCycleStatus::kCompleted);
  EXPECT_FALSE(HasErrorIssue(result));
  EXPECT_TRUE(result.issues.empty());
  EXPECT_TRUE(result.product.output_frame.has_l1_image);
}

TEST(SarSessionPipelineTest, MinValidSnrRejectsApertureBelowThreshold) {
  config::SarSessionConfig config = MakeSmallRdaConfig();
  config.policy.minimum_snr_db = 200.0;
  session::SarSession session = session::SarSession::Create(config);

  const session::SarCycleResult result = session.StepWithResult(MakeInput());

  EXPECT_NE(result.status, session::SarCycleStatus::kCompleted);
  EXPECT_TRUE(HasErrorIssue(result));
  EXPECT_EQ(result.abort_reason, session::SarPipelineAbortReason::kPipelineExecutionFailed);
  EXPECT_TRUE(result.product.output_frame.has_raw_echo);
  EXPECT_FALSE(result.product.output_frame.has_l1_image);
  EXPECT_LT(result.product.output_frame.estimated_snr_db, config.policy.minimum_snr_db);
  EXPECT_TRUE(HasIssueContaining(result, "sar.snr_below_minimum", "below"));
}

TEST(SarSessionPipelineTest, HardwareLinkBudgetControlsInternalRawEchoSnr) {
  const auto run_with = [](const config::SarHardwareConfig& hardware) {
    config::SarSessionConfig config = MakeSmallRdaConfig();
    config.hardware = hardware;
    config.policy.minimum_snr_db = -1000.0;
    config.environment.surface_backscatter_sigma0_db = -300.0;
    return session::SarSession::Create(config).StepWithResult(MakeInput());
  };

  const config::SarHardwareConfig baseline = MakeSmallRdaConfig().hardware;
  const session::SarCycleResult reference = run_with(baseline);
  ASSERT_EQ(reference.status, session::SarCycleStatus::kCompleted);

  config::SarHardwareConfig higher_power = baseline;
  higher_power.peak_power_w *= 10.0;
  EXPECT_GT(run_with(higher_power).product.output_frame.estimated_snr_db,
            reference.product.output_frame.estimated_snr_db + 9.9);

  config::SarHardwareConfig higher_gain = baseline;
  higher_gain.antenna_gain_db += 10.0;
  EXPECT_GT(run_with(higher_gain).product.output_frame.estimated_snr_db,
            reference.product.output_frame.estimated_snr_db + 19.9);

  config::SarHardwareConfig higher_loss = baseline;
  higher_loss.system_loss_db += 10.0;
  EXPECT_LT(run_with(higher_loss).product.output_frame.estimated_snr_db,
            reference.product.output_frame.estimated_snr_db - 9.9);

  config::SarHardwareConfig higher_noise_figure = baseline;
  higher_noise_figure.receiver_noise_figure_db += 10.0;
  EXPECT_LT(run_with(higher_noise_figure).product.output_frame.estimated_snr_db,
            reference.product.output_frame.estimated_snr_db - 9.9);
}

TEST(SarSessionPipelineTest, AtmosphericAttenuationAppliesTwoWayLossToInternalEcho) {
  config::SarSessionConfig reference_config = MakeSmallRdaConfig();
  reference_config.policy.minimum_snr_db = -1000.0;
  reference_config.environment.surface_backscatter_sigma0_db = -300.0;
  const session::SarCycleResult reference =
      session::SarSession::Create(reference_config).StepWithResult(MakeInput());
  ASSERT_EQ(reference.status, session::SarCycleStatus::kCompleted);

  config::SarSessionConfig attenuated_config = reference_config;
  attenuated_config.environment.enable_atmospheric_attenuation = true;
  attenuated_config.environment.atmospheric_loss_db_per_km = 100.0;
  const session::SarCycleResult attenuated =
      session::SarSession::Create(attenuated_config).StepWithResult(MakeInput());
  ASSERT_EQ(attenuated.status, session::SarCycleStatus::kCompleted);

  // 目标斜距约 30 m；100 dB/km 的单程比损耗应在双程造成约 6 dB SNR 下降。
  EXPECT_NEAR(reference.product.output_frame.estimated_snr_db -
                  attenuated.product.output_frame.estimated_snr_db,
              6.0, 0.1);
}

TEST(SarSessionPipelineTest, SurfaceSigma0ControlsDistributedInternalBackground) {
  const auto run_with = [](double sigma0_db) {
    config::SarSessionConfig config = MakeSmallRdaConfig();
    config.policy.minimum_snr_db = -1000.0;
    config.environment.surface_backscatter_sigma0_db = sigma0_db;
    return session::SarSession::Create(config).StepWithResult(MakeInput());
  };

  const session::SarCycleResult low_background = run_with(-30.0);
  const session::SarCycleResult high_background = run_with(-10.0);
  ASSERT_EQ(low_background.status, session::SarCycleStatus::kCompleted);
  ASSERT_EQ(high_background.status, session::SarCycleStatus::kCompleted);
  ASSERT_TRUE(std::isfinite(low_background.product.output_frame.estimated_snr_db));
  ASSERT_TRUE(std::isfinite(high_background.product.output_frame.estimated_snr_db));
  EXPECT_NEAR(low_background.product.output_frame.estimated_snr_db -
                  high_background.product.output_frame.estimated_snr_db,
              20.0, 0.05);
}

TEST(SarSessionPipelineTest, ExternalRawIqDoesNotReapplyHardwareOrSnrGate) {
  config::SarSessionConfig config = MakeSmallRdaConfig();
  config.hardware.peak_power_w = 1.0e-9;
  config.hardware.antenna_gain_db = -100.0;
  config.hardware.receiver_noise_figure_db = 100.0;
  config.hardware.system_loss_db = 100.0;
  config.policy.minimum_snr_db = 1000.0;
  session::SarSession session = session::SarSession::Create(config);

  const session::SarCycleResult result = session.StepWithResult(MakeExternalRawIqInput());

  EXPECT_EQ(result.status, session::SarCycleStatus::kCompleted);
  EXPECT_FALSE(HasErrorIssue(result));
  EXPECT_EQ(result.product.output_frame.estimated_snr_db,
            -std::numeric_limits<double>::infinity());
  EXPECT_TRUE(HasIssueContaining(result, "sar.external_raw_iq_snr_unavailable",
                                      "not reapplied"));
}

TEST(SarSessionPipelineTest, InvalidHardwareLinkBudgetFailsBeforeRawEcho) {
  config::SarSessionConfig config = MakeSmallRdaConfig();
  config.hardware.peak_power_w = 0.0;
  session::SarSession session = session::SarSession::Create(config);

  const session::SarCycleResult result = session.StepWithResult(MakeInput());

  EXPECT_NE(result.status, session::SarCycleStatus::kCompleted);
  EXPECT_TRUE(HasErrorIssue(result));
  EXPECT_EQ(result.abort_reason, session::SarPipelineAbortReason::kValidationRejected);
  EXPECT_FALSE(result.product.output_frame.has_raw_echo);
}

TEST(SarSessionPipelineTest, EmptySceneDoesNotTripMinSnrGate) {
  config::SarSessionConfig config = MakeSmallRdaConfig();
  session::SarSession session = session::SarSession::Create(config);
  session::SarCycleInput input = MakeInput();
  input.point_targets.clear();

  const session::SarCycleResult result = session.StepWithResult(input);

  EXPECT_EQ(result.status, session::SarCycleStatus::kCompleted);
  EXPECT_FALSE(HasErrorIssue(result));
  EXPECT_TRUE(result.product.output_frame.has_raw_echo);
  EXPECT_TRUE(result.product.output_frame.has_l1_image);
  EXPECT_EQ(result.product.output_frame.estimated_snr_db, -std::numeric_limits<double>::infinity());
}

TEST(SarSessionPipelineTest, RawPulseHistoryUsesCrossCycleRingBuffer) {
  session::SarSession session = session::SarSession::Create(MakeSmallRdaConfig());

  const session::SarCycleResult first = session.StepWithResult(MakeInput(1U));
  ASSERT_EQ(first.status, session::SarCycleStatus::kCompleted);
  EXPECT_TRUE(first.product.output_frame.has_l1_image);
  EXPECT_TRUE(HasIssueContaining(first, "sar.pulse_ring_buffer", "generated=9"));

  const session::SarCycleResult second = session.StepWithResult(MakeInput(2U));
  EXPECT_EQ(second.status, session::SarCycleStatus::kCompleted);
  EXPECT_TRUE(second.product.output_frame.has_l1_image);
  EXPECT_TRUE(HasIssueContaining(second, "sar.pulse_ring_buffer", "generated=2"));
  EXPECT_TRUE(HasIssueContaining(second, "sar.pulse_ring_buffer", "overflow=true"));
}

TEST(SarSessionPipelineTest, ExternalRawIqRunsL1RdaAndReturnsFocusedImage) {
  session::SarSession session = session::SarSession::Create(MakeSmallRdaConfig());

  const session::SarCycleResult result = session.StepWithResult(MakeExternalRawIqInput());

  EXPECT_EQ(result.status, session::SarCycleStatus::kCompleted);
  EXPECT_FALSE(HasErrorIssue(result));
  EXPECT_TRUE(result.product.output_frame.has_raw_echo);
  EXPECT_TRUE(result.product.output_frame.has_l1_image);
  EXPECT_EQ(result.product.focused_image.source, session::SarFocusedImageSource::kL1Rda);
  EXPECT_TRUE(HasNonZeroFocusedPixel(result.product.focused_image));
  EXPECT_TRUE(HasIssueContaining(result, "sar.external_raw_iq", "pulses=9"));
  EXPECT_FALSE(HasIssueContaining(result, "sar.pulse_ring_buffer", ""));
}

TEST(SarSessionPipelineTest, RecordingSessionWithoutReplayWriterAcceptsExternalRawIq) {
  session::SarRecordingSession session(MakeSmallRdaConfig());

  const session::SarCycleResult result = session.StepWithResult(MakeExternalRawIqInput());

  EXPECT_EQ(result.status, session::SarCycleStatus::kCompleted);
  EXPECT_FALSE(HasErrorIssue(result));
  EXPECT_TRUE(result.product.output_frame.has_l1_image);
  EXPECT_EQ(result.product.focused_image.source, session::SarFocusedImageSource::kL1Rda);
}

TEST(SarSessionPipelineTest, ReplayCodecPreservesExternalRawIq) {
  const session::SarCycleInput input = MakeExternalRawIqInputWithDualTrajectory();
  const std::string encoded = session::EncodeSarCycleInput(input);
  ASSERT_FALSE(encoded.empty());
  session::SarCycleInput decoded;
  ASSERT_TRUE(session::DecodeSarCycleInput(encoded, &decoded));
  EXPECT_EQ(decoded.raw_iq.samples_per_pulse, input.raw_iq.samples_per_pulse);
  EXPECT_EQ(decoded.raw_iq.i_values, input.raw_iq.i_values);
  EXPECT_EQ(decoded.raw_iq.q_values, input.raw_iq.q_values);
  EXPECT_EQ(decoded.raw_iq.pulse_states.size(), input.raw_iq.pulse_states.size());
  EXPECT_EQ(decoded.raw_iq.ideal_pulse_states.size(), input.raw_iq.ideal_pulse_states.size());
}

TEST(SarSessionPipelineTest, ExternalRawIqRejectsShapeMismatchAndAdvancedPaths) {
  session::SarCycleInput malformed = MakeExternalRawIqInput();
  malformed.raw_iq.i_values.pop_back();
  session::SarSession malformed_session = session::SarSession::Create(MakeSmallRdaConfig());
  const session::SarCycleResult malformed_result = malformed_session.StepWithResult(malformed);
  EXPECT_NE(malformed_result.status, session::SarCycleStatus::kCompleted);
  EXPECT_EQ(malformed_result.abort_reason, session::SarPipelineAbortReason::kExternalInputRejected);
  // 外部 IQ 拒绝走 RecordAbort→PhaseForAbortReason：kExternalInputRejected→kInputValidation。
  EXPECT_TRUE(HasPhaseError(malformed_result.issues, session::SarIssuePhase::kInputValidation));

  session::SarCycleInput non_finite = MakeExternalRawIqInput();
  non_finite.raw_iq.q_values[0] = std::numeric_limits<double>::quiet_NaN();
  session::SarSession non_finite_session = session::SarSession::Create(MakeSmallRdaConfig());
  const session::SarCycleResult non_finite_result = non_finite_session.StepWithResult(non_finite);
  EXPECT_NE(non_finite_result.status, session::SarCycleStatus::kCompleted);
  EXPECT_EQ(non_finite_result.abort_reason, session::SarPipelineAbortReason::kExternalInputRejected);
  EXPECT_TRUE(HasPhaseError(non_finite_result.issues, session::SarIssuePhase::kInputValidation));

  config::SarSessionConfig l2_config = MakeSmallRdaConfig();
  l2_config.policy.enable_l2_motion_compensation = true;
  l2_config.policy.max_allowed_squint_angle_deg = 89.0;
  session::SarSession l2_session = session::SarSession::Create(l2_config);
  const session::SarCycleResult l2_result = l2_session.StepWithResult(MakeExternalRawIqInput());
  EXPECT_NE(l2_result.status, session::SarCycleStatus::kCompleted);
  EXPECT_EQ(l2_result.abort_reason, session::SarPipelineAbortReason::kExternalInputRejected);
  EXPECT_TRUE(HasPhaseError(l2_result.issues, session::SarIssuePhase::kInputValidation));
}

TEST(SarSessionPipelineTest, ExternalRawIqWithPulseStatesRunsL3Bp) {
  session::SarSession session = session::SarSession::Create(MakeSmallL3BpConfig());

  const session::SarCycleResult result =
      session.StepWithResult(MakeExternalRawIqInputWithTrajectory());

  EXPECT_EQ(result.status, session::SarCycleStatus::kCompleted);
  EXPECT_FALSE(HasErrorIssue(result));
  EXPECT_TRUE(result.product.output_frame.has_l3_bp_image);
  EXPECT_EQ(result.product.focused_image.source, session::SarFocusedImageSource::kL3Bp);
  EXPECT_TRUE(HasNonZeroFocusedPixel(result.product.focused_image));
  EXPECT_TRUE(HasIssueContaining(result, "sar.external_raw_iq", "pulses=9"));
  EXPECT_FALSE(HasIssueContaining(result, "sar.l3_trajectory", ""));
}

TEST(SarSessionPipelineTest, ExternalRawIqL1ExplicitlyIgnoresPulseStates) {
  session::SarSession session = session::SarSession::Create(MakeSmallRdaConfig());

  const session::SarCycleResult result =
      session.StepWithResult(MakeExternalRawIqInputWithTrajectory());

  EXPECT_EQ(result.status, session::SarCycleStatus::kCompleted);
  EXPECT_FALSE(HasErrorIssue(result));
  EXPECT_TRUE(result.product.output_frame.has_l1_image);
  EXPECT_TRUE(HasIssueContaining(result, "sar.external_raw_iq_trajectory_ignored", ""));
}

TEST(SarSessionPipelineTest, ExternalRawIqDualTrajectoryRunsL2MotionCompensation) {
  config::SarSessionConfig config = MakeSmallRdaConfig();
  config.policy.enable_l2_motion_compensation = true;
  config.policy.max_allowed_squint_angle_deg = 89.0;
  session::SarSession session = session::SarSession::Create(config);

  const session::SarCycleResult result =
      session.StepWithResult(MakeExternalRawIqInputWithDualTrajectory());

  EXPECT_EQ(result.status, session::SarCycleStatus::kCompleted);
  EXPECT_FALSE(HasErrorIssue(result));
  EXPECT_TRUE(result.product.output_frame.has_l1_image);
  EXPECT_EQ(result.product.focused_image.source, session::SarFocusedImageSource::kL1Rda);
  EXPECT_TRUE(HasIssueContaining(result, "sar.motion_compensation", "max_abs_range_error_m="));
  EXPECT_FALSE(HasIssueContaining(result, "sar.external_raw_iq_trajectory_ignored", ""));
}

TEST(SarSessionPipelineTest, ExternalRawIqL2RejectsMissingOrInvalidIdealTrajectory) {
  config::SarSessionConfig config = MakeSmallRdaConfig();
  config.policy.enable_l2_motion_compensation = true;

  session::SarSession missing_session = session::SarSession::Create(config);
  const session::SarCycleResult missing_result =
      missing_session.StepWithResult(MakeExternalRawIqInputWithTrajectory());
  EXPECT_NE(missing_result.status, session::SarCycleStatus::kCompleted);
  EXPECT_EQ(missing_result.abort_reason, session::SarPipelineAbortReason::kExternalInputRejected);
  EXPECT_TRUE(HasPhaseError(missing_result.issues, session::SarIssuePhase::kInputValidation));

  session::SarCycleInput invalid = MakeExternalRawIqInputWithDualTrajectory();
  invalid.raw_iq.ideal_pulse_states[3].time_s = invalid.raw_iq.ideal_pulse_states[2].time_s;
  session::SarSession invalid_session = session::SarSession::Create(config);
  const session::SarCycleResult invalid_result = invalid_session.StepWithResult(invalid);
  EXPECT_NE(invalid_result.status, session::SarCycleStatus::kCompleted);
  EXPECT_EQ(invalid_result.abort_reason, session::SarPipelineAbortReason::kExternalInputRejected);
  EXPECT_TRUE(HasPhaseError(invalid_result.issues, session::SarIssuePhase::kInputValidation));
}

TEST(SarSessionPipelineTest, ExternalRawIqBpRejectsMissingOrInvalidTrajectory) {
  session::SarSession missing_session = session::SarSession::Create(MakeSmallL3BpConfig());
  const session::SarCycleResult missing_result =
      missing_session.StepWithResult(MakeExternalRawIqInput());
  EXPECT_NE(missing_result.status, session::SarCycleStatus::kCompleted);
  EXPECT_EQ(missing_result.abort_reason, session::SarPipelineAbortReason::kExternalInputRejected);
  EXPECT_TRUE(HasPhaseError(missing_result.issues, session::SarIssuePhase::kInputValidation));

  session::SarCycleInput invalid = MakeExternalRawIqInputWithTrajectory();
  invalid.raw_iq.pulse_states[2].pulse_id = invalid.raw_iq.pulse_states[1].pulse_id;
  session::SarSession invalid_session = session::SarSession::Create(MakeSmallL3BpConfig());
  const session::SarCycleResult invalid_result = invalid_session.StepWithResult(invalid);
  EXPECT_NE(invalid_result.status, session::SarCycleStatus::kCompleted);
  EXPECT_EQ(invalid_result.abort_reason, session::SarPipelineAbortReason::kValidationRejected);
  EXPECT_TRUE(HasIssueContaining(invalid_result, "sar.validation.invalid_pulse_sequence",
                                 "contiguous"));

  session::SarCycleInput non_finite = MakeExternalRawIqInputWithTrajectory();
  non_finite.raw_iq.pulse_states[0].position_y_m = std::numeric_limits<double>::infinity();
  session::SarSession non_finite_session =
      session::SarSession::Create(MakeSmallL3BpConfig());
  const session::SarCycleResult non_finite_result = non_finite_session.StepWithResult(non_finite);
  EXPECT_NE(non_finite_result.status, session::SarCycleStatus::kCompleted);
  EXPECT_EQ(non_finite_result.abort_reason, session::SarPipelineAbortReason::kValidationRejected);
  EXPECT_TRUE(HasIssueContaining(non_finite_result, "sar.validation.non_finite_pulse_field",
                                 "non-finite"));
}

TEST(SarSessionPipelineTest, RuntimeSizeGateRejectsUnapprovedLargeRda) {
  config::SarSessionConfig config = MakeSmallRdaConfig();
  config.mission.range_sample_count = 2048U;
  config.mission.azimuth_pulse_count = 1024U;
  session::SarSession session = session::SarSession::Create(config);

  const session::SarCycleResult result = session.StepWithResult(MakeInput());

  EXPECT_NE(result.status, session::SarCycleStatus::kCompleted);
  EXPECT_TRUE(HasErrorIssue(result));
  EXPECT_EQ(result.abort_reason, session::SarPipelineAbortReason::kValidationRejected);
  EXPECT_FALSE(result.product.output_frame.has_l1_image);
}

TEST(SarSessionPipelineTest, RdaRequiresRawEcho) {
  config::SarSessionConfig config = MakeSmallRdaConfig();
  config.policy.enable_raw_echo_generation = false;
  const session::SarCycleResult result =
      session::SarSession::Create(config).StepWithResult(MakeInput());

  EXPECT_NE(result.status, session::SarCycleStatus::kCompleted);
  EXPECT_TRUE(HasErrorIssue(result));
  EXPECT_EQ(result.abort_reason, session::SarPipelineAbortReason::kValidationRejected);
  EXPECT_FALSE(result.product.output_frame.has_range_compressed_echo);
  EXPECT_FALSE(result.product.output_frame.has_l1_image);
}

TEST(SarSessionPipelineTest, RangeCompressionStatusRequiresExecutedImaging) {
  config::SarSessionConfig no_raw = MakeSmallRdaConfig();
  no_raw.policy.enable_raw_echo_generation = false;
  no_raw.policy.enable_l1_rda_imaging = false;
  const session::SarCycleResult no_raw_result =
      session::SarSession::Create(no_raw).StepWithResult(MakeInput());
  EXPECT_EQ(no_raw_result.status, session::SarCycleStatus::kCompleted);
  EXPECT_FALSE(no_raw_result.product.output_frame.has_raw_echo);
  EXPECT_FALSE(no_raw_result.product.output_frame.has_range_compressed_echo);
  EXPECT_EQ(no_raw_result.product.output_frame.completed_stage, session::SarProcessingStage::kNone);

  config::SarSessionConfig raw_only = MakeSmallRdaConfig();
  raw_only.policy.enable_l1_rda_imaging = false;
  const session::SarCycleResult raw_only_result =
      session::SarSession::Create(raw_only).StepWithResult(MakeInput());
  EXPECT_EQ(raw_only_result.status, session::SarCycleStatus::kCompleted);
  EXPECT_TRUE(raw_only_result.product.output_frame.has_raw_echo);
  EXPECT_FALSE(raw_only_result.product.output_frame.has_range_compressed_echo);
  EXPECT_EQ(raw_only_result.product.output_frame.completed_stage, session::SarProcessingStage::kRawEcho);
}

TEST(SarSessionPipelineTest, L2MotionCompensationIsDefaultOffAndRunsWhenExplicitlyEnabled) {
  config::SarSessionConfig l1_config = MakeSmallRdaConfig();
  EXPECT_FALSE(l1_config.policy.enable_l2_motion_compensation);
  session::SarSession l1_session = session::SarSession::Create(l1_config);
  const session::SarCycleResult l1_result = l1_session.StepWithResult(MakeInput());
  ASSERT_EQ(l1_result.status, session::SarCycleStatus::kCompleted);
  EXPECT_FALSE(HasIssueContaining(l1_result, "sar.l2_trajectory", ""));
  EXPECT_FALSE(HasIssueContaining(l1_result, "sar.motion_compensation", ""));

  config::SarSessionConfig l2_config = MakeSmallRdaConfig();
  l2_config.policy.enable_l2_motion_compensation = true;
  l2_config.mission.l2_velocity_error_stddev_y_mps = 30.0;
  l2_config.mission.l2_velocity_error_stddev_z_mps = 10.0;
  l2_config.mission.l2_random_seed = 2026U;
  session::SarSession l2_session = session::SarSession::Create(l2_config);
  const session::SarCycleResult l2_result = l2_session.StepWithResult(MakeInput());

  EXPECT_EQ(l2_result.status, session::SarCycleStatus::kCompleted);
  EXPECT_FALSE(HasErrorIssue(l2_result));
  EXPECT_TRUE(l2_result.product.output_frame.has_l1_image);
  EXPECT_TRUE(HasIssueContaining(l2_result, "sar.l2_trajectory", "max_position_error_m="));
  EXPECT_TRUE(
      HasIssueContaining(l2_result, "sar.motion_compensation", "max_abs_range_error_m="));
}

TEST(SarSessionPipelineTest, ZeroPerturbationL2StrictlyDegradesToL1Trajectory) {
  session::SarSession l1_session = session::SarSession::Create(MakeSmallRdaConfig());
  const session::SarCycleResult l1_result = l1_session.StepWithResult(MakeInput());
  ASSERT_EQ(l1_result.status, session::SarCycleStatus::kCompleted);

  config::SarSessionConfig config = MakeSmallRdaConfig();
  config.policy.enable_l2_motion_compensation = true;
  session::SarSession session = session::SarSession::Create(config);

  const session::SarCycleResult result = session.StepWithResult(MakeInput());

  ASSERT_EQ(result.status, session::SarCycleStatus::kCompleted);
  EXPECT_FALSE(HasErrorIssue(result));
  EXPECT_TRUE(result.product.output_frame.has_l1_image);
  EXPECT_EQ(result.product.output_frame.cycle_index, l1_result.product.output_frame.cycle_index);
  EXPECT_EQ(result.product.output_frame.completed_stage, l1_result.product.output_frame.completed_stage);
  EXPECT_EQ(result.product.output_frame.range_sample_count, l1_result.product.output_frame.range_sample_count);
  EXPECT_EQ(result.product.output_frame.azimuth_pulse_count, l1_result.product.output_frame.azimuth_pulse_count);
  EXPECT_EQ(result.product.output_frame.center_slant_range_m, l1_result.product.output_frame.center_slant_range_m);
  EXPECT_EQ(result.product.output_frame.estimated_snr_db, l1_result.product.output_frame.estimated_snr_db);
  EXPECT_EQ(result.product.output_frame.has_raw_echo, l1_result.product.output_frame.has_raw_echo);
  EXPECT_EQ(result.product.output_frame.has_range_compressed_echo,
            l1_result.product.output_frame.has_range_compressed_echo);
  EXPECT_EQ(result.product.output_frame.has_l1_image, l1_result.product.output_frame.has_l1_image);
  EXPECT_TRUE(
      HasIssueContaining(result, "sar.l2_trajectory", "max_position_error_m=0.000000"));
  EXPECT_TRUE(
      HasIssueContaining(result, "sar.l2_trajectory", "rms_position_error_m=0.000000"));
  EXPECT_TRUE(
      HasIssueContaining(result, "sar.motion_compensation", "max_abs_range_error_m=0.000000"));
  EXPECT_TRUE(
      HasIssueContaining(result, "sar.motion_compensation", "rms_range_error_m=0.000000"));
}

TEST(SarSessionPipelineTest, L2TrajectoryHistoryRemainsAlignedAcrossCycles) {
  config::SarSessionConfig config = MakeSmallRdaConfig();
  config.policy.enable_l2_motion_compensation = true;
  config.mission.l2_velocity_error_stddev_y_mps = 30.0;
  config.mission.l2_velocity_error_stddev_z_mps = 10.0;
  config.mission.l2_random_seed = 2026U;
  session::SarSession session = session::SarSession::Create(config);

  const session::SarCycleResult first = session.StepWithResult(MakeInput(1U));
  ASSERT_EQ(first.status, session::SarCycleStatus::kCompleted);
  EXPECT_FALSE(HasErrorIssue(first));
  EXPECT_TRUE(HasIssueContaining(first, "sar.pulse_ring_buffer", "generated=9"));
  EXPECT_TRUE(HasIssueContaining(first, "sar.motion_compensation", ""));

  const session::SarCycleResult second = session.StepWithResult(MakeInput(2U));
  EXPECT_EQ(second.status, session::SarCycleStatus::kCompleted);
  EXPECT_FALSE(HasErrorIssue(second));
  EXPECT_TRUE(second.product.output_frame.has_l1_image);
  EXPECT_TRUE(HasIssueContaining(second, "sar.pulse_ring_buffer", "generated=2"));
  EXPECT_TRUE(HasIssueContaining(second, "sar.pulse_ring_buffer", "overflow=true"));
  EXPECT_TRUE(HasIssueContaining(second, "sar.l2_trajectory", "max_position_error_m="));
  EXPECT_TRUE(HasIssueContaining(second, "sar.motion_compensation", "max_abs_range_error_m="));
}

TEST(SarSessionPipelineTest, L2MotionCompensationRequiresRawEchoAndRda) {
  config::SarSessionConfig config = MakeSmallRdaConfig();
  config.policy.enable_l2_motion_compensation = true;
  config.policy.enable_l1_rda_imaging = false;
  session::SarSession session = session::SarSession::Create(config);

  const session::SarCycleResult result = session.StepWithResult(MakeInput());

  EXPECT_NE(result.status, session::SarCycleStatus::kCompleted);
  EXPECT_TRUE(HasErrorIssue(result));
  EXPECT_EQ(result.abort_reason, session::SarPipelineAbortReason::kValidationRejected);
}

TEST(SarSessionPipelineTest, L3BpRunsOnlyWhenExplicitlyEnabled) {
  session::SarSession session = session::SarSession::Create(MakeSmallL3BpConfig());

  const session::SarCycleResult result = session.StepWithResult(MakeInput());

  EXPECT_EQ(result.status, session::SarCycleStatus::kCompleted);
  EXPECT_FALSE(HasErrorIssue(result));
  EXPECT_TRUE(result.product.output_frame.has_raw_echo);
  EXPECT_TRUE(result.product.output_frame.has_range_compressed_echo);
  EXPECT_FALSE(result.product.output_frame.has_l1_image);
  EXPECT_TRUE(result.product.output_frame.has_l3_bp_image);
  EXPECT_TRUE(result.product.output_frame.has_image_quality_metrics);
  EXPECT_FALSE(result.product.output_frame.image_resolution_m_valid);
  EXPECT_FALSE(result.product.output_frame.phase_reference_applied);
  EXPECT_EQ(result.product.output_frame.phase_reference_mode, session::SarPhaseReferenceMode::kNative);
  EXPECT_EQ(result.product.output_frame.image_quality_mainlobe_method,
            session::SarMainlobeEstimationMethod::k3dB);
  EXPECT_GT(result.product.output_frame.range_width_3db_bins, 0.0);
  EXPECT_GT(result.product.output_frame.azimuth_width_3db_bins, 0.0);
  EXPECT_GE(result.product.output_frame.image_entropy_nats, 0.0);
  EXPECT_GE(result.product.output_frame.image_contrast, 0.0);
  EXPECT_EQ(result.product.output_frame.completed_stage, session::SarProcessingStage::kL3BpImage);
  EXPECT_EQ(result.product.focused_image.source, session::SarFocusedImageSource::kL3Bp);
  EXPECT_EQ(result.product.focused_image.row_count, 9U);
  EXPECT_EQ(result.product.focused_image.column_count, 64U);
  EXPECT_EQ(result.product.focused_image.real_values.size(), 9U * 64U);
  EXPECT_EQ(result.product.focused_image.imaginary_values.size(), 9U * 64U);
  EXPECT_FALSE(result.product.focused_image.is_placeholder);
  EXPECT_TRUE(HasNonZeroFocusedPixel(result.product.focused_image));
  EXPECT_TRUE(HasIssueContaining(result, "sar.l3_trajectory", "generated=9"));
  EXPECT_TRUE(HasIssueContaining(result, "sar.bp_peak", "peak_row="));
  EXPECT_TRUE(HasIssueContaining(result, "sar.bp_traversal", "pulse_major"));
}

TEST(SarSessionPipelineTest, L3BpRejectsMutualExclusionAndSizeViolations) {
  config::SarSessionConfig mutual_exclusion = MakeSmallL3BpConfig();
  mutual_exclusion.policy.enable_l1_rda_imaging = true;
  session::SarSession mutual_session = session::SarSession::Create(mutual_exclusion);
  const session::SarCycleResult mutual_result = mutual_session.StepWithResult(MakeInput());
  EXPECT_NE(mutual_result.status, session::SarCycleStatus::kCompleted);
  EXPECT_EQ(mutual_result.abort_reason, session::SarPipelineAbortReason::kValidationRejected);

  config::SarSessionConfig oversized = MakeSmallL3BpConfig();
  oversized.mission.range_sample_count = 129U;
  session::SarSession oversized_session = session::SarSession::Create(oversized);
  const session::SarCycleResult oversized_result = oversized_session.StepWithResult(MakeInput());
  EXPECT_NE(oversized_result.status, session::SarCycleStatus::kCompleted);
  EXPECT_EQ(oversized_result.abort_reason, session::SarPipelineAbortReason::kValidationRejected);
}

TEST(SarSessionPipelineTest, L3BpRequiresRawEcho) {
  config::SarSessionConfig no_raw = MakeSmallL3BpConfig();
  no_raw.policy.enable_raw_echo_generation = false;
  session::SarSession no_raw_session = session::SarSession::Create(no_raw);
  const session::SarCycleResult no_raw_result = no_raw_session.StepWithResult(MakeInput());
  EXPECT_NE(no_raw_result.status, session::SarCycleStatus::kCompleted);
  EXPECT_EQ(no_raw_result.abort_reason, session::SarPipelineAbortReason::kValidationRejected);
}

TEST(SarSessionPipelineTest, L3BpRejectsInvalidWaypointStructure) {
  config::SarSessionConfig nonzero_start = MakeSmallL3BpConfig();
  nonzero_start.mission.l3_waypoints.front().time_from_session_start_s = 0.01;
  session::SarSession nonzero_start_session = session::SarSession::Create(nonzero_start);
  const session::SarCycleResult nonzero_start_result =
      nonzero_start_session.StepWithResult(MakeInput());
  EXPECT_NE(nonzero_start_result.status, session::SarCycleStatus::kCompleted);
  EXPECT_EQ(nonzero_start_result.abort_reason, session::SarPipelineAbortReason::kValidationRejected);

  config::SarSessionConfig nonmonotonic = MakeSmallL3BpConfig();
  nonmonotonic.mission.l3_waypoints.back().time_from_session_start_s =
      nonmonotonic.mission.l3_waypoints[1].time_from_session_start_s;
  session::SarSession nonmonotonic_session = session::SarSession::Create(nonmonotonic);
  const session::SarCycleResult nonmonotonic_result =
      nonmonotonic_session.StepWithResult(MakeInput());
  EXPECT_NE(nonmonotonic_result.status, session::SarCycleStatus::kCompleted);
  EXPECT_EQ(nonmonotonic_result.abort_reason, session::SarPipelineAbortReason::kValidationRejected);
}

TEST(SarSessionPipelineTest, L3BpTrajectoryHistoryRemainsAlignedAcrossCycles) {
  session::SarSession session = session::SarSession::Create(MakeSmallL3BpConfig());

  const session::SarCycleResult first = session.StepWithResult(MakeInput(1U));
  ASSERT_EQ(first.status, session::SarCycleStatus::kCompleted);
  ASSERT_FALSE(HasErrorIssue(first));
  EXPECT_TRUE(first.product.output_frame.has_l3_bp_image);
  EXPECT_TRUE(HasIssueContaining(first, "sar.l3_trajectory", "generated=9"));
  EXPECT_TRUE(HasIssueContaining(first, "sar.l3_trajectory", "last_time_s=0.400000"));

  const session::SarCycleResult second = session.StepWithResult(MakeInput(2U));
  EXPECT_EQ(second.status, session::SarCycleStatus::kCompleted);
  EXPECT_FALSE(HasErrorIssue(second));
  EXPECT_TRUE(second.product.output_frame.has_l3_bp_image);
  EXPECT_TRUE(HasIssueContaining(second, "sar.pulse_ring_buffer", "generated=2"));
  EXPECT_TRUE(HasIssueContaining(second, "sar.pulse_ring_buffer", "overflow=true"));
  EXPECT_TRUE(HasIssueContaining(second, "sar.l3_trajectory", "first_time_s=0.450000"));
  EXPECT_TRUE(HasIssueContaining(second, "sar.l3_trajectory", "last_time_s=0.500000"));
  EXPECT_TRUE(HasIssueContaining(second, "sar.bp_traversal", "pulse_major"));
}

TEST(SarSessionPipelineTest, L3BpRejectsWaypointCoverageGap) {
  config::SarSessionConfig config = MakeSmallL3BpConfig();
  config.mission.l3_waypoints.back().time_from_session_start_s = 0.2;
  config.mission.l3_waypoints[1].time_from_session_start_s = 0.1;
  session::SarSession session = session::SarSession::Create(config);

  const session::SarCycleResult result = session.StepWithResult(MakeInput());

  EXPECT_NE(result.status, session::SarCycleStatus::kCompleted);
  EXPECT_TRUE(HasErrorIssue(result));
  EXPECT_EQ(result.abort_reason, session::SarPipelineAbortReason::kPipelineExecutionFailed);
}

TEST(SarSessionPipelineTest, InvalidCycleReturnsEmptyOutputNotReused) {
  session::SarSession session = session::SarSession::Create(MakeSmallRdaConfig());

  const session::SarCycleResult first = session.StepWithResult(MakeInput(3U));
  ASSERT_EQ(first.status, session::SarCycleStatus::kCompleted);
  session::SarCycleInput invalid = MakeInput(4U);
  invalid.dt_sec = 0.0f;
  const session::SarCycleResult second = session.StepWithResult(invalid);

  EXPECT_NE(second.status, session::SarCycleStatus::kCompleted);
  EXPECT_TRUE(HasErrorIssue(second));
  EXPECT_EQ(second.product.output_frame.cycle_index, 0U);
  EXPECT_FALSE(second.product.output_frame.has_l1_image);
  EXPECT_EQ(second.product.focused_image.source, session::SarFocusedImageSource::kNone);
  EXPECT_TRUE(second.product.focused_image.real_values.empty());
  EXPECT_TRUE(second.product.focused_image.imaginary_values.empty());
}

TEST(SarSessionPipelineTest, InvalidTargetInputAbortsBeforeImaging) {
  session::SarSession session = session::SarSession::Create(MakeSmallRdaConfig());
  session::SarCycleInput input = MakeInput();
  input.point_targets[0].radar_cross_section_dbsm = std::numeric_limits<double>::quiet_NaN();

  const session::SarCycleResult result = session.StepWithResult(input);

  EXPECT_NE(result.status, session::SarCycleStatus::kCompleted);
  EXPECT_TRUE(HasErrorIssue(result));
  EXPECT_EQ(result.abort_reason, session::SarPipelineAbortReason::kValidationRejected);
  EXPECT_FALSE(result.product.output_frame.has_l1_image);
  EXPECT_TRUE(HasIssueContaining(result, "sar.validation.non_finite_target_field",
                                  "non-finite"));
  EXPECT_EQ(result.product.focused_image.source, session::SarFocusedImageSource::kNone);
}

// 运行期配置提交策略契约（docs/common/contract.md）：SAR 属立即提交类——
// TryApplyRuntimeConfig 调用即生效、单向落定、无 session 层回滚。有效 patch 在
// 下个 Step 立即生效；无效 patch 经 resolver 拒绝且不污染 runtime_config。

TEST(SarSessionPipelineTest, ValidRuntimePatchTakesEffectImmediately) {
  session::SarSession session = session::SarSession::Create(MakeSmallRdaConfig());
  const session::SarCycleResult baseline = session.StepWithResult(MakeInput());
  ASSERT_EQ(baseline.status, session::SarCycleStatus::kCompleted);
  // 默认 retain_focused_image=true，基线输出应含非占位聚焦图像。
  ASSERT_FALSE(baseline.product.focused_image.is_placeholder);

  // 立即提交：关闭 retain_focused_image，下个 Step 立即反映。
  config::SarRuntimeConfigPatch patch;
  patch.has_retain_focused_image = true;
  patch.retain_focused_image = false;
  EXPECT_TRUE(session.TryApplyRuntimeConfig(patch));

  const session::SarCycleResult updated = session.StepWithResult(MakeInput(2U));
  EXPECT_EQ(updated.status, session::SarCycleStatus::kCompleted);
  EXPECT_TRUE(updated.product.focused_image.is_placeholder);
}

TEST(SarSessionPipelineTest, InvalidRuntimePatchRejectedWithoutPollutingConfig) {
  session::SarSession session = session::SarSession::Create(MakeSmallRdaConfig());
  const session::SarCycleResult baseline = session.StepWithResult(MakeInput());
  ASSERT_EQ(baseline.status, session::SarCycleStatus::kCompleted);
  ASSERT_FALSE(baseline.product.focused_image.is_placeholder);

  // 无效 patch（NaN minimum_snr_db）应被 resolver 拒绝。
  config::SarRuntimeConfigPatch invalid_patch;
  invalid_patch.has_minimum_snr_db = true;
  invalid_patch.minimum_snr_db = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(session.TryApplyRuntimeConfig(invalid_patch));

  // 拒绝后 runtime_config 不被污染：retain_focused_image 仍为默认 true。
  const session::SarCycleResult after_invalid = session.StepWithResult(MakeInput(2U));
  EXPECT_EQ(after_invalid.status, session::SarCycleStatus::kCompleted);
  EXPECT_FALSE(after_invalid.product.focused_image.is_placeholder);
}

TEST(SarSessionPipelineTest, RuntimePatchViolatingL1RdaDependencyRejected) {
  // 当前 config 启用 L1 RDA（依赖 raw echo）。补丁显式关闭 raw echo 应被拒绝，
  // 因 resolver 前置了 L1-RDA-requires-raw-echo 不变式。
  session::SarSession session = session::SarSession::Create(MakeSmallRdaConfig());
  config::SarRuntimeConfigPatch patch;
  patch.has_enable_raw_echo_generation = true;
  patch.enable_raw_echo_generation = false;
  EXPECT_FALSE(session.TryApplyRuntimeConfig(patch));

  // 拒绝后配置不变，仍能正常执行 RDA 成像。
  const session::SarCycleResult result = session.StepWithResult(MakeInput());
  EXPECT_EQ(result.status, session::SarCycleStatus::kCompleted);
  EXPECT_TRUE(result.product.output_frame.has_l1_image);
}

TEST(SarSessionPipelineTest, AttachRecorderDrivesUpdateAutomatically) {
  session::SarSession session = session::SarSession::Create(MakeSmallRdaConfig());
  session::SarProductLifecycleRecorder recorder;

  session.AttachProductLifecycleRecorder(&recorder);

  // 第一次步进应产出图像产品——recorder 应被自动驱动，发出 kImageProduced。
  const session::SarCycleResult first = session.StepWithResult(MakeInput(1U));
  ASSERT_EQ(first.status, session::SarCycleStatus::kCompleted);
  ASSERT_TRUE(first.product.output_frame.has_l1_image);

  const std::vector<session::SarProductLifecycleEvent>& first_events = recorder.GetLastEvents();
  ASSERT_EQ(first_events.size(), 1U);
  EXPECT_EQ(first_events.front().kind, session::SarProductLifecycleEventKind::kImageProduced);

  // 第二次步进——产品持续存在，应发出 kProductSustained。
  const session::SarCycleResult second = session.StepWithResult(MakeInput(2U));
  ASSERT_EQ(second.status, session::SarCycleStatus::kCompleted);
  const std::vector<session::SarProductLifecycleEvent>& second_events = recorder.GetLastEvents();
  ASSERT_EQ(second_events.size(), 1U);
  EXPECT_EQ(second_events.front().kind, session::SarProductLifecycleEventKind::kProductSustained);
}

TEST(SarSessionPipelineTest, DetachRecorderStopsAutomaticDriving) {
  session::SarSession session = session::SarSession::Create(MakeSmallRdaConfig());
  session::SarProductLifecycleRecorder recorder;

  session.AttachProductLifecycleRecorder(&recorder);
  session.StepWithResult(MakeInput(1U));
  ASSERT_EQ(recorder.GetLastEvents().size(), 1U);
  EXPECT_EQ(recorder.GetLastEvents().front().kind,
            session::SarProductLifecycleEventKind::kImageProduced);

  // 解除注册后再步进——recorder 不应被驱动，last_events 不变。
  session.AttachProductLifecycleRecorder(nullptr);
  session.StepWithResult(MakeInput(2U));
  ASSERT_EQ(recorder.GetLastEvents().size(), 1U);
  EXPECT_EQ(recorder.GetLastEvents().front().kind,
            session::SarProductLifecycleEventKind::kImageProduced);
}

TEST(SarSessionPipelineTest, SessionWithoutRecorderIsBackwardCompatible) {
  session::SarSession session = session::SarSession::Create(MakeSmallRdaConfig());

  // 不 Attach 任何 recorder，行为与改动前完全一致。
  const session::SarCycleResult result = session.StepWithResult(MakeInput());
  EXPECT_EQ(result.status, session::SarCycleStatus::kCompleted);
  EXPECT_TRUE(result.product.output_frame.has_l1_image);
}

TEST(SarSessionPipelineTest, GetLastEventsEmptyAfterConstruction) {
  session::SarProductLifecycleRecorder recorder;
  EXPECT_TRUE(recorder.GetLastEvents().empty());
}

TEST(SarSessionPipelineTest, NonExecutedCycleDoesNotUpdateLastEvents) {
  session::SarSession session = session::SarSession::Create(MakeSmallRdaConfig());
  session::SarProductLifecycleRecorder recorder;
  session.AttachProductLifecycleRecorder(&recorder);

  // 第一次步进产出图像。
  session.StepWithResult(MakeInput(1U));
  ASSERT_EQ(recorder.GetLastEvents().size(), 1U);

  // 构造一个会触发 validation rejection 的输入（cycle_index=0 或其他非法值）。
  // 非执行周期不推进 recorder 状态，last_events 保持不变。
  session::SarCycleInput invalid;
  invalid.cycle_index = 2U;
  invalid.dt_sec = -1.0f;  // 非法 dt
  session.StepWithResult(invalid);
  // last_events 仍为上一次执行周期的事件。
  ASSERT_EQ(recorder.GetLastEvents().size(), 1U);
  EXPECT_EQ(recorder.GetLastEvents().front().kind,
            session::SarProductLifecycleEventKind::kImageProduced);
}

}  // namespace
}  // namespace sar
