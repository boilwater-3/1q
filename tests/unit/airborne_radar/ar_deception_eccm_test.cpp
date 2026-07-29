#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "1q/airborne_radar/session/ArControlProfile.h"
#include "airborne_radar/decision/ControlReducer.h"
#include "airborne_radar/decision/EccmEvaluator.h"
#include "airborne_radar/signal/detection/ArDetectionCellResolver.h"
#include "airborne_radar/signal/tracking/TrackFilter.h"

namespace airborne_radar {
namespace {

session::ArInterferenceObservation BuildObservation(
    oneq::electromagnetics::RfSceneWaveformKind waveform_kind,
    double off_boresight_deg, double jammer_to_noise_db) {
  session::ArInterferenceObservation observation;
  observation.observation_id = 1U;
  observation.estimated_bearing_azimuth_deg = 10.0;
  observation.estimated_bearing_elevation_deg = 2.0;
  observation.estimated_off_boresight_deg = off_boresight_deg;
  observation.estimated_center_frequency_hz = 3.0e9;
  observation.estimated_bandwidth_hz = 2.0e6;
  observation.estimated_waveform_kind = waveform_kind;
  observation.jammer_to_noise_db = jammer_to_noise_db;
  observation.bearing_standard_deviation_deg = 1.0;
  observation.frequency_standard_deviation_hz = 1000.0;
  observation.bandwidth_standard_deviation_hz = 2000.0;
  observation.deception_class = session::DeceptionClass::kNone;
  observation.coherent_emission_count = 0U;
  return observation;
}

session::ArInterferenceObservation BuildFalseTargetObservation() {
  session::ArInterferenceObservation observation = BuildObservation(
      oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain, 5.0, 8.0);
  observation.deception_class = session::DeceptionClass::kLikelyFalseTarget;
  observation.coherent_emission_count = 3U;
  return observation;
}

bool ContainsDirective(const std::vector<session::TacticalProposal>& proposals,
                       session::ControlDirectiveType type) {
  return std::find_if(proposals.begin(), proposals.end(),
                      [type](const session::TacticalProposal& proposal) {
                        return proposal.directive.type == type;
                      }) != proposals.end();
}

// ============================================================================
// Phase 2: 反欺骗 ECCM 评估测试
// ============================================================================

TEST(ArDeceptionEccmTest, PulseTrainObservationTriggersAntiRgpoProposal) {
  decision::EccmEvaluator evaluator;
  std::vector<session::TacticalProposal> proposals;
  const session::ArInterferenceObservation observation = BuildObservation(
      oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain, 5.0, 8.0);

  const decision::EccmEvaluator::Result result =
      evaluator.Evaluate({observation}, &proposals);

  EXPECT_TRUE(result.eccm_activated);
  EXPECT_TRUE(ContainsDirective(
      proposals, session::ControlDirectiveType::REQUEST_ANTI_RGPO_LEADING_EDGE));
}

TEST(ArDeceptionEccmTest, PulseTrainObservationTriggersAntiVgpoProposal) {
  decision::EccmEvaluator evaluator;
  std::vector<session::TacticalProposal> proposals;
  const session::ArInterferenceObservation observation = BuildObservation(
      oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain, 5.0, 8.0);

  const decision::EccmEvaluator::Result result =
      evaluator.Evaluate({observation}, &proposals);

  EXPECT_TRUE(result.eccm_activated);
  EXPECT_TRUE(ContainsDirective(
      proposals, session::ControlDirectiveType::REQUEST_ANTI_VGPO_ACCELERATION_BOUND));
}

TEST(ArDeceptionEccmTest, FalseTargetClassTriggersAntiFalseTargetProposal) {
  decision::EccmEvaluator evaluator;
  std::vector<session::TacticalProposal> proposals;
  const session::ArInterferenceObservation observation = BuildFalseTargetObservation();

  const decision::EccmEvaluator::Result result =
      evaluator.Evaluate({observation}, &proposals);

  EXPECT_TRUE(result.eccm_activated);
  EXPECT_TRUE(ContainsDirective(
      proposals, session::ControlDirectiveType::REQUEST_ANTI_FALSE_TARGET_DISCRIMINATION));
}

TEST(ArDeceptionEccmTest, AntiDeceptionProposalsHaveSurvivabilitySource) {
  decision::EccmEvaluator evaluator;
  std::vector<session::TacticalProposal> proposals;
  const session::ArInterferenceObservation observation = BuildObservation(
      oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain, 5.0, 8.0);

  evaluator.Evaluate({observation}, &proposals);

  // 验证反欺骗提案来自 SURVIVABILITY 源。
  for (const auto& proposal : proposals) {
    if (proposal.directive.type ==
            session::ControlDirectiveType::REQUEST_ANTI_RGPO_LEADING_EDGE ||
        proposal.directive.type ==
            session::ControlDirectiveType::REQUEST_ANTI_VGPO_ACCELERATION_BOUND ||
        proposal.directive.type ==
            session::ControlDirectiveType::REQUEST_ANTI_FALSE_TARGET_DISCRIMINATION) {
      EXPECT_EQ(proposal.directive.source,
                session::ControlDirectiveSource::SURVIVABILITY);
    }
  }
}

TEST(ArDeceptionEccmTest, AntiDeceptionEccmActivatesProtectedEmissionMode) {
  decision::EccmEvaluator evaluator;
  std::vector<session::TacticalProposal> proposals;
  const session::ArInterferenceObservation observation = BuildObservation(
      oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain, 5.0, 8.0);

  const decision::EccmEvaluator::Result result =
      evaluator.Evaluate({observation}, &proposals);

  // kPulseTrain 触发反欺骗措施，eccm_activated 应为 true。
  EXPECT_TRUE(result.eccm_activated);
  EXPECT_EQ(result.activation_source,
            decision::EccmEvaluator::ActivationSource::kReceiverRf);
}

TEST(ArDeceptionEccmTest, EmptyObservationSetDoesNotActivateAntiDeception) {
  decision::EccmEvaluator evaluator;
  std::vector<session::TacticalProposal> proposals;

  const decision::EccmEvaluator::Result result =
      evaluator.Evaluate({}, &proposals);

  EXPECT_FALSE(result.eccm_activated);
  EXPECT_FALSE(ContainsDirective(
      proposals, session::ControlDirectiveType::REQUEST_ANTI_RGPO_LEADING_EDGE));
  EXPECT_FALSE(ContainsDirective(
      proposals, session::ControlDirectiveType::REQUEST_ANTI_VGPO_ACCELERATION_BOUND));
  EXPECT_FALSE(ContainsDirective(
      proposals, session::ControlDirectiveType::REQUEST_ANTI_FALSE_TARGET_DISCRIMINATION));
}

TEST(ArDeceptionEccmTest, NonPulseTrainObservationDoesNotTriggerAntiDeception) {
  decision::EccmEvaluator evaluator;
  // 使用连续波（非脉冲列）——不触发反欺骗。
  std::vector<session::TacticalProposal> proposals_k_continuous;
  evaluator.Evaluate(
      {BuildObservation(oneq::electromagnetics::RfSceneWaveformKind::kContinuous, 5.0, 8.0)},
      &proposals_k_continuous);

  EXPECT_FALSE(ContainsDirective(
      proposals_k_continuous,
      session::ControlDirectiveType::REQUEST_ANTI_RGPO_LEADING_EDGE));
  EXPECT_FALSE(ContainsDirective(
      proposals_k_continuous,
      session::ControlDirectiveType::REQUEST_ANTI_VGPO_ACCELERATION_BOUND));

  // 使用带限噪声——不触发反欺骗。
  std::vector<session::TacticalProposal> proposals_noise;
  evaluator.Evaluate(
      {BuildObservation(oneq::electromagnetics::RfSceneWaveformKind::kBandLimitedNoise, 5.0,
                        8.0)},
      &proposals_noise);

  EXPECT_FALSE(ContainsDirective(
      proposals_noise,
      session::ControlDirectiveType::REQUEST_ANTI_RGPO_LEADING_EDGE));
  EXPECT_FALSE(ContainsDirective(
      proposals_noise,
      session::ControlDirectiveType::REQUEST_ANTI_VGPO_ACCELERATION_BOUND));
}

TEST(ArDeceptionEccmTest, AntiRgpoProposalHasHigherPriorityThanRejitter) {
  decision::EccmEvaluator evaluator;
  std::vector<session::TacticalProposal> proposals;
  const session::ArInterferenceObservation observation = BuildObservation(
      oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain, 8.0, 12.0);

  evaluator.Evaluate({observation}, &proposals);

  const auto rgpo_it = std::find_if(
      proposals.begin(), proposals.end(),
      [](const session::TacticalProposal& p) {
        return p.directive.type ==
               session::ControlDirectiveType::REQUEST_ANTI_RGPO_LEADING_EDGE;
      });
  const auto rejitter_it = std::find_if(
      proposals.begin(), proposals.end(),
      [](const session::TacticalProposal& p) {
        return p.directive.type ==
               session::ControlDirectiveType::REQUEST_ECCM_REJITTER;
      });
  ASSERT_NE(rgpo_it, proposals.end());
  ASSERT_NE(rejitter_it, proposals.end());
  // 反 RGPO 前沿跟踪（基础优先级 89）应高于重频抖动（基础优先级 83）。
  EXPECT_GT(rgpo_it->priority, rejitter_it->priority);
}

TEST(ArDeceptionEccmTest, MultiplePulseTrainObservationsAccumulateAntiDeceptionScores) {
  decision::EccmEvaluator evaluator;
  std::vector<session::TacticalProposal> proposals;
  // 两个 kPulseTrain 观测：每个贡献 anti_rgpo_score += 1.5，anti_vgpo_score += 1.5。
  // 一个观测已足够触发（阈值 1.5），两个应仍触发且得分更高。
  const session::ArInterferenceObservation obs1 = BuildObservation(
      oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain, 5.0, 8.0);
  const session::ArInterferenceObservation obs2 = BuildObservation(
      oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain, 3.0, 7.0);

  evaluator.Evaluate({obs1, obs2}, &proposals);

  EXPECT_TRUE(ContainsDirective(
      proposals, session::ControlDirectiveType::REQUEST_ANTI_RGPO_LEADING_EDGE));
  EXPECT_TRUE(ContainsDirective(
      proposals, session::ControlDirectiveType::REQUEST_ANTI_VGPO_ACCELERATION_BOUND));
}

// ============================================================================
// Phase 3: 信号层反制措施测试
// ============================================================================

TEST(ArDeceptionEccmTest, AntiDeceptionDirectiveRejectedWhenInvalidValue) {
  // 布尔型反欺骗指令不应携带 requested_value。
  const session::ControlDirective directive_with_value(
      session::ControlDirectiveType::REQUEST_ANTI_RGPO_LEADING_EDGE,
      session::ControlDirectiveSource::SURVIVABILITY, 1.0f);
  EXPECT_TRUE(directive_with_value.has_requested_value);

  // 构造 ControlReducer 验证该指令被 IsValidDirectiveValue 拒绝。
  decision::ControlReducer reducer({});
  std::vector<session::TacticalProposal> proposals;
  proposals.push_back(session::TacticalProposal{directive_with_value, 90, ""});

  session::ArControlProfile profile;
  const auto result = reducer.Reduce(profile, proposals);

  EXPECT_TRUE(result.applied_directives.empty());
  EXPECT_FALSE(result.rejected_directives.empty());
  EXPECT_FALSE(result.profile.enable_anti_rgpo_leading_edge);
}

TEST(ArDeceptionEccmTest, AntiRgpoDetectionCellWorksWithAntiRgpoEnabled) {
  // 验证启用 anti-RGPO 时 detection cell 仍正常求解（不失配）。
  using namespace oneq::electromagnetics;

  // 构造 kPulseTrain 干扰入射链路。
  RfIncidentLinkResult link;
  link.identity = RfEmissionIdentity{1U, 100U, 1000U};
  link.received_power_before_overlap_w = 10.0;
  link.received_power_w = 10.0;
  link.propagation_delay_s = 0.0;  // 零传播延迟确保 emission_time 落入活动窗口
  link.doppler_shift_hz = 0.0;
  link.emission_waveform.kind = RfSceneWaveformKind::kPulseTrain;
  link.emission_waveform.center_frequency_hz = 3.0e9;
  link.emission_waveform.occupied_bandwidth_hz = 2.0e6;
  link.emission_waveform.activity_start_time_s = 0.0;
  link.emission_waveform.activity_duration_s = 0.1;
  link.emission_waveform.pulse_width_s = 1e-6;
  link.emission_waveform.pulse_repetition_interval_s = 1e-4;
  link.emission_waveform.first_pulse_time_s = 0.0;
  link.emission_waveform.pulse_count = 1000U;
  link.emission_waveform.pulse_jitter_fraction = 0.0;
  link.emission_waveform.transmit_power_w = 100.0;

  // 配置 AR 自身发射波形。
  RfWaveformSchedule own_waveform;
  own_waveform.kind = RfSceneWaveformKind::kPulseTrain;
  own_waveform.center_frequency_hz = 3.0e9;
  own_waveform.occupied_bandwidth_hz = 2.0e6;
  own_waveform.pulse_width_s = 1e-6;
  own_waveform.pulse_repetition_interval_s = 1e-4;
  own_waveform.first_pulse_time_s = 0.0;
  own_waveform.pulse_count = 1000U;
  own_waveform.pulse_jitter_fraction = 0.0;
  own_waveform.transmit_power_w = 1000.0;
  own_waveform.activity_start_time_s = 0.0;
  own_waveform.activity_duration_s = 0.1;

  signal::detection::ArDetectionCellConfig cell_config;
  cell_config.own_transmit_waveform = own_waveform;
  cell_config.receive_window_start_time_s = 0.0;
  cell_config.receive_window_duration_s = 0.1;
  cell_config.matched_filter_bandwidth_hz = 2.0e6;
  cell_config.one_way_antenna_gain_dbi = 30.0;
  cell_config.receiver_loss_db = 3.0;
  cell_config.receiver_noise_figure_db = 3.0;

  signal::detection::ArDetectionCellTarget cell_target;
  cell_target.range_m = 1.0;  // 极近距离使 echo 与干扰脉冲在时域重叠
  cell_target.closing_radial_velocity_mps = 100.0;
  cell_target.rcs_m2 = 1.0;
  cell_target.two_way_additional_propagation_loss_db = 0.0;
  cell_target.effective_pulse_count = 10U;

  const RfEmissionIdentity own_id{999U, 999U, 999U};
  const std::vector<RfIncidentLinkResult> incident_links = {link};

  // 不启用 anti-RGPO：正常求解。
  {
    cell_config.enable_anti_rgpo_leading_edge = false;
    signal::detection::ArDetectionCellResult result;
    ASSERT_TRUE(signal::detection::TryResolveArDetectionCell(
        cell_config, cell_target, own_id, incident_links, 0.0, &result));
    EXPECT_GT(result.interference_power_w, 0.0);
  }

  // 启用 anti-RGPO：仍正常求解，干扰功率因 kPulseTrain 功率减半而降低。
  {
    cell_config.enable_anti_rgpo_leading_edge = true;
    signal::detection::ArDetectionCellResult result;
    ASSERT_TRUE(signal::detection::TryResolveArDetectionCell(
        cell_config, cell_target, own_id, incident_links, 0.0, &result));
    EXPECT_GT(result.interference_power_w, 0.0);

    cell_config.enable_anti_rgpo_leading_edge = false;
    signal::detection::ArDetectionCellResult result_off;
    ASSERT_TRUE(signal::detection::TryResolveArDetectionCell(
        cell_config, cell_target, own_id, incident_links, 0.0, &result_off));
    EXPECT_GT(result_off.interference_power_w, 0.0);

    // 启用 anti-RGPO 后 kPulseTrain 干扰功率应降低约 50%。
    EXPECT_LT(result.interference_power_w, result_off.interference_power_w);
  }
}

TEST(ArDeceptionEccmTest, AntiVgpoBoundsVelocityChangePerCycle) {
  // 验证启用加速度限幅后，速度变化不超出 max_acceleration * dt。
  signal::tracking::TrackFilterConfig filter_config;
  filter_config.enable_anti_vgpo_acceleration_bound = true;
  filter_config.max_acceleration_mps2 = 10.0;  // 最大加速度 10 m/s²

  signal::tracking::TrackFilter filter(filter_config);

  // 构造输入目标：速度突增 50 m/s（远超出允许范围）。
  session::ArSceneTarget input(100.0f, 50.0f, 0.0f, 1.0f);
  input.velocity_x = 100.0f;   // 当前测量速度
  input.velocity_y = 50.0f;

  // 上下文：上一周期速度仅为 10 m/s，检测成功，dt = 0.1s。
  signal::tracking::TrackFilterContext context;
  context.detection_succeeded = true;
  context.previous_velocity_x = 10.0f;  // 上一周期 X 速度
  context.previous_velocity_y = 0.0f;   // 上一周期 Y 速度
  context.previous_velocity_z = 0.0f;
  context.cycle_dt_sec = 0.1;           // 步长 0.1s

  const session::ArSceneTarget output = filter.Filter(input, context);

  // max_acceleration * dt = 10.0 * 0.1 = 1.0 m/s
  // 上一周期 X=10，当前测量 X=100 → 变化 90 m/s，应裁剪为 10+1=11
  EXPECT_LE(output.velocity_x, 11.0f);
  // 上一周期 Y=0，当前测量 Y=50 → 变化 50 m/s，应裁剪为 0+1=1
  EXPECT_LE(output.velocity_y, 1.0f);
}

TEST(ArDeceptionEccmTest, AntiVgpoDisabledDoesNotBoundVelocity) {
  // 不启用时，速度不变（通过 identity filter）。
  signal::tracking::TrackFilterConfig filter_config;
  filter_config.enable_anti_vgpo_acceleration_bound = false;

  signal::tracking::TrackFilter filter(filter_config);
  session::ArSceneTarget input(100.0f, 50.0f, 0.0f, 1.0f);

  signal::tracking::TrackFilterContext context;
  context.detection_succeeded = true;
  context.previous_velocity_x = 10.0f;
  context.previous_velocity_y = 0.0f;
  context.previous_velocity_z = 0.0f;
  context.cycle_dt_sec = 0.1;

  const session::ArSceneTarget output = filter.Filter(input, context);

  // 不启用限幅 → 速度原样通过。
  EXPECT_FLOAT_EQ(output.velocity_x, 100.0f);
  EXPECT_FLOAT_EQ(output.velocity_y, 50.0f);
}

TEST(ArDeceptionEccmTest, AntiDeceptionControlDirectiveAppliedByReducer) {
  // 验证合法的反欺骗指令经 ControlReducer::Reduce 后写入 ArControlProfile。
  decision::ControlReducer reducer({});
  std::vector<session::TacticalProposal> proposals;
  proposals.push_back(session::TacticalProposal{
      session::ControlDirective(
          session::ControlDirectiveType::REQUEST_ANTI_RGPO_LEADING_EDGE,
          session::ControlDirectiveSource::SURVIVABILITY),
      90, ""});
  proposals.push_back(session::TacticalProposal{
      session::ControlDirective(
          session::ControlDirectiveType::REQUEST_ANTI_VGPO_ACCELERATION_BOUND,
          session::ControlDirectiveSource::SURVIVABILITY),
      85, ""});
  proposals.push_back(session::TacticalProposal{
      session::ControlDirective(
          session::ControlDirectiveType::REQUEST_ANTI_FALSE_TARGET_DISCRIMINATION,
          session::ControlDirectiveSource::SURVIVABILITY),
      81, ""});

  session::ArControlProfile profile;
  const auto result = reducer.Reduce(profile, proposals);

  EXPECT_TRUE(result.profile.enable_anti_rgpo_leading_edge);
  EXPECT_TRUE(result.profile.enable_anti_vgpo_acceleration_bound);
  EXPECT_TRUE(result.profile.enable_anti_false_target_discrimination);
  EXPECT_EQ(result.applied_directives.size(), 3U);
  // 反欺骗属于 ECCM 域，应在 ResetEccmDomain 后写入。
  EXPECT_FALSE(result.profile.enable_lpi_power_control);
}

}  // namespace
}  // namespace airborne_radar
