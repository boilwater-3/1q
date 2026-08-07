#include "airborne_radar/runtime/ArController.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <unordered_map>

#include "1q/airborne_radar/session/ArControlProfile.h"
#include "airborne_radar/session/ArDiagnosticUtils.h"
#include "1q/airborne_radar/session/ArTrackOutput.h"
#include "airborne_radar/decision/ControlReducer.h"
#include "airborne_radar/decision/TacticalCoordinator.h"
#include "airborne_radar/environment/IEnvironmentService.h"
#include "airborne_radar/runtime/ControlCommandMapper.h"
#include "airborne_radar/session/MutableArContext.h"
#include "airborne_radar/signal/detection/RadarEquations.h"
#include "airborne_radar/signal/pipeline/ISignalPipeline.h"
#include "common/logging/ProjectLog.h"
#include "common/runtime/RuntimeCycleExecutor.h"

namespace airborne_radar {
namespace extension {

namespace {

bool HasOperationalProfileChanged(const session::ArControlProfile& previous,
                                  const session::ArControlProfile& next) {
  return previous.enable_lpi_power_control != next.enable_lpi_power_control ||
         previous.lpi_power_scale != next.lpi_power_scale ||
         previous.enable_lpi_beamforming != next.enable_lpi_beamforming ||
         previous.lpi_dwell_scale != next.lpi_dwell_scale ||
         previous.enable_agility_frequency != next.enable_agility_frequency ||
         previous.agility_frequency_hop_phase != next.agility_frequency_hop_phase ||
         previous.enable_sidelobe_canceller != next.enable_sidelobe_canceller ||
         previous.enable_adaptive_beamforming != next.enable_adaptive_beamforming ||
         previous.enable_eccm_rejitter != next.enable_eccm_rejitter ||
         previous.eccm_burnthrough_gain != next.eccm_burnthrough_gain ||
         previous.enable_anti_rgpo_leading_edge != next.enable_anti_rgpo_leading_edge ||
         previous.enable_anti_vgpo_acceleration_bound != next.enable_anti_vgpo_acceleration_bound ||
         previous.enable_anti_false_target_discrimination != next.enable_anti_false_target_discrimination;
}

extension::ControlReducerConfig MapDecisionControlConfig(
    const config::DecisionControlConfig& config) {
  extension::ControlReducerConfig mapped;
  mapped.lpi_hold_cycles_after_request = config.lpi_hold_cycles_after_request;
  mapped.eccm_hold_cycles_after_request = config.eccm_hold_cycles_after_request;
  mapped.lpi_cooldown_cycles_after_release = config.lpi_cooldown_cycles_after_release;
  mapped.eccm_cooldown_cycles_after_release = config.eccm_cooldown_cycles_after_release;
  return mapped;
}

}  // namespace

/**
 * @brief 控制器内部自有组件，生命周期由 Impl 管理。
 *
 * 统一封装默认决策子系统。
 */
struct OwnedDecisionComponents {
  std::unique_ptr<decision::TacticalCoordinator> decision_engine;
  std::unique_ptr<session::TacticalStateStore> tactical_state_store;
  std::unique_ptr<decision::ControlReducer> control_reducer;
};

/**
 * @brief ArController 内部实现体，持有所有运行时状态。
 */
struct ArController::Impl {
  // -- 外部引用（生命周期由 Session 管理）
  session::MutableArContext& radar_context;
  signal::ISignalPipeline& signal_pipeline;
  environment::IEnvironmentService& environment_service;

  // -- 决策子系统
  OwnedDecisionComponents owned_decision_components;

  // -- 控制状态
  session::ArControlProfile control_profile{};

  // -- 独立生命周期组件
  std::unique_ptr<extension::ControlCommandMapper> command_mapper;

  // -- 周期运行时状态
  oneq::common::runtime::RuntimeCycleState<session::TrackOutputFrame, session::ArIssueList>
      cycle_state{};
  bool last_cycle_executed{false};
  session::SignalCycleAbortReason last_signal_abort_reason{session::SignalCycleAbortReason::kNone};
  session::ArIssueList latest_issues{}; /**< 正常周期按目标排除的 kInfo 诊断（规则 13b）。 */

  bool has_pending_internal_decision{false};
  std::uint32_t pending_internal_cycle_index{0U};
  std::uint64_t pending_internal_batch_id{0U};
  std::vector<session::TacticalProposal> pending_internal_proposals{};
  bool has_pending_external_override{false};
  session::ExternalDecisionOverride pending_external_override{};
  bool has_latest_decision_observation{false};
  session::DecisionObservation latest_decision_observation{};
  session::DecisionControlSource last_applied_decision_source{
      session::DecisionControlSource::kNone};
  std::uint32_t last_applied_decision_cycle_index{0U};
  std::uint64_t last_applied_decision_batch_id{0U};
  std::vector<session::TacticalProposal> last_applied_decision_proposals{};
  bool control_prepared_for_cycle{false};

  // -- 识别链路（kLrr）
  ArRecognitionStaticContext recognition_static_context{};
  config::ArRecognitionConfig recognition_config{};
  config::ArWorkMode work_mode{config::ArWorkMode::kTws};
  recognition::RecognitionTracker recognition_tracker{};
  std::unique_ptr<recognition::RecognitionFeatureDatabase> recognition_database{};
  std::string recognition_database_path{};
  bool recognition_mode_active{false}; /**< 上一已执行周期是否处于 kLrr。 */
  float sim_time_sec{0.0f};            /**< 已执行周期累计仿真时刻（s）。 */
  session::ArRecognitionCycleSummary latest_recognition_summary{};
  bool has_latest_recognition_summary{false};

  /** @brief 构造使用默认 TacticalCoordinator 的控制器。 */
  Impl(session::MutableArContext& ctx, signal::ISignalPipeline& sig,
       environment::IEnvironmentService& env,
       const config::DecisionControlConfig& decision_control_config,
       const ArRecognitionStaticContext& recognition_static_context_in)
      : radar_context(ctx),
        signal_pipeline(sig),
        environment_service(env),
        recognition_static_context(recognition_static_context_in) {
    owned_decision_components.decision_engine.reset(new decision::TacticalCoordinator(nullptr));
    owned_decision_components.tactical_state_store.reset(new session::TacticalStateStore());
    owned_decision_components.control_reducer.reset(
        new decision::ControlReducer(MapDecisionControlConfig(decision_control_config)));
    command_mapper.reset(
        new extension::ControlCommandMapper(*owned_decision_components.control_reducer, ctx));
  }

  /** @brief 重置每周期可变标志位。 */
  void ResetPerCycleFlags(bool preserve_prepared_control_attribution) {
    last_cycle_executed = false;
    last_signal_abort_reason = session::SignalCycleAbortReason::kNone;
    if (!preserve_prepared_control_attribution) {
      last_applied_decision_source = session::DecisionControlSource::kNone;
      last_applied_decision_cycle_index = 0U;
      last_applied_decision_batch_id = 0U;
      last_applied_decision_proposals.clear();
    }
  }

  /** @brief 校验外部覆盖返回的 profile 字段是否在合法范围内。 */
  static bool IsValidOverrideProfile(const session::ArControlProfile& profile) {
    if (profile.enable_lpi_power_control) {
      if (!std::isfinite(profile.lpi_power_scale) || profile.lpi_power_scale <= 0.0f ||
          profile.lpi_power_scale > 1.0f) {
        return false;
      }
    }
    if (profile.lpi_dwell_scale != 1.0f) {
      if (!std::isfinite(profile.lpi_dwell_scale) || profile.lpi_dwell_scale < 0.25f ||
          profile.lpi_dwell_scale > 1.0f) {
        return false;
      }
    }
    if (profile.eccm_burnthrough_gain != 1.0f) {
      if (!std::isfinite(profile.eccm_burnthrough_gain) || profile.eccm_burnthrough_gain <= 1.0f ||
          profile.eccm_burnthrough_gain > 2.0f) {
        return false;
      }
    }
    if (profile.agility_frequency_hop_phase > 1) {
      return false;
    }
    return true;
  }

  /** @brief 将外部覆盖应用到原生 profile 上，返回最终 profile。 */
  static session::ArControlProfile ApplyExternalOverride(
    const session::ArControlProfile& native_profile,
    const session::ExternalDecisionOverride& override) {
  session::ArControlProfile override_profile = override.profile;
  if (!IsValidOverrideProfile(override_profile)) {
    return native_profile;
  }
    override_profile.version = HasOperationalProfileChanged(native_profile, override_profile)
                                   ? native_profile.version + 1U
                                   : native_profile.version;
    return override_profile;
  }

  void ApplyPendingDecisionControl() {
    if (!has_pending_internal_decision) {
      return;
    }

    // 1. 原生归约（始终执行）
    const extension::ControlReductionResult native_result =
        command_mapper->Apply(&control_profile, pending_internal_proposals);

    // 2. 外部覆盖
    if (has_pending_external_override) {
      const session::ArControlProfile override_profile =
          ApplyExternalOverride(control_profile, pending_external_override);
      if (override_profile.version != control_profile.version) {
        control_profile = override_profile;
        radar_context.UpdateRadarControlProfile(control_profile);
        // 对差异字段生成 ArCommand
        const std::vector<session::ControlDirective> diffs =
            command_mapper->DiffProfiles(native_result.profile, control_profile);
        for (std::size_t i = 0; i < diffs.size(); ++i) {
          const session::ArCommand cmd =
              extension::ControlCommandMapper::DirectiveToCommand(diffs[i]);
          if (cmd.type != session::ArCommandType::NONE) {
            radar_context.SubmitControlCommand(cmd);
          }
        }
      }
      last_applied_decision_source = session::DecisionControlSource::kExternal;
    } else {
      last_applied_decision_source = session::DecisionControlSource::kInternal;
    }

    last_applied_decision_cycle_index = pending_internal_cycle_index;
    last_applied_decision_batch_id = pending_internal_batch_id;
    last_applied_decision_proposals = pending_internal_proposals;
    has_pending_internal_decision = false;
    pending_internal_proposals.clear();
    has_pending_external_override = false;
    pending_external_override = session::ExternalDecisionOverride();
  }

  /** @brief 由会话在运行期配置提交边界调用；按路径变化原子加载识别数据库。 */
  void UpdateRecognitionRuntime(config::ArWorkMode work_mode_in,
                                const config::ArRecognitionConfig& recognition_config_in) {
    work_mode = work_mode_in;
    recognition_config = recognition_config_in;

    recognition::RecognitionTracker::Options options;
    options.min_confirmed_hits = recognition_config.min_confirmed_hits;
    options.accumulation_window_sec = recognition_config.accumulation_window_sec;
    options.min_observation_count = recognition_config.min_observation_count;
    options.acceptance_score = recognition_config.acceptance_score;
    options.minimum_margin = recognition_config.minimum_margin;
    options.result_hold_sec = recognition_config.result_hold_sec;
    options.max_range_m = recognition_config.max_range_m;
    recognition_tracker.SetOptions(options);

    // 数据库加载：路径变化或未加载时按需加载；失败保持原库（识别降级 kDisabled）。
    const bool should_load =
        recognition_config.enabled && !recognition_config.database_path.empty() &&
        recognition_config.database_path != recognition_database_path;
    if (!should_load) {
      return;
    }
    std::unique_ptr<recognition::RecognitionFeatureDatabase> candidate(
        new recognition::RecognitionFeatureDatabase());
    std::string error;
    if (recognition::RecognitionFeatureDatabase::Load(recognition_config.database_path,
                                                      candidate.get(), &error)) {
      recognition_database = std::move(candidate);
      recognition_database_path = recognition_config.database_path;
      recognition_tracker.SetActiveDatabaseVersion(recognition_database->version());
    } else {
      // 中译：识别特征数据库加载失败：{}。
      // 标识：外部数据加载失败——识别数据库不可用时识别功能降级，
      //       排查数据库路径与文件格式。
      PROJECT_LOG_ERROR("[ArController] recognition database load failed: {}", error);
    }
  }

  /**
   * @brief kLrr 波束指向（Path A）：从上一已执行周期输出选择优先确认航迹，
   * 经 ISignalPipeline 周期扫描中心覆盖设指向；非 kLrr 时清除覆盖。
   */
  void PrepareLrrPointing() {
    if (work_mode != config::ArWorkMode::kLrr) {
      signal_pipeline.ClearCycleScanCenterOverride();
      return;
    }
    const session::TrackStateSnapshot* selected = nullptr;
    const session::TrackOutputFrame& prior = cycle_state.latest_output;
    for (std::size_t i = 0U; i < prior.tracks.size(); ++i) {
      const session::TrackStateSnapshot& track = prior.tracks[i];
      if (track.status != session::TrackStatus::kConfirmed) {
        continue;
      }
      if (selected == nullptr || IsBetterLrrCandidate(track, *selected)) {
        selected = &track;
      }
    }
    if (selected == nullptr) {
      return;
    }
    const float range_hypot = std::sqrt(selected->position_x * selected->position_x +
                                        selected->position_y * selected->position_y);
    config::AzimuthElevationDeg pointing;
    pointing.az_deg =
        std::atan2(selected->position_y, selected->position_x) * 180.0f / static_cast<float>(M_PI);
    pointing.el_deg = std::atan2(selected->position_z, range_hypot) * 180.0f /
                      static_cast<float>(M_PI);
    signal_pipeline.SetCycleScanCenterOverride(pointing);
  }

  /** @brief kLrr 优先候选排序：威胁等级优先，其次斜距更近。 */
  static bool IsBetterLrrCandidate(const session::TrackStateSnapshot& candidate,
                                   const session::TrackStateSnapshot& current) {
    const auto threat_rank = [](const std::string& type) {
      if (type == "HIGH_THREAT_FIGHTER") {
        return 2;
      }
      if (type == "LOW_THREAT_TARGET") {
        return 1;
      }
      return 0;
    };
    const int candidate_rank = threat_rank(candidate.target_type);
    const int current_rank = threat_rank(current.target_type);
    if (candidate_rank != current_rank) {
      return candidate_rank > current_rank;
    }
    const float candidate_range =
        std::sqrt(candidate.position_x * candidate.position_x +
                  candidate.position_y * candidate.position_y +
                  candidate.position_z * candidate.position_z);
    const float current_range =
        std::sqrt(current.position_x * current.position_x +
                  current.position_y * current.position_y +
                  current.position_z * current.position_z);
    return candidate_range < current_range;
  }

  /** @brief 效能级 SNR：单站雷达方程回波功率与热噪声底之差（dB）。 */
  float ComputeSnrDb(float rcs_m2, float range_m) const {
    const float echo_dBW = signal::detection::RadarEquations::ComputeEchoPower_dBW(
        recognition_static_context.transmitter, recognition_static_context.antenna, rcs_m2,
        range_m, 0.0f);
    const float noise_w = signal::detection::RadarEquations::ComputeThermalNoisePower_W(
        recognition_static_context.transmitter, recognition_static_context.receiver);
    return echo_dBW - 10.0f * std::log10(std::max(1.0e-12f, noise_w));
  }

  /** @brief 单目标识别观测上下文（视线角、SNR、带宽、驻留）。 */
  recognition::RecognitionObservationContext MakeRecognitionContext(
      const session::ArSceneTarget& target, float platform_altitude_m) const {
    recognition::RecognitionObservationContext context;
    context.snr_db = ComputeSnrDb(target.rcs, target.range_m);
    context.range_m = target.range_m;
    context.bandwidth_hz = recognition_static_context.transmitter.bandwidth_hz;
    context.dwell_sec = recognition_config.recognition_dwell_sec;
    const float range_hypot =
        std::sqrt(target.position_x * target.position_x + target.position_y * target.position_y);
    context.look_az_deg =
        std::atan2(target.position_y, target.position_x) * 180.0f / static_cast<float>(M_PI);
    context.look_el_deg =
        std::atan2(target.position_z, range_hypot) * 180.0f / static_cast<float>(M_PI);
    context.platform_altitude_m = platform_altitude_m;
    // 观测有效性不由硬编码视角覆盖下限门控：覆盖下限属数据库 profile 级
    // 适用条件，由匹配阶段按需判定。
    context.minimum_aspect_coverage_deg = 0.0f;
    return context;
  }

  /** @brief 已执行周期内的识别链路：积累/判定 + 回填 + 摘要。 */
  void RunRecognitionCycle(const session::DecisionInputFrame& decision_frame,
                           session::TrackOutputFrame* output_frame,
                           const session::ArSceneTargetList& scene_targets,
                           const oneq::common::runtime::RuntimeCycleStamp& stamp) {
    const bool in_lrr = work_mode == config::ArWorkMode::kLrr;
    // 模式切换：退出 kLrr → 清空积累（结论进入保持期）。
    if (recognition_mode_active && !in_lrr) {
      recognition_tracker.ExitRecognitionMode();
    }
    recognition_mode_active = in_lrr;
    sim_time_sec += radar_context.GetCycleDeltaTimeSec();

    if (in_lrr && recognition_database != nullptr) {
      // 观测输入：external_target_id → 场景目标 → association_key 映射。
      std::unordered_map<std::uint64_t, recognition::RecognitionTracker::TrackObservationInput>
          observations_by_target_id;
      const float platform_altitude_m = radar_context.GetPlatformAltitudeM();
      for (std::size_t i = 0U; i < scene_targets.size(); ++i) {
        if (scene_targets[i].external_target_id == 0U) {
          continue;
        }
        recognition::RecognitionTracker::TrackObservationInput input;
        input.target = &scene_targets[i];
        input.context = MakeRecognitionContext(scene_targets[i], platform_altitude_m);
        observations_by_target_id[scene_targets[i].external_target_id] = input;
      }
      std::unordered_map<std::uint64_t, recognition::RecognitionTracker::TrackObservationInput>
          observations_by_key;
      for (std::size_t i = 0U; i < decision_frame.tracks.size(); ++i) {
        const session::TrackStateSnapshot& track = decision_frame.tracks[i];
        const std::unordered_map<std::uint64_t,
                                 recognition::RecognitionTracker::TrackObservationInput>::const_iterator
            found = observations_by_target_id.find(track.external_target_id);
        if (found != observations_by_target_id.end()) {
          observations_by_key[track.association_key] = found->second;
        }
      }
      recognition_tracker.UpdateCycle(decision_frame.tracks, observations_by_key,
                                      *recognition_database, recognition_config.feature_weights,
                                      sim_time_sec, stamp.cycle_index, stamp.batch_id);
    } else {
      recognition_tracker.HoldCycle(decision_frame.tracks, sim_time_sec);
    }

    // 回填（照威胁分类先例）：仅 output_frame；decision_frame 不含识别字段。
    for (std::size_t i = 0U; i < output_frame->tracks.size(); ++i) {
      const session::ArRecognitionResult* result =
          recognition_tracker.FindResult(output_frame->tracks[i].association_key);
      if (result != nullptr) {
        output_frame->tracks[i].recognition = *result;
      }
    }
    latest_recognition_summary = recognition_tracker.BuildSummary(output_frame->tracks);
    has_latest_recognition_summary = true;
  }
};

// -- 构造函数

ArController::ArController(session::MutableArContext& radar_context,
                           signal::ISignalPipeline& signal_pipeline,
                           environment::IEnvironmentService& environment_service,
                           config::DecisionControlConfig decision_control_config,
                           ArRecognitionStaticContext recognition_static_context)
    : impl_(new Impl(radar_context, signal_pipeline, environment_service, decision_control_config,
                     recognition_static_context)) {}

ArController::~ArController() = default;

void ArController::UpdateDecisionControlConfig(
    const config::DecisionControlConfig& decision_control_config) {
  impl_->owned_decision_components.control_reducer->UpdateConfig(
      MapDecisionControlConfig(decision_control_config));
}

void ArController::UpdateRecognitionRuntime(
    config::ArWorkMode work_mode, const config::ArRecognitionConfig& recognition_config) {
  impl_->UpdateRecognitionRuntime(work_mode, recognition_config);
}

bool ArController::HasLatestRecognitionSummary() const {
  return impl_->has_latest_recognition_summary;
}

const session::ArRecognitionCycleSummary& ArController::GetLatestRecognitionSummary() const {
  return impl_->latest_recognition_summary;
}

std::string ArController::GetActiveRecognitionDatabaseVersion() const {
  return impl_->recognition_tracker.ActiveDatabaseVersion();
}

void ArController::RunOnce(const signal::pipeline::SignalCycleInput& cycle_input) {
  const bool control_was_prepared = impl_->control_prepared_for_cycle;
  impl_->ResetPerCycleFlags(control_was_prepared);

  const session::ArSceneTargetList& scene_targets = impl_->radar_context.GetSceneTargets();
  const config::PlatformAttitudeDeg platform_attitude = impl_->radar_context.GetPlatformAttitude();
  const float platform_altitude_m = impl_->radar_context.GetPlatformAltitudeM();
  const float cycle_dt_sec = impl_->radar_context.GetCycleDeltaTimeSec();
  const std::uint32_t cycle_index = impl_->radar_context.GetCycleIndex();

  const oneq::common::runtime::RuntimeCycleStamp stamp =
      oneq::common::runtime::MakeRuntimeCycleStamp(cycle_index, impl_->cycle_state.next_batch_id);

  // 校验
  session::ArIssueList issues = session::ValidateArCycleDeltaTime(cycle_dt_sec);
  {
    const session::ArIssueList target_issues =
        session::ValidateArSceneTargets(scene_targets);
    issues.insert(issues.end(), target_issues.begin(), target_issues.end());
  }
  impl_->cycle_state.last_validation_issues = issues;

  if (session::HasValidationError(issues)) {
    impl_->last_signal_abort_reason = session::SignalCycleAbortReason::kValidationRejected;
    return;
  }

  // 冻结环境
  session::EnvironmentCycleContext environment_cycle_context;
  environment_cycle_context.cycle_index = stamp.cycle_index;
  environment_cycle_context.dt_sec = cycle_dt_sec;
  impl_->environment_service.BeginCycle(environment_cycle_context);

  // 执行信号流水线与决策引擎
  if (!control_was_prepared) {
    impl_->ApplyPendingDecisionControl();
  }
  impl_->radar_context.UpdateRadarControlProfile(impl_->control_profile);
  impl_->signal_pipeline.SetControlProfile(impl_->control_profile);
  impl_->signal_pipeline.UpdatePlatformAttitude(platform_attitude);
  impl_->signal_pipeline.UpdatePlatformAltitudeM(platform_altitude_m);
  // 周期输入已通过 SignalCycleInput 显式传入，不再依赖 mutable 旁路状态。

  // kLrr 波束指向（Path A）：从 prior-cycle 确认航迹选优先目标并经周期覆盖注入。
  impl_->PrepareLrrPointing();

  session::SignalCycleResult signal_result =
      impl_->signal_pipeline.RunCycle(cycle_input, impl_->environment_service);
  impl_->control_prepared_for_cycle = false;

  impl_->last_cycle_executed = signal_result.executed_this_cycle;
  impl_->last_signal_abort_reason = signal_result.abort_reason;

  if (!impl_->last_cycle_executed) {
    return;
  }
  // 规则 13b：正常周期按目标排除的 kInfo 诊断转写（abort 路径不变）。
  impl_->latest_issues = std::move(signal_result.issues);

  session::DecisionInputFrame decision_frame = signal_result.decision_frame;
  decision_frame.cycle_index = stamp.cycle_index;
  decision_frame.batch_id = stamp.batch_id;
  decision_frame.interference_observations = cycle_input.interference_observations;

  session::TrackOutputFrame track_output_frame;
  track_output_frame.cycle_index = stamp.cycle_index;
  track_output_frame.batch_id = stamp.batch_id;
  track_output_frame.tracks = decision_frame.tracks;

  session::TacticalDecisionResult decision_result;
  if (impl_->owned_decision_components.decision_engine != nullptr &&
      impl_->owned_decision_components.tactical_state_store != nullptr) {
    decision_result = impl_->owned_decision_components.decision_engine->Evaluate(
        decision_frame, *impl_->owned_decision_components.tactical_state_store);

    // 将目标分类结果回填到轨迹输出帧
    const auto& classifications = decision_result.target_classification_result;
    auto& output_tracks = track_output_frame.tracks;
    const std::size_t count = std::min(classifications.size(), output_tracks.size());
    for (std::size_t i = 0; i < count; ++i) {
      output_tracks[i].target_type = classifications[i].target_type;
      output_tracks[i].target_probability = classifications[i].probability;
    }
  }

  // 识别链路（仅识别启用时执行；回填照威胁分类先例，只进 output_frame）
  if (impl_->recognition_config.enabled) {
    impl_->RunRecognitionCycle(decision_frame, &track_output_frame,
                               impl_->radar_context.GetSceneTargets(), stamp);
  } else {
    // 识别未启用：输出帧识别字段复位为默认（kDisabled），不残留旧结论。
    for (std::size_t i = 0U; i < track_output_frame.tracks.size(); ++i) {
      track_output_frame.tracks[i].recognition = session::ArRecognitionResult{};
    }
  }

  signal_result.decision_frame = decision_frame;

  impl_->has_pending_internal_decision = true;
  impl_->pending_internal_cycle_index = stamp.cycle_index;
  impl_->pending_internal_batch_id = stamp.batch_id;
  impl_->pending_internal_proposals = decision_result.proposals;
  impl_->latest_decision_observation.input_frame = decision_frame;
  impl_->latest_decision_observation.active_control_profile = impl_->control_profile;
  impl_->has_latest_decision_observation = true;

  impl_->cycle_state.latest_output = track_output_frame;
  impl_->cycle_state.has_latest_output = true;
  ++impl_->cycle_state.next_batch_id;
}

bool ArController::PrepareEmissionControl() {
  if (impl_->control_prepared_for_cycle) {
    return false;
  }
  impl_->last_applied_decision_source = session::DecisionControlSource::kNone;
  impl_->last_applied_decision_cycle_index = 0U;
  impl_->last_applied_decision_batch_id = 0U;
  impl_->last_applied_decision_proposals.clear();
  impl_->ApplyPendingDecisionControl();
  impl_->radar_context.UpdateRadarControlProfile(impl_->control_profile);
  impl_->signal_pipeline.SetControlProfile(impl_->control_profile);
  impl_->control_prepared_for_cycle = true;
  return true;
}

void ArController::ReleasePreparedEmissionControl() {
  impl_->control_prepared_for_cycle = false;
}

const session::ArControlProfile& ArController::GetControlProfile() const {
  return impl_->control_profile;
}

void ArController::RunCycles(std::size_t cycles) {
  for (std::size_t i = 0; i < cycles; ++i) {
    RunOnce(signal::pipeline::SignalCycleInput{impl_->radar_context.GetSceneTargets()});
  }
}

bool ArController::HasLatestTrackOutputFrame() const {
  return impl_->cycle_state.has_latest_output;
}

const session::TrackOutputFrame& ArController::GetLatestTrackOutputFrame() const {
  return impl_->cycle_state.latest_output;
}

const session::ArIssueList& ArController::GetLatestIssues() const {
  return impl_->latest_issues;
}

const session::ArIssueList& ArController::GetLastValidationIssues() const {
  return impl_->cycle_state.last_validation_issues;
}

bool ArController::HasValidationError() const {
  return session::HasValidationError(impl_->cycle_state.last_validation_issues);
}

bool ArController::ExecutedLatestCycle() const { return impl_->last_cycle_executed; }

session::SignalCycleAbortReason ArController::GetLastSignalCycleAbortReason() const {
  return impl_->last_signal_abort_reason;
}

const session::DecisionObservation& ArController::GetLatestDecisionObservation() const {
  return impl_->latest_decision_observation;
}

bool ArController::HasLatestDecisionObservation() const {
  return impl_->has_latest_decision_observation;
}

session::ExternalDecisionSubmitStatus ArController::SubmitExternalDecision(
    session::ExternalDecisionOverride override_decision) {
  if (!impl_->has_pending_internal_decision || !impl_->has_latest_decision_observation) {
    return session::ExternalDecisionSubmitStatus::kNoPendingObservation;
  }
  if (impl_->has_pending_external_override) {
    return session::ExternalDecisionSubmitStatus::kAlreadySubmitted;
  }
  if (!impl_->IsValidOverrideProfile(override_decision.profile)) {
    return session::ExternalDecisionSubmitStatus::kInvalidProfile;
  }
  impl_->pending_external_override = std::move(override_decision);
  impl_->has_pending_external_override = true;
  return session::ExternalDecisionSubmitStatus::kAccepted;
}

session::DecisionControlSource ArController::GetLastAppliedDecisionSource() const {
  return impl_->last_applied_decision_source;
}

std::uint32_t ArController::GetLastAppliedDecisionCycleIndex() const {
  return impl_->last_applied_decision_cycle_index;
}

std::uint64_t ArController::GetLastAppliedDecisionBatchId() const {
  return impl_->last_applied_decision_batch_id;
}

const std::vector<session::TacticalProposal>& ArController::GetLastAppliedDecisionProposals()
    const {
  return impl_->last_applied_decision_proposals;
}

extension::ArControllerRuntimeState ArController::CaptureRuntimeState() const {
  extension::ArControllerRuntimeState state;
  state.owner_identity = this;
  state.schema_version = 7U;
  state.latest_output = impl_->cycle_state.latest_output;
  state.has_latest_output = impl_->cycle_state.has_latest_output;
  state.last_validation_issues = impl_->cycle_state.last_validation_issues;
  state.next_batch_id = impl_->cycle_state.next_batch_id;
  state.last_cycle_executed = impl_->last_cycle_executed;
  state.last_signal_abort_reason = impl_->last_signal_abort_reason;
  state.control_profile = impl_->control_profile;
  state.control_reducer_config = impl_->owned_decision_components.control_reducer->GetConfig();
  state.control_reducer_state = impl_->owned_decision_components.control_reducer->GetRuntimeState();
  state.has_pending_internal_decision = impl_->has_pending_internal_decision;
  state.pending_internal_cycle_index = impl_->pending_internal_cycle_index;
  state.pending_internal_batch_id = impl_->pending_internal_batch_id;
  state.pending_internal_proposals = impl_->pending_internal_proposals;
  state.has_pending_external_override = impl_->has_pending_external_override;
  state.pending_external_override = impl_->pending_external_override;
  state.has_latest_decision_observation = impl_->has_latest_decision_observation;
  state.latest_decision_observation = impl_->latest_decision_observation;
  state.last_applied_decision_source = impl_->last_applied_decision_source;
  state.last_applied_decision_cycle_index = impl_->last_applied_decision_cycle_index;
  state.last_applied_decision_batch_id = impl_->last_applied_decision_batch_id;
  state.last_applied_decision_proposals = impl_->last_applied_decision_proposals;
  state.control_prepared_for_cycle = impl_->control_prepared_for_cycle;
  state.recognition_tracker_state = impl_->recognition_tracker.Capture();
  state.work_mode = impl_->work_mode;
  state.recognition_config = impl_->recognition_config;
  state.recognition_database_path = impl_->recognition_database_path;
  state.latest_recognition_summary = impl_->latest_recognition_summary;
  state.has_latest_recognition_summary = impl_->has_latest_recognition_summary;
  return state;
}

bool ArController::RestoreRuntimeState(const extension::ArControllerRuntimeState& state) {
  if (state.owner_identity != this || state.schema_version != 7U) {
    // 中译：控制器运行状态恢复被拒绝：所有者/结构不匹配。
    // 标识：回滚保护——归属/结构校验失败时拒绝恢复，防止错误状态写入。
    PROJECT_LOG_ERROR(
        "[ArController] controller runtime state restore rejected: "
        "owner/schema mismatch.");
    return false;
  }
  impl_->cycle_state.latest_output = state.latest_output;
  impl_->cycle_state.has_latest_output = state.has_latest_output;
  impl_->cycle_state.last_validation_issues = state.last_validation_issues;
  impl_->cycle_state.next_batch_id = state.next_batch_id;
  impl_->last_cycle_executed = state.last_cycle_executed;
  impl_->last_signal_abort_reason = state.last_signal_abort_reason;
  impl_->control_profile = state.control_profile;
  impl_->owned_decision_components.control_reducer->UpdateConfig(state.control_reducer_config);
  impl_->owned_decision_components.control_reducer->RestoreRuntimeState(
      state.control_reducer_state);
  impl_->has_pending_internal_decision = state.has_pending_internal_decision;
  impl_->pending_internal_cycle_index = state.pending_internal_cycle_index;
  impl_->pending_internal_batch_id = state.pending_internal_batch_id;
  impl_->pending_internal_proposals = state.pending_internal_proposals;
  impl_->has_pending_external_override = state.has_pending_external_override;
  impl_->pending_external_override = state.pending_external_override;
  impl_->has_latest_decision_observation = state.has_latest_decision_observation;
  impl_->latest_decision_observation = state.latest_decision_observation;
  impl_->last_applied_decision_source = state.last_applied_decision_source;
  impl_->last_applied_decision_cycle_index = state.last_applied_decision_cycle_index;
  impl_->last_applied_decision_batch_id = state.last_applied_decision_batch_id;
  impl_->last_applied_decision_proposals = state.last_applied_decision_proposals;
  impl_->control_prepared_for_cycle = state.control_prepared_for_cycle;
  impl_->recognition_tracker.Restore(state.recognition_tracker_state);
  impl_->work_mode = state.work_mode;
  impl_->recognition_config = state.recognition_config;
  // 数据库指针/路径回滚：路径与快照不一致时释放当前库，下次提交按路径重新加载。
  if (impl_->recognition_database_path != state.recognition_database_path) {
    impl_->recognition_database.reset();
    impl_->recognition_database_path.clear();
  }
  impl_->latest_recognition_summary = state.latest_recognition_summary;
  impl_->has_latest_recognition_summary = state.has_latest_recognition_summary;
  return true;
}

}  // namespace extension
}  // namespace airborne_radar
