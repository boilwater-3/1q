/**
 * @file RirController.cpp
 * @brief 远程识别雷达识别链路控制器实现。
 */

#include "remote_identification_radar/runtime/RirController.h"

#include <algorithm>
#include <cmath>

#include "common/logging/ProjectLog.h"
#include "common/numerics/Constants.h"
#include "remote_identification_radar/internal/RirRadarEquations.h"
#include "remote_identification_radar/recognition/RecognitionObservationBuilder.h"

namespace remote_identification_radar {
namespace runtime {

void RirController::UpdateRuntime(config::RirWorkMode work_mode,
                                  const config::RirRecognitionPolicy& recognition_config) {
  work_mode_ = work_mode;
  recognition_config_ = recognition_config;

  recognition::RirTracker::Options options;
  options.min_confirmed_hits = recognition_config_.min_confirmed_hits;
  options.accumulation_window_sec = recognition_config_.accumulation_window_sec;
  options.min_observation_count = recognition_config_.min_observation_count;
  options.acceptance_score = recognition_config_.acceptance_score;
  options.minimum_margin = recognition_config_.minimum_margin;
  options.result_hold_sec = recognition_config_.result_hold_sec;
  options.max_range_m = recognition_config_.max_range_m;
  tracker_.SetOptions(options);

  // 数据库按需加载：路径变化且已启用时重新加载；失败保持原库（识别降级 kDisabled）。
  if (recognition_config_.enabled && !recognition_config_.database_path.empty() &&
      recognition_config_.database_path != database_path_) {
    std::unique_ptr<recognition::RirFeatureDatabase> candidate(
        new recognition::RirFeatureDatabase());
    std::string error;
    if (recognition::RirFeatureDatabase::Load(recognition_config_.database_path, candidate.get(),
                                              &error)) {
      database_ = std::move(candidate);
      database_path_ = recognition_config_.database_path;
      tracker_.SetActiveDatabaseVersion(database_->version());
    } else {
      // 中译：识别特征数据库加载失败，本次更新不生效，识别链路保持当前库或 kDisabled 降级。
      // 标识：RirController::UpdateRuntime，数据库路径/格式错误时触发；不影响本模块其他能力。
      PROJECT_LOG_ERROR("[RirController] recognition database load failed: {}", error);
    }
  }
}

float RirController::ComputeSnrDb(float rcs_m2, float range_m) const {
  const float echo_dBW = internal::RirRadarEquations::ComputeEchoPower_dBW(
      hardware_.transmitter, hardware_.antenna, rcs_m2, range_m, 0.0f);
  const float noise_w = internal::RirRadarEquations::ComputeThermalNoisePower_W(
      hardware_.transmitter, hardware_.receiver);
  return echo_dBW - 10.0f * std::log10(std::max(1.0e-12f, noise_w));
}

recognition::RirObservationContext RirController::MakeObservationContext(
    const session::RirSceneTarget& target, float platform_altitude_m) const {
  recognition::RirObservationContext context;
  context.snr_db = ComputeSnrDb(target.rcs, target.range_m);
  context.range_m = target.range_m;
  context.bandwidth_hz = hardware_.transmitter.bandwidth_hz;
  context.dwell_sec = recognition_config_.recognition_dwell_sec;
  const float range_hypot =
      std::sqrt(target.position_x * target.position_x + target.position_y * target.position_y);
  context.look_az_deg = oneq::common::numerics::RadToDeg(
      std::atan2(target.position_y, target.position_x));
  context.look_el_deg = oneq::common::numerics::RadToDeg(
      std::atan2(target.position_z, range_hypot));
  context.platform_altitude_m = platform_altitude_m;
  // 观测有效性不由硬编码视角覆盖下限门控：覆盖下限属数据库 profile 级适用条件。
  context.minimum_aspect_coverage_deg = 0.0f;
  return context;
}

void RirController::RunCycle(const session::RirCycleInput& input,
                             session::RirOutputFrame* output_frame) {
  const bool in_identify = work_mode_ == config::RirWorkMode::kIdentify;
  // 模式切换：退出 kIdentify → 清空积累（结论进入保持期）。
  if (recognition_mode_active_ && !in_identify) {
    tracker_.ExitRecognitionMode();
  }
  recognition_mode_active_ = in_identify;
  sim_time_sec_ = input.sim_time_sec;

  if (in_identify && database_ != nullptr) {
    // 观测输入：external_target_id → 场景目标 → association_key 映射。
    std::unordered_map<std::uint64_t, recognition::RirTracker::TrackObservationInput>
        observations_by_target_id;
    for (std::size_t i = 0U; i < input.scene_targets.size(); ++i) {
      if (input.scene_targets[i].external_target_id == 0U) {
        continue;
      }
      recognition::RirTracker::TrackObservationInput observation;
      observation.target = &input.scene_targets[i];
      observation.context =
          MakeObservationContext(input.scene_targets[i], input.platform_altitude_m);
      observations_by_target_id[input.scene_targets[i].external_target_id] = observation;
    }
    std::unordered_map<std::uint64_t, recognition::RirTracker::TrackObservationInput>
        observations_by_key;
    for (std::size_t i = 0U; i < input.track_feed.size(); ++i) {
      const session::RirTrackFeedEntry& track = input.track_feed[i];
      const std::unordered_map<
          std::uint64_t, recognition::RirTracker::TrackObservationInput>::const_iterator found =
          observations_by_target_id.find(track.external_target_id);
      if (found != observations_by_target_id.end()) {
        observations_by_key[track.association_key] = found->second;
      }
    }
    tracker_.UpdateCycle(input.track_feed, observations_by_key, *database_,
                         recognition_config_.feature_weights, sim_time_sec_,
                         input.input_cycle_index, input.batch_id);
  } else {
    tracker_.HoldCycle(input.track_feed, sim_time_sec_);
  }

  // 回填：逐航迹识别结论。
  output_frame->recognition_outputs.clear();
  output_frame->recognition_outputs.reserve(input.track_feed.size());
  for (std::size_t i = 0U; i < input.track_feed.size(); ++i) {
    session::RirTrackRecognitionOutput output;
    output.association_key = input.track_feed[i].association_key;
    const session::RirRecognitionResult* result =
        tracker_.FindResult(input.track_feed[i].association_key);
    if (result != nullptr) {
      output.result = *result;
    }
    output_frame->recognition_outputs.push_back(output);
  }
  latest_summary_ = tracker_.BuildSummary(input.track_feed);
  has_latest_summary_ = true;
}

}  // namespace runtime
}  // namespace remote_identification_radar
