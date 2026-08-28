/**
 * @file threat_component.cpp
 * @brief 威胁评估组件实现（融合态势 + 运动学 → 威胁分/等级 + 升级事件）。
 *
 * 1. 订阅融合态势事件（证据侧主集合）与 AR 航迹状态事件（属性侧补充：
 *    速度/距离/RCS，association_key == FusedTarget.key 对齐），逐目标缓存
 *    ——不调用融合/AR 组件方法（集成方同走事件机制）；
 * 2. 示例层数据源无加速度与类型概率字段 → 按属性缺失（NaN）传入，评估器
 *    归一化 0 贡献（threat_assessment 边界语义，见 docs/threat_assessment/）；
 * 3. 等级升级（首见按低威胁计）→ 关键事件；每目标发布威胁更新信号（事件
 *    为集成契约，库内结果展平为镜像枚举/数值字段）；每周期视图摘要直写
 *    集成端日志。
 */

#include "threat_component.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "core/events.h"
#include "core/world.h"
#include "logger/logger.h"

namespace component_attachment {

namespace {

/** @brief 威胁等级中文人读名（事件与视图行用）。 */
const char* ThreatLevelName(threat_assessment::ThreatLevel level) {
  switch (level) {
    case threat_assessment::ThreatLevel::kHigh:
      return "高";
    case threat_assessment::ThreatLevel::kMedium:
      return "中";
    default:
      return "低";
  }
}

}  // namespace

ThreatComponent::ThreatComponent(const threat_assessment::ThreatEvaluatorConfig& config)
    : evaluator_(config) {}

void ThreatComponent::OnFusionUpdated(const FusionUpdatedEvent& event) {
  FusionSnapshot& snapshot = fusion_by_key_[event.key];
  snapshot.cycle = event.cycle;
  snapshot.confidence = event.confidence;
}

void ThreatComponent::OnArTrackState(const ArTrackStateEvent& event) {
  ArTrackSnapshot& snapshot = ar_tracks_by_key_[event.association_key];
  snapshot.cycle = event.cycle;
  snapshot.speed_m_per_s = event.speed_m_per_s;
  snapshot.rcs_m2 = event.rcs_m2;
  snapshot.position_x_m = event.position_x_m;
  snapshot.position_y_m = event.position_y_m;
  snapshot.position_z_m = event.position_z_m;
}

std::vector<threat_assessment::ThreatEvaluationInput> ThreatComponent::BuildEvaluationInputs(
    std::uint64_t cycle) const {
  // 输入组装：融合态势缓存为主集合（证据侧，本周期新鲜条目），AR 航迹缓存
  // 按键补充属性侧（AR/融合组件挂载序在前，同周期事件先于本组件 Step 到达）。
  std::vector<std::uint64_t> keys;  // key 升序（与融合态势输出序一致，评估序确定）
  keys.reserve(fusion_by_key_.size());
  for (const auto& entry : fusion_by_key_) {
    keys.push_back(entry.first);
  }
  std::sort(keys.begin(), keys.end());

  std::vector<threat_assessment::ThreatEvaluationInput> inputs;
  for (const std::uint64_t key : keys) {
    const FusionSnapshot& fused = fusion_by_key_.at(key);
    if (fused.cycle != cycle) {
      continue;  // 本周期融合未发布（目标已消失）：非评估输入
    }
    threat_assessment::ThreatEvaluationInput input;
    input.key = key;
    input.fusion_confidence = static_cast<float>(fused.confidence);

    // 属性侧：AR 航迹缓存按键匹配（FusedTarget.key 源自 AR association_key 适配）。
    const auto track_it = ar_tracks_by_key_.find(key);
    if (track_it != ar_tracks_by_key_.end() && track_it->second.cycle == cycle) {
      const ArTrackSnapshot& track = track_it->second;
      input.speed = static_cast<float>(track.speed_m_per_s);
      input.rcs = static_cast<float>(track.rcs_m2);
      input.range_m = std::sqrt(track.position_x_m * track.position_x_m +
                                track.position_y_m * track.position_y_m +
                                track.position_z_m * track.position_z_m);
    } else {
      // 无 AR 匹配：运动学属性按缺失传入（距离 0 是合法满分，缺失必须显式 NaN）。
      input.speed = std::numeric_limits<float>::quiet_NaN();
      input.range_m = std::numeric_limits<float>::quiet_NaN();
      input.rcs = std::numeric_limits<float>::quiet_NaN();
    }
    // 示例层数据源无加速度/类型概率字段：按属性缺失处理。
    input.acceleration = std::numeric_limits<float>::quiet_NaN();
    input.target_probability = std::numeric_limits<float>::quiet_NaN();

    inputs.push_back(input);
  }
  return inputs;
}

void ThreatComponent::EnsureSignalConnections(World& world) {
  if (!fusion_connection_.connected()) {
    fusion_connection_ = world.signals().on_fusion_updated.connect(
        [this](const FusionUpdatedEvent& event) { OnFusionUpdated(event); });
  }
  if (!ar_track_connection_.connected()) {
    ar_track_connection_ = world.signals().on_ar_track_state.connect(
        [this](const ArTrackStateEvent& event) { OnArTrackState(event); });
  }
}

void ThreatComponent::Step(World& world, double dt_sec) {
  (void)dt_sec;
  EnsureSignalConnections(world);

  const std::vector<threat_assessment::ThreatEvaluationInput> inputs =
      BuildEvaluationInputs(world.scene_state().cycle);
  // 评估（纯函数式；无输入时输出为空）。
  results_ = evaluator_.Evaluate(inputs);
  high_threat_count_ = 0U;
  for (const threat_assessment::ThreatResult& result : results_) {
    if (result.level == threat_assessment::ThreatLevel::kHigh) {
      ++high_threat_count_;
    }
  }

  PublishThreatEvents(world);
  LogThreatView(world);
}

void ThreatComponent::PublishThreatEvents(World& world) {
  // 等级升级判定（首见按低威胁计）+ 威胁更新事件发布（库内结果展平为
  // 集成契约字段：镜像枚举 + 贡献分解数值）。
  std::size_t level_up_count = 0U;
  for (const threat_assessment::ThreatResult& result : results_) {
    const auto prev = prev_levels_.find(result.key);
    const threat_assessment::ThreatLevel prev_level =
        prev != prev_levels_.end() ? prev->second : threat_assessment::ThreatLevel::kLow;
    prev_levels_[result.key] = result.level;

    const bool upgraded = result.level > prev_level;
    if (upgraded) {
      ++level_up_count;
      // 中译：目标键 {} 威胁等级升级为{}（威胁分 {:.2f}）。
      // 标识：威胁升级关键事件——等级相对上一周期上升（含首见即高威胁），
      //       战术决策侧的触发信号。
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string threat_level_up_event_log =
          std::string("键=") +
          std::to_string(static_cast<unsigned long long>(result.key)) +
          " 等级=" +
          (ThreatLevelName(result.level)) +
          " 威胁分=" +
          std::to_string(result.threat_score) +
          " 贡献=[距" +
          std::to_string(result.contributions.range) +
          " 速" +
          std::to_string(result.contributions.speed) +
          " RCS" +
          std::to_string(result.contributions.rcs) +
          " 融" +
          std::to_string(result.contributions.fusion_confidence) +
          "]";
      CA_LOG_EVENT(world, "threat_level_up",
                   "键={} 等级={} 威胁分={:.2f} 贡献=[距{:.2f} 速{:.2f} RCS{:.2f} 融{:.2f}]",
                   static_cast<unsigned long long>(result.key),
                   ThreatLevelName(result.level), result.threat_score,
                   result.contributions.range, result.contributions.speed,
                   result.contributions.rcs, result.contributions.fusion_confidence);
    }

    ThreatUpdatedEvent event;
    event.cycle = world.scene_state().cycle;
    event.key = result.key;
    event.threat_score = result.threat_score;
    event.level = static_cast<EventThreatLevel>(result.level);
    event.contribution_range = result.contributions.range;
    event.contribution_speed = result.contributions.speed;
    event.contribution_acceleration = result.contributions.acceleration;
    event.contribution_rcs = result.contributions.rcs;
    event.contribution_target_probability = result.contributions.target_probability;
    event.contribution_fusion_confidence = result.contributions.fusion_confidence;
    world.signals().on_threat_updated(event);
  }

  last_level_up_count_ = level_up_count;
}

void ThreatComponent::LogThreatView(World& world) {
  // 视图摘要行（每周期一行；默认跨周期增量模式下仍恒写——威胁态势每周期变化）。
  std::size_t medium_count = 0U;
  std::size_t low_count = 0U;
  std::uint64_t top_key = 0U;
  float top_score = -1.0f;
  for (const threat_assessment::ThreatResult& result : results_) {
    if (result.level == threat_assessment::ThreatLevel::kMedium) {
      ++medium_count;
    } else if (result.level == threat_assessment::ThreatLevel::kLow) {
      ++low_count;
    }
    if (result.threat_score > top_score) {
      top_score = result.threat_score;
      top_key = result.key;
    }
  }
  if (results_.empty()) {
    // 中译：威胁评估视图：无融合目标。
    // 标识：空周期视图行——无目标时威胁态势为空，正常分支。
    // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
    const std::string threat_view_log =
        std::string("周期=") +
        std::to_string(world.scene_state().cycle) +
        " 目标=0 高=0 中=0 低=0";
    CA_LOG_VIEW("threat", "周期={} 目标=0 高=0 中=0 低=0",
                world.scene_state().cycle);
  } else {
    // 中译：威胁评估视图摘要：目标数/等级分布/最高威胁目标。
    // 标识：每周期威胁态势快照——等级分布与最高威胁目标（键+分），
    //       供预期表核对威胁分排序与等级映射。
    // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
    const std::string threat_view_log_2 =
        std::string("周期=") +
        std::to_string(world.scene_state().cycle) +
        " 目标=" +
        std::to_string(results_.size()) +
        " 高=" +
        std::to_string(high_threat_count_) +
        " 中=" +
        std::to_string(medium_count) +
        " 低=" +
        std::to_string(low_count) +
        " 最高=键" +
        std::to_string(static_cast<unsigned long long>(top_key)) +
        ":" +
        std::to_string(top_score) +
        " 升级=" +
        std::to_string(last_level_up_count_);
    CA_LOG_VIEW("threat", "周期={} 目标={} 高={} 中={} 低={} 最高=键{}:{:.2f} 升级={}",
                world.scene_state().cycle, results_.size(), high_threat_count_,
                medium_count, low_count, static_cast<unsigned long long>(top_key),
                top_score, last_level_up_count_);
  }
}

}  // namespace component_attachment
