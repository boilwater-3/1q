/**
 * @file threat_component.cpp
 * @brief 威胁评估组件实现（融合态势 + 运动学 → 威胁分/等级 + 升级事件）。
 *
 * 1. 以融合目标为主集合，AR 调试视图按键（association_key == FusedTarget.key）
 *    补充属性侧字段（速度/距离/RCS）；
 * 2. 示例层数据源无加速度与类型概率字段 → 按属性缺失（NaN）传入，评估器
 *    归一化 0 贡献（threat_assessment 边界语义，见 docs/threat_assessment/）；
 * 3. 等级升级（首见按低威胁计）→ 关键事件；每目标发布威胁更新信号；
 *    每周期视图摘要直写集成端日志。
 */

#include "threat_component.h"

#include <cmath>
#include <limits>

#include "1q/airborne_radar/session/ArTrackOutputDebugView.h"
#include "core/events.h"
#include "core/world.h"
#include "logger/logger.h"
#include "ar_sensor_component.h"
#include "fusion_component.h"

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

void ThreatComponent::Step(World& world, double dt_sec) {
  (void)dt_sec;
  if (host_ == nullptr) {
    return;  // 未挂载：无威胁评估
  }

  // 输入组装：融合目标为主集合（证据侧），AR 调试视图按键补充属性侧。
  const auto* fusion = host_->Find<FusionComponent>();
  const auto* ar = host_->Find<ArSensorComponent>();
  std::vector<threat_assessment::ThreatEvaluationInput> inputs;
  if (fusion != nullptr) {
    inputs.reserve(fusion->targets().size());
    const airborne_radar::session::ArTrackOutputDebugView& ar_view =
        ar != nullptr ? ar->LastDebugView() : airborne_radar::session::ArTrackOutputDebugView{};

    for (const fusion::FusedTarget& fused : fusion->targets()) {
      threat_assessment::ThreatEvaluationInput input;
      input.key = fused.key;
      input.fusion_confidence = static_cast<float>(fused.confidence);

      // 属性侧：AR 视图匹配（FusedTarget.key 源自 AR association_key 适配）。
      bool has_track = false;
      for (const auto& track : ar_view.tracks) {
        if (track.has_track && track.association_key == fused.key) {
          input.speed = track.speed;
          input.rcs = track.rcs;
          input.range_m = std::sqrt(track.position_x * track.position_x +
                                    track.position_y * track.position_y +
                                    track.position_z * track.position_z);
          has_track = true;
          break;
        }
      }
      // 无 AR 匹配：运动学属性按缺失传入（距离 0 是合法满分，缺失必须显式 NaN）。
      if (!has_track) {
        input.speed = std::numeric_limits<float>::quiet_NaN();
        input.range_m = std::numeric_limits<float>::quiet_NaN();
        input.rcs = std::numeric_limits<float>::quiet_NaN();
      }
      // 示例层数据源无加速度/类型概率字段：按属性缺失处理。
      input.acceleration = std::numeric_limits<float>::quiet_NaN();
      input.target_probability = std::numeric_limits<float>::quiet_NaN();

      inputs.push_back(input);
    }
  }

  // 评估（纯函数式；无输入时输出为空）。
  results_ = evaluator_.Evaluate(inputs);
  high_threat_count_ = 0U;
  for (const threat_assessment::ThreatResult& result : results_) {
    if (result.level == threat_assessment::ThreatLevel::kHigh) {
      ++high_threat_count_;
    }
  }

  // 等级升级判定（首见按低威胁计）+ 威胁更新事件发布。
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
    event.result = result;
    world.signals().on_threat_updated(event);
  }

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
        std::to_string(level_up_count);
    CA_LOG_VIEW("threat", "周期={} 目标={} 高={} 中={} 低={} 最高=键{}:{:.2f} 升级={}",
                world.scene_state().cycle, results_.size(), high_threat_count_,
                medium_count, low_count, static_cast<unsigned long long>(top_key),
                top_score, level_up_count);
  }
}

}  // namespace component_attachment
