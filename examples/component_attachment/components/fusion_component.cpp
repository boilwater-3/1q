/**
 * @file fusion_component.cpp
 * @brief 融合组件实现（多源聚合 + 态势差分）。
 *
 * 与行为层 recon_system 的融合段同构：聚合三传感器组件本周期探测记录
 * 一次 Update（同周期同时间戳，多源时间对齐即业务层职责）；新/消失
 * 目标按 key 集合差分（N 很小，O(N²) 即可；FusedTarget 按 key 升序）。
 * 每个融合目标经 World 信号发布 FusionUpdatedEvent（含通道样本构成）。
 */

#include "fusion_component.h"

#include <algorithm>
#include <utility>

#include "core/events.h"
#include "core/world.h"
#include "demo_log.h"
#include "ar_sensor_component.h"
#include "eos_sensor_component.h"
#include "esr_sensor_component.h"
#include "sbirs_sensor_component.h"
#include "sensor_utils.h"

namespace component_attachment {

FusionComponent::FusionComponent(std::unique_ptr<fusion::FusionEngine> engine)
    : engine_(std::move(engine)) {}

void FusionComponent::Step(World& world, double dt_sec) {
  (void)dt_sec;
  if (host_ == nullptr || engine_ == nullptr) {
    return;  // 未挂载或无引擎：无融合
  }

  // 周期内同步数据聚合：类型化访问同实体四个传感器组件的本周期探测。
  std::vector<fusion::DetectionRecord> all_detections;
  if (const auto* ar = host_->Find<ArSensorComponent>()) {
    all_detections.insert(all_detections.end(), ar->detections().begin(),
                          ar->detections().end());
  }
  if (const auto* esr = host_->Find<EsrSensorComponent>()) {
    all_detections.insert(all_detections.end(), esr->detections().begin(),
                          esr->detections().end());
  }
  if (const auto* eos = host_->Find<EosSensorComponent>()) {
    all_detections.insert(all_detections.end(), eos->detections().begin(),
                          eos->detections().end());
  }
  if (const auto* sbirs = host_->Find<SbirsSensorComponent>()) {
    all_detections.insert(all_detections.end(), sbirs->detections().begin(),
                          sbirs->detections().end());
  }

  const std::uint64_t cycle = world.scene_state().cycle;
  const std::vector<fusion::FusedTarget> fused = engine_->Update(all_detections, cycle);

  // 新/消失目标按键集合差分（对照上一周期态势）。
  std::size_t new_count = 0U;
  for (const auto& target : fused) {
    const auto it = std::find_if(targets_.begin(), targets_.end(),
                                 [&target](const fusion::FusedTarget& prev) {
                                   return prev.key == target.key;
                                 });
    if (it == targets_.end()) {
      ++new_count;
    }
  }
  std::size_t lost_count = 0U;
  for (const auto& prev : targets_) {
    const auto it = std::find_if(fused.begin(), fused.end(),
                                 [&prev](const fusion::FusedTarget& target) {
                                   return target.key == prev.key;
                                 });
    if (it == fused.end()) {
      ++lost_count;
    }
  }
  targets_ = fused;

  // 发布融合态势事件（每融合目标一条，携带通道样本构成与周期差分计数）。
  for (const auto& target : targets_) {
    FusionUpdatedEvent event;
    event.cycle = cycle;
    event.key = target.key;
    event.confidence = target.confidence;
    for (const auto& channel : target.channels) {
      event.channels.emplace_back(channel.source_id, channel.sample_count);
    }
    event.new_targets = new_count;
    event.lost_targets = lost_count;
    // 事件日志：字符串就地填充（日志宏 + 组件源文件内格式化串）。
    std::string channels;
    for (const auto& channel : event.channels) {
      if (!channels.empty()) channels += ",";
      channels += demo::Fmt("%u:%zu", channel.first, channel.second);
    }
    CA_LOG_EVENT(world, "fusion_updated", "key=%llu conf=%.2f new=%zu lost=%zu ch[%s]",
                 static_cast<unsigned long long>(event.key), event.confidence,
                 event.new_targets, event.lost_targets, channels.c_str());
    world.signals().on_fusion_updated(event);
  }
}

}  // namespace component_attachment
